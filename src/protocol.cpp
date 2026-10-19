// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith

#include "protocol.h"

#include "commands.h"
#include "fmt.h"
#include "line.h"
#include "parse.h"
#include "report.h"
#include "serial.h"

namespace protocol {

namespace {

using commands::Status;

constexpr uint8_t kQueueSize = 20;  // faithful PACK_MAX_SIZE (uarm_protocol.h:9)
constexpr uint8_t kTailMax = 48;    // longest tail: P2246, about 40 characters
constexpr uint8_t kTextMax = 3 + kTailMax;  // code + tail; also holds the line body (<= 48) until it runs
// "$" seq " " text "\n"
constexpr uint8_t kReplyMax = 1 + parse::kMaxSeqDigits + 1 + kTextMax + 1;
constexpr uint8_t kEventMax = 24;  // the longest, "@8 N1 V-123.4567", with "\n" and NUL
static_assert(line::kMaxLength <= kTextMax, "a queued line must fit the entry text");

enum class State : uint8_t { kWaiting, kDone };

struct Entry {
    uint8_t port;             // the serial port the command came from; its reply goes back there (R-6)
    char seq[parse::kMaxSeqDigits + 1];
    char text[kTextMax + 1];  // the line body while kWaiting, code + tail when kDone
    State state;
    bool motion;              // kDone: a motion command (its reply may be streamed, Q-32)
};

Entry queue[kQueueSize];
uint8_t head = 0;     // oldest entry, the next to reply
uint8_t count = 0;    // entries in use (waiting to run or waiting to reply; SP-53)
uint8_t started = 0;  // entries from head that have run (they run in order)

line::Assembler assembler[serial::kPorts];  // one per port (R-6)
uint8_t next_port = 0;                      // ports take turns supplying lines

// Commands that returned kPending. One P/S command (async: an encoder query) and one G/M command (sync: a re-sync,
// a line check, a delay) may be pending at the same time. While the async one is pending no line is taken; while the
// sync one is pending no other sync command starts. Queries keep being answered during a long sync job.
bool async_pending = false;
uint8_t async_port = 0;
char async_seq[parse::kMaxSeqDigits + 1];
char async_tail_buf[kTailMax + 1];
fmt::Writer async_tail(async_tail_buf, sizeof async_tail_buf);
bool sync_pending = false;
bool sync_discard = false;  // the sync command was aborted (S1100, Ctrl-X): drop its result
char sync_tail_buf[kTailMax + 1];
fmt::Writer sync_tail(sync_tail_buf, sizeof sync_tail_buf);

uint8_t slot(uint8_t i) { return static_cast<uint8_t>((head + i) % kQueueSize); }

const fmt::FlashText* code_text(Status s) {
    switch (s) {
        case Status::kOk: return FLASH("ok");
        case Status::kE20: return FLASH("E20");
        case Status::kE21: return FLASH("E21");
        case Status::kE22: return FLASH("E22");
        case Status::kE25: return FLASH("E25");
        case Status::kE26: return FLASH("E26");
        case Status::kPending: break;  // never sent: a pending command has no reply yet
    }
    return FLASH("E20");
}

// Puts a reply code into an entry's text.
void set_code(char* dst, Status s) {
    fmt::Writer w(dst, kTextMax + 1);
    w.put(code_text(s));
}

void copy(char* dst, const char* src, uint8_t cap) {
    uint8_t i = 0;
    for (; i + 1 < cap && src[i] != '\0'; ++i) dst[i] = src[i];
    dst[i] = '\0';
}

// Writes "$seq text\n" or "text\n" to port. The caller has checked that kReplyMax bytes are free for it.
void emit(uint8_t port, const char* seq, const char* text) {
    char buf[kReplyMax + 1];
    fmt::Writer w(buf, sizeof buf);
    if (seq[0] != '\0') {
        w.put('$');
        w.put(seq);
        w.put(' ');
    }
    w.put(text);
    w.put('\n');
    serial::write(port, w.text(), w.length());
}

// Code and tail of a finished command into dst (kTextMax + 1 bytes).
void result_text(Status s, const fmt::Writer& tail, char* dst) {
    fmt::Writer w(dst, kTextMax + 1);
    if (tail.overflow()) {  // a reply that doesn't fit is an operation failure, never a truncated line
        w.put(code_text(Status::kE25));
        return;
    }
    w.put(code_text(s));
    w.put(tail.text());
}

// Maps a command-word problem to its reply (SP-11, SP-12, Q-53).
Status word_status(parse::Word w) {
    switch (w) {
        case parse::Word::kOk: return Status::kOk;
        case parse::Word::kBadNumber: return Status::kE21;
        case parse::Word::kNotInteger: return Status::kE21;
        case parse::Word::kOutOfRange: return Status::kE20;
    }
    return Status::kE20;
}

// S1100 and Ctrl-X: every sync command still waiting to run or to reply gets E25 (SP-05, Q-4, Q-46).
void abort_all() {
    if (sync_pending) sync_discard = true;
    for (uint8_t i = 0; i < count; ++i) {
        Entry& e = queue[slot(i)];
        set_code(e.text, Status::kE25);
        e.state = State::kDone;
        e.motion = false;
    }
    started = count;
}

void run_async(uint8_t port, const parse::Routed& r) {
    parse::Command cmd{};
    const parse::Word w = parse::command(r.body, &cmd);
    char text[kTextMax + 1];
    if (w != parse::Word::kOk) {
        set_code(text, word_status(w));
        emit(port, r.seq, text);
        return;
    }
    if (cmd.family == 'S' && cmd.number == 1100) {  // uarm_protocol.c:467-470; the rest of the line is ignored
        commands::stop();
        abort_all();
        set_code(text, Status::kOk);
        emit(port, r.seq, text);
        return;
    }
    async_tail = fmt::Writer(async_tail_buf, sizeof async_tail_buf);
    const Status s = commands::execute(cmd, async_tail);
    if (s == Status::kPending) {
        async_pending = true;
        async_port = port;
        copy(async_seq, r.seq, sizeof async_seq);
        return;
    }
    result_text(s, async_tail, text);
    emit(port, r.seq, text);
}

void enqueue(uint8_t port, const parse::Routed& r) {
    if (count == kQueueSize) {  // SP-13, SP-53: the original printed "E23 buffer full\r\n" without $N
        emit(port, r.seq, "E23");
        return;
    }
    Entry& e = queue[slot(count)];
    e.port = port;
    copy(e.seq, r.seq, sizeof e.seq);
    copy(e.text, r.body, sizeof e.text);
    e.state = State::kWaiting;
    e.motion = false;
    ++count;
}

void handle_line(uint8_t port, line::Result result) {
    if (result == line::Result::kLost) {  // SP-01, Q-3: the line's #N may be among the lost bytes
        emit(port, "", "E23");
        return;
    }
    const parse::Routed r = parse::route(assembler[port].text());
    if (result == line::Result::kTooLong) {  // Q-22
        emit(port, r.route == parse::Route::kBadSequence ? "" : r.seq, "E21");
        return;
    }
    switch (r.route) {
        case parse::Route::kIgnore: return;                       // faithful uarm_protocol.c:39-41
        case parse::Route::kSystem: emit(port, "", "E20"); return;         // Q-36, Q-42
        case parse::Route::kNotCommand: emit(port, r.seq, "E20"); return;  // SP-52
        case parse::Route::kBadSequence: emit(port, "", "E21"); return;    // SP-03
        case parse::Route::kAsync: run_async(port, r); return;
        case parse::Route::kSync: {
            // Q-56: M2200 ("is moving?") is answered at once, like a query.
            parse::Command cmd{};
            if (parse::command(r.body, &cmd) == parse::Word::kOk && cmd.family == 'M' && cmd.number == 2200) {
                run_async(port, r);
                return;
            }
            enqueue(port, r);
            return;
        }
    }
}

// Takes at most one line per pass (the ports take turns), and only from a port whose reply will fit.
bool take_line() {
    bool progress = false;
    for (uint8_t k = 0; k < serial::kPorts; ++k) {
        const uint8_t port = next_port;
        next_port = static_cast<uint8_t>((next_port + 1) % serial::kPorts);
        if (serial::tx_room(port) < kReplyMax) continue;
        uint8_t c;
        while (serial::read(port, &c)) {
            progress = true;
            const line::Result result = assembler[port].feed(c);
            if (result != line::Result::kNone) {
                handle_line(port, result);
                return true;
            }
        }
    }
    return progress;
}

// Starts the oldest sync command that hasn't run, if the machine allows it (uarm_protocol.c:144-165).
bool start_next() {
    if (started == count) return false;
    Entry& e = queue[slot(started)];
    parse::Command cmd{};
    const parse::Word w = parse::command(e.text, &cmd);
    const commands::Kind k = w == parse::Word::kOk ? commands::kind(cmd) : commands::Kind::kNormal;
    if (!commands::can_start(k)) return false;
    sync_tail = fmt::Writer(sync_tail_buf, sizeof sync_tail_buf);
    const Status s = w == parse::Word::kOk ? commands::execute(cmd, sync_tail) : word_status(w);
    e.motion = k == commands::Kind::kMotion;
    if (s == Status::kPending) {
        sync_pending = true;
        sync_discard = false;
        return true;
    }
    result_text(s, sync_tail, e.text);  // overwrites the line, which is no longer needed
    e.state = State::kDone;
    ++started;
    return true;
}

// Completes pending commands that have finished.
bool resume_async() {
    if (!async_pending || serial::tx_room(async_port) < kReplyMax) return false;  // its reply must fit
    const Status s = commands::resume_async(async_tail);
    if (s == Status::kPending) return false;
    char text[kTextMax + 1];
    result_text(s, async_tail, text);
    emit(async_port, async_seq, text);
    async_pending = false;
    return true;
}

bool resume_sync() {
    if (!sync_pending) return false;
    const Status s = commands::resume_sync(sync_tail);
    if (s == Status::kPending) return false;
    if (!sync_discard) {
        Entry& e = queue[slot(started)];
        result_text(s, sync_tail, e.text);
        e.state = State::kDone;
        ++started;
    }
    sync_pending = false;
    return true;
}

// The @3 report, after the events (uarm_protocol.c:187-190). It is sent only if a reply still fits after it, else
// skipped and counted (Q-21), so reports never hold up replies. Not inlined: its buffer stays off the stack of the
// motion path (docs/ARCHITECTURE.md §11).
__attribute__((noinline)) void send_report(uint8_t port) {
    char buf[report::kMaxLength];
    fmt::Writer w(buf, sizeof buf);
    report::write(w);
    w.put('\n');
    if (!w.overflow() && serial::tx_room(port) >= static_cast<uint16_t>(w.length() + kReplyMax))
        serial::write(port, w.text(), w.length());
}

// Sends finished sync replies in FIFO order (uarm_protocol.c:175-182; Q-32 for streamed motion replies).
bool pump() {
    bool progress = false;
    while (count > 0) {
        Entry& e = queue[head];
        if (e.state != State::kDone) break;
        const bool allowed = commands::idle() || (commands::streaming() && e.motion);
        if (!allowed || serial::tx_room(e.port) < kReplyMax) break;
        emit(e.port, e.seq, e.text);
        head = static_cast<uint8_t>((head + 1) % kQueueSize);
        --count;
        --started;
        progress = true;
    }
    // Events after the replies, one per pass (uarm_protocol.c:187-221). The event is taken only when it will fit.
    const uint8_t port = serial::last_port();
    if (serial::tx_room(port) >= kEventMax) {
        char buf[kEventMax];
        fmt::Writer w(buf, sizeof buf);
        if (commands::next_event(w)) {
            w.put('\n');
            serial::write(port, w.text(), w.length());
            progress = true;
        }
    }
    if (report::due()) {
        send_report(port);
        progress = true;
    }
    return progress;
}

}  // namespace

void init() {
    async_pending = false;
    sync_pending = false;
    sync_discard = false;
    head = 0;
    count = 0;
    started = 0;
    next_port = 0;
    for (line::Assembler& a : assembler) a = line::Assembler();
}

bool poll() {
    bool progress = false;
    if (serial::take_stop_request()) {  // Ctrl-X (Q-48): like S1100, without a reply of its own
        commands::stop();
        abort_all();
        progress = true;
    }
    progress = commands::poll() || progress;
    if (commands::take_fault()) {  // lost steps (SP-41): as S1100; the moving command gets E25
        commands::stop();
        abort_all();
        progress = true;
    }
    progress = resume_async() || progress;
    progress = resume_sync() || progress;
    if (!async_pending) progress = take_line() || progress;
    // Replies (and @9) before the next sync command starts, as the original: a move's reply and its "@9 V0" come
    // before anything the next command prints (uarm_protocol.c:167-221, then :144-165 on the next pass).
    progress = pump() || progress;
    if (!sync_pending) progress = start_next() || progress;
    return progress;
}

bool queue_empty() { return count == 0; }

}  // namespace protocol
