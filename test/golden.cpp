// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Golden-trace runner. Host only (uses the standard library).
//
//   golden [--lint] <trace files...>
//
// Every trace is linted, then run against the simulator. --lint only lints (for traces whose modules don't exist
// yet); those are reported PENDING.
//
// Trace syntax, one directive per line:
//   # text            comment
//   #: <tag> <text>   provenance of the next command and its expected output. Tags:
//                       faithful <file:line>       behaviour of the original, VERIFIED in code
//                       measured <log> <id>        seen on the arm
//                       change SP-nn|Q-mm|R-n ...  approved change to the original (R-n: internal improvement)
//                       new SP-nn|Q-mm|R-n ...     approved addition (not in the original)
//                       pending SP-nn              faithful for now; SP-nn awaits a decision
//   = <key> <value>   set a simulated input (keys in sim.h)
//   ~ boot            power on; the banner must be matched
//   ~ boot quiet      power on, run 500 ms (past the 260 ms boot beep), discard all output
//   ~ ms <n>          advance simulated time
//   ~ idle            advance until motion and the sync queue are idle (fails after 60 s)
//   > <text>          send <text> and "\n", then settle
//   >raw <text>       send <text> exactly (escapes \r \n \t \\ \xNN), then settle
//   < <text>          the next output line equals <text> (without its "\n"; escapes as above)
//   <re <regex>       the next output line matches <regex> (ECMAScript, whole line)
// Before every '=', '~', '>' and at the end of the trace, all output so far must have been matched.
// Every '<' needs at least one tag above it in the same block. A block ends at a blank line.

#include "sim.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <regex>
#include <string>
#include <vector>

namespace {

struct Line {
    int number;
    std::string text;
};

struct Result {
    int errors = 0;
    int tags_faithful = 0, tags_measured = 0, tags_change = 0, tags_new = 0, tags_pending = 0;
};

const char* const kKeys[] = {"hw",    "uid",   "mode",  "enc.ref",  "enc.raw",     "enc.status", "enc.nack",
                             "enc.stall", "enc.slip", "power", "limit", "pump.adc", "gripper.pa0", "button",
                             "tool.switch"};

bool starts_with(const std::string& s, const char* p) { return s.rfind(p, 0) == 0; }

// Decode \r \n \t \\ \xNN. Returns false on a bad escape.
bool unescape(const std::string& in, std::string& out) {
    out.clear();
    for (size_t i = 0; i < in.size(); ++i) {
        if (in[i] != '\\') { out += in[i]; continue; }
        if (++i == in.size()) return false;
        switch (in[i]) {
            case 'r': out += '\r'; break;
            case 'n': out += '\n'; break;
            case 't': out += '\t'; break;
            case '\\': out += '\\'; break;
            case 'x': {
                if (i + 2 >= in.size()) return false;
                const std::string hex = in.substr(i + 1, 2);
                char* end = nullptr;
                const long v = std::strtol(hex.c_str(), &end, 16);
                if (end != hex.c_str() + 2) return false;
                out += static_cast<char>(v);
                i += 2;
                break;
            }
            default: return false;
        }
    }
    return true;
}

std::string visible(const std::string& s) {
    std::string out;
    for (const char c : s) {
        if (c == '\r') out += "\\r";
        else if (c == '\n') out += "\\n";
        else if (static_cast<unsigned char>(c) < 0x20 || static_cast<unsigned char>(c) > 0x7e) {
            char buf[8];
            std::snprintf(buf, sizeof buf, "\\x%02X", static_cast<unsigned char>(c));
            out += buf;
        } else out += c;
    }
    return out;
}

class Runner {
public:
    Runner(const char* file, bool run) : file_(file), run_(run) {}

    Result go(const std::vector<Line>& lines) {
        if (run_) sim_reset();
        for (const Line& l : lines) step(l);
        if (run_) drain_check(lines.empty() ? 0 : lines.back().number);
        return r_;
    }

private:
    const char* file_;
    bool run_;
    Result r_;
    int tags_ = 0;  // tags seen in the current block (a blank line ends a block)
    std::string out_;       // output not yet matched

    void error(int n, const std::string& msg) {
        std::printf("%s:%d: %s\n", file_, n, msg.c_str());
        ++r_.errors;
    }

    void pull() { out_ += sim_take_tx(); }

    void drain_check(int n) {
        pull();
        if (!out_.empty()) {
            error(n, "unexpected output: " + visible(out_));
            out_.clear();
        }
    }

    void start_group(int n) {
        if (run_) drain_check(n);
    }

    void tag(const Line& l) {
        const size_t start = l.text.find_first_not_of(' ', 2);
        const std::string w = start == std::string::npos ? "" : l.text.substr(start);
        if (starts_with(w, "faithful ")) ++r_.tags_faithful;
        else if (starts_with(w, "measured ")) ++r_.tags_measured;
        else if (starts_with(w, "change SP-") || starts_with(w, "change Q-") || starts_with(w, "change R-")) ++r_.tags_change;
        else if (starts_with(w, "new SP-") || starts_with(w, "new Q-") || starts_with(w, "new R-")) ++r_.tags_new;
        else if (starts_with(w, "pending SP-")) ++r_.tags_pending;
        else { error(l.number, "unknown tag: " + w); return; }
        ++tags_;
    }

    bool next_line(int n, std::string& line) {
        pull();
        const size_t nl = out_.find('\n');
        if (nl == std::string::npos) {
            error(n, out_.empty() ? "expected a line, got no output" : "expected a line, got partial: " + visible(out_));
            out_.clear();
            return false;
        }
        line = out_.substr(0, nl);
        out_.erase(0, nl + 1);
        return true;
    }

    void expect(const Line& l, bool is_regex) {
        if (tags_ == 0) error(l.number, "expected output without a provenance tag");
        const std::string body = l.text.substr(is_regex ? 4 : 2);
        if (is_regex) {
            std::regex re;
            try { re = std::regex(body, std::regex::ECMAScript); }
            catch (const std::regex_error&) { error(l.number, "bad regex"); return; }
            std::string got;
            if (run_ && next_line(l.number, got) && !std::regex_match(got, re))
                error(l.number, "got  " + visible(got) + "\n    want /" + body + "/");
            return;
        }
        std::string want;
        if (!unescape(body, want)) { error(l.number, "bad escape"); return; }
        std::string got;
        if (run_ && next_line(l.number, got) && got != want)
            error(l.number, "got  " + visible(got) + "\n    want " + visible(want));
    }

    void set(const Line& l) {
        const std::string t = l.text.substr(2);
        const size_t sp = t.find(' ');
        const std::string key = t.substr(0, sp);
        const std::string value = sp == std::string::npos ? "" : t.substr(sp + 1);
        bool known = false;
        for (const char* k : kKeys) known = known || key == k;
        known = known || (key.size() == 6 && key.rfind("pin.", 0) == 0);  // pin.<port><bit>, checked by sim_set
        if (!known) { error(l.number, "unknown key: " + key); return; }
        if (run_ && !sim_set(key, value)) error(l.number, "simulator rejected " + key + " " + value);
    }

    void time(const Line& l) {
        const std::string t = l.text.substr(2);
        if (t == "boot") { if (run_) sim_boot(); return; }
        if (t == "boot quiet") {
            if (run_) { sim_boot(); sim_advance_ms(500); sim_take_tx(); }
            return;
        }
        if (t == "idle") {
            if (!run_) return;
            for (int ms = 0; !sim_idle(); ++ms) {
                if (ms == 60000) { error(l.number, "not idle after 60 s"); return; }
                sim_advance_ms(1);
            }
            return;
        }
        if (starts_with(t, "ms ")) {
            char* end = nullptr;
            const unsigned long ms = std::strtoul(t.c_str() + 3, &end, 10);
            if (*end != '\0' || ms == 0) { error(l.number, "bad ~ ms"); return; }
            if (run_) sim_advance_ms(static_cast<uint32_t>(ms));
            return;
        }
        error(l.number, "unknown ~ directive");
    }

    void send(const Line& l, bool raw) {
        std::string bytes;
        if (raw) {
            if (!unescape(l.text.substr(5), bytes)) { error(l.number, "bad escape"); return; }
        } else {
            bytes = l.text.substr(2) + "\n";
        }
        if (run_) { sim_rx(bytes); sim_settle(); }
    }

    void step(const Line& l) {
        const std::string& t = l.text;
        if (t.empty()) { tags_ = 0; return; }
        if (starts_with(t, "#:")) { tag(l); return; }
        if (t[0] == '#') return;
        if (starts_with(t, "<re ")) { expect(l, true); return; }
        if (starts_with(t, "< ") || t == "<") { expect(l, false); return; }
        if (starts_with(t, "= ")) { start_group(l.number); set(l); return; }
        if (starts_with(t, "~ ")) { start_group(l.number); time(l); return; }
        if (starts_with(t, ">raw ")) { start_group(l.number); send(l, true); return; }
        if (starts_with(t, "> ")) { start_group(l.number); send(l, false); return; }
        error(l.number, "unknown directive");
    }
};

bool load(const char* file, std::vector<Line>& lines) {
    std::ifstream in(file);
    if (!in) return false;
    std::string s;
    for (int n = 1; std::getline(in, s); ++n) {
        if (!s.empty() && s.back() == '\r') s.pop_back();
        lines.push_back({n, s});
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    bool lint_only = false;
    int first = 1;
    if (argc > 1 && std::string(argv[1]) == "--lint") { lint_only = true; first = 2; }
    if (first >= argc) {
        std::printf("usage: golden [--lint] <trace files...>\n");
        return 2;
    }

    const bool run = sim_available() && !lint_only;
    int failed = 0, pending = 0, passed = 0;
    Result total;
    for (int i = first; i < argc; ++i) {
        std::vector<Line> lines;
        if (!load(argv[i], lines)) { std::printf("%s: cannot read\n", argv[i]); ++failed; continue; }
        // Lint first (no simulator), then run.
        const Result lint = Runner(argv[i], false).go(lines);
        Result res = lint;
        if (lint.errors == 0 && run) res = Runner(argv[i], true).go(lines);
        const char* status = res.errors ? "FAIL" : run ? "PASS" : "PENDING";
        std::printf("%-8s %s  (faithful %d, measured %d, change %d, new %d, pending %d)\n", status, argv[i],
                    lint.tags_faithful, lint.tags_measured, lint.tags_change, lint.tags_new, lint.tags_pending);
        if (res.errors) ++failed;
        else if (run) ++passed;
        else ++pending;
        total.tags_pending += lint.tags_pending;
    }
    std::printf("%d passed, %d failed, %d pending (linted only); %d expectations still wait for a decision\n", passed,
                failed, pending, total.tags_pending);
    return failed ? 1 : 0;
}
