// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Runs the real firmware image (build/avr/swiftpro-pp.elf) on a simulated ATmega2560 (simavr) and checks a scripted
// session against it. Host-side only; nothing here touches the arm.
//
// Simulated around the MCU:
//   USART0      bytes fed at 100 us per byte (just under 115200 baud); output collected line by line
//   AS5600 x3   bit-level I2C slaves on the real encoder pins (L PC1/PC3, R PH2/PH7, B PD5/PD6), open-drain SDA with
//               a pull-up; registers STATUS 0x0B, ANGLE 0x0E/0x0F, AGC 0x1A, MAGNITUDE 0x1B/0x1C. Each angle follows
//               its motor (4096 counts per 28800 steps) unless stalled or slipped
//   motors      STEP rising edges counted on PF6 (L), PL3 (R), PF0 (B); DIR PF7, PL1, PF1 high = negative
//   straps      PJ6 high, PJ3-PJ5 low: board V3.3.1
//   ADC         A5 = 2.5 V (motor power present)
//   EEPROM      work mode 0, the mode 0 offsets, encoder references (as test/sim.cpp)
//   stack       SRAM from __heap_start to RAMEND painted before reset; the lowest byte touched is reported
//
// Script (tools/simavr/session.txt), one directive per line; a subset of the golden-trace syntax:
//   # text                    comment
//   ~ ms <n>                  run n ms of simulated time
//   > <text>                  send text and "\n" (fed at line rate; run time with ~ ms)
//   < <text>                  the next output line equals text
//   <re <regex>               the next output line matches regex (whole line)
//   <skip <regex>             drop output lines matching regex until one doesn't (for periodic @3)
//   = stall <L|R|B|none>      that encoder stops following its motor
//   = slip <L|R|B> <counts>   that encoder jumps by counts, then follows again
//   = reset                   reset the AVR (as opening the USB port does); the AS5600s keep their bus state
//   = pin <P><b> <0|1>        drive an input pin from outside, e.g. "= pin K5 0" presses the effector origin switch
//   ? steps <L|R|B|T> <lo> <hi> the motor's net step count since boot is within lo..hi (T: the effector stepper)
//   ? port <P><b> <0|1>       the PORT (output) bit
//   ? en <L|R|B> <0|1>        the driver enable pin level
//   ? stack <max bytes>       the deepest stack so far is at most max bytes
// Before every '>', '=' and '?' and at the end, all output so far must have been matched.
//
//   harness <firmware.elf> <__heap_start, hex> <script>

#include <sim_avr.h>
#include <sim_elf.h>
#include <avr_ioport.h>
#include <avr_uart.h>
#include <avr_adc.h>
#include <avr_eeprom.h>
#include <sim_cycle_timers.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <regex>
#include <string>
#include <vector>

namespace {

avr_t* avr = nullptr;
constexpr uint32_t kHz = 16000000;

// ------------------------------------------------------------------------------------------------ motors

struct Motor {
    char port;
    uint8_t step, dir;
    char en_port;
    uint8_t en;
    int32_t steps = 0;
    bool last = false;
};
// L, R, B, and T: the stepper end effector (STEP PH6, DIR PH5, no enable pin; its "enable" entry is unused).
Motor motors[4] = {{'F', 6, 7, 'F', 2}, {'L', 3, 1, 'K', 0}, {'F', 0, 1, 'D', 7}, {'H', 6, 5, 'H', 6}};

uint8_t reg(char port, bool ddr) {
    avr_ioport_state_t st;
    avr_ioctl(avr, AVR_IOCTL_IOPORT_GETSTATE(port), &st);
    return ddr ? st.ddr : st.port;
}

void on_step(avr_irq_t*, uint32_t value, void* param) {
    Motor& m = *static_cast<Motor*>(param);
    const bool level = value != 0;
    if (level && !m.last) m.steps += (reg(m.port, false) >> m.dir) & 1 ? -1 : 1;
    m.last = level;
}

// ------------------------------------------------------------------------------------------------ AS5600

struct Chip {
    // bus
    char port;
    uint8_t sda, scl;
    uint8_t port_val = 0, ddr_val = 0;
    bool scl_prev = true, sda_prev = true, rise_seen = false;
    bool slave_low = false;
    enum class S { kIdle, kAddr, kAck, kReg, kRead, kMasterAck } s = S::kIdle;
    bool reading = false;
    uint8_t bits = 0, byte = 0, pointer = 0;
    // model
    uint16_t raw_base = 0;
    int32_t steps_base = 0;
    bool stalled = false;
    uint8_t status = 0x20;
    int motor = 0;
};
Chip chips[3] = {{'C', 1, 3}, {'H', 2, 7}, {'D', 5, 6}};
// PIN register data addresses (ATmega2560 datasheet register summary; avr-libc iom2560.h: PINC 0x06+0x20, PIND
// 0x09+0x20, PINH 0x100). An input pin reads from here.
uint16_t pin_addr(char port) { return port == 'C' ? 0x26 : port == 'D' ? 0x29 : 0x100; }

void drive_line(const Chip& c) {
    uint8_t& pin = avr->data[pin_addr(c.port)];
    pin = c.sda_prev ? static_cast<uint8_t>(pin | (1u << c.sda)) : static_cast<uint8_t>(pin & ~(1u << c.sda));
}

uint16_t angle(const Chip& c) {
    if (c.stalled) return c.raw_base;
    const long n = std::lround(static_cast<double>(motors[c.motor].steps - c.steps_base) * 4096.0 / 28800.0);
    return static_cast<uint16_t>(((c.raw_base + n) % 4096 + 4096) % 4096);
}

void rebase(Chip& c, uint16_t raw) {
    c.raw_base = raw;
    c.steps_base = motors[c.motor].steps;
}

uint8_t chip_reg(const Chip& c, uint8_t r) {
    switch (r) {
        case 0x0B: return c.status;
        case 0x0E: return static_cast<uint8_t>(angle(c) >> 8);
        case 0x0F: return static_cast<uint8_t>(angle(c) & 0xFF);
        case 0x1A: return 128;
        case 0x1B: return 0x07;
        case 0x1C: return 0xD0;
        default: return 0;
    }
}

void drive_bit(Chip& c) { c.slave_low = !((c.byte >> (7 - c.bits)) & 1); }

void bus_update(Chip& c) {
    // SCL has a pull-up on the board too: high unless driven low (a released pin, e.g. during reset, reads high).
    const bool scl = !(((c.ddr_val >> c.scl) & 1) && !((c.port_val >> c.scl) & 1));
    const bool master_low = (c.ddr_val >> c.sda) & 1;  // SDA's PORT bit is 0: DDR set = pulled low
    const bool sda = !(master_low || c.slave_low);
    if (scl && c.scl_prev && sda != c.sda_prev) {
        if (!sda) {  // START or repeated START
            if (std::getenv("HARNESS_I2C"))
                std::printf("  i2c %c START at %.3f ms\n", c.port, static_cast<double>(avr->cycle) * 1000.0 / kHz);
            c.s = Chip::S::kAddr;
            c.bits = 0;
            c.byte = 0;
        } else {     // STOP
            c.s = Chip::S::kIdle;
        }
        c.slave_low = false;
    } else if (scl && !c.scl_prev) {  // rising: sample
        c.rise_seen = true;
        if (c.s == Chip::S::kAddr || c.s == Chip::S::kReg) {
            c.byte = static_cast<uint8_t>(c.byte << 1 | (sda ? 1 : 0));
            ++c.bits;
        } else if (c.s == Chip::S::kMasterAck) {
            c.reading = !sda;  // reuse: true = master ACKed, more bytes wanted
        }
    } else if (!scl && c.scl_prev && c.rise_seen) {  // falling after a clock
        c.rise_seen = false;
        switch (c.s) {
            case Chip::S::kAddr:
                if (c.bits < 8) break;
                if ((c.byte >> 1) == 0x36) {
                    c.reading = c.byte & 1;
                    c.slave_low = true;  // ACK
                    c.s = Chip::S::kAck;
                } else {
                    c.s = Chip::S::kIdle;
                }
                break;
            case Chip::S::kReg:
                if (c.bits < 8) break;
                c.pointer = c.byte;
                c.slave_low = true;
                c.reading = false;
                c.s = Chip::S::kAck;
                break;
            case Chip::S::kAck:  // end of the ACK clock
                c.slave_low = false;
                c.bits = 0;
                if (c.reading) {
                    c.byte = chip_reg(c, c.pointer++);
                    drive_bit(c);
                    c.s = Chip::S::kRead;
                } else {
                    c.byte = 0;
                    c.s = Chip::S::kReg;
                }
                break;
            case Chip::S::kRead:
                if (++c.bits < 8) {
                    drive_bit(c);
                } else {
                    c.slave_low = false;
                    c.s = Chip::S::kMasterAck;
                }
                break;
            case Chip::S::kMasterAck:
                if (c.reading) {
                    c.bits = 0;
                    c.byte = chip_reg(c, c.pointer++);
                    drive_bit(c);
                    c.s = Chip::S::kRead;
                } else {
                    c.s = Chip::S::kIdle;
                }
                break;
            case Chip::S::kIdle: break;
        }
    }
    c.scl_prev = scl;
    const bool line = !(master_low || c.slave_low);
    c.sda_prev = line;
    drive_line(c);
}

void on_port(avr_irq_t*, uint32_t value, void* param) {
    Chip& c = *static_cast<Chip*>(param);
    c.port_val = static_cast<uint8_t>(value);
    bus_update(c);
}

// simavr signals a DDR write before it stores the new DDR, and may rewrite PIN from the old direction. So the bus is
// evaluated now (edges, START/STOP) and the SDA level written again one cycle later.
avr_cycle_count_t redrive(avr_t*, avr_cycle_count_t, void* param) {
    drive_line(*static_cast<Chip*>(param));
    return 0;
}

void on_ddr(avr_irq_t*, uint32_t value, void* param) {
    Chip& c = *static_cast<Chip*>(param);
    c.ddr_val = static_cast<uint8_t>(value);
    bus_update(c);
    avr_cycle_timer_register(avr, 1, redrive, &c);
}

// ------------------------------------------------------------------------------------------------ UART

std::deque<uint8_t> rx_pending;
std::string tx_line;
std::deque<std::string> out_lines;

void on_tx(avr_irq_t*, uint32_t value, void*) {
    const char ch = static_cast<char>(value);
    if (ch == '\n') {
        if (std::getenv("HARNESS_DEBUG"))
            std::printf("  [%8.3f ms] %s\n", static_cast<double>(avr->cycle) * 1000.0 / kHz, tx_line.c_str());
        out_lines.push_back(tx_line);
        tx_line.clear();
    } else if (ch != '\r') {
        tx_line += ch;
    } else {
        tx_line += "\\r";
    }
}

avr_cycle_count_t feed(avr_t* a, avr_cycle_count_t when, void*) {
    if (!rx_pending.empty()) {
        avr_raise_irq(avr_io_getirq(a, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT), rx_pending.front());
        rx_pending.pop_front();
    }
    return when + kHz / 10000;  // every 100 us
}

// ------------------------------------------------------------------------------------------------ profile

// HARNESS_PROFILE=1: sample the PC every 10 us and print the samples per word address at the end (for avr-addr2line).
std::vector<uint32_t> pc_samples;

avr_cycle_count_t sample_pc(avr_t* a, avr_cycle_count_t when, void*) {
    pc_samples.push_back(a->pc);
    return when + kHz / 100000;
}

// ------------------------------------------------------------------------------------------------ run

uint16_t heap_start = 0;
constexpr uint16_t kRamEnd = 0x21FF;
constexpr uint8_t kPaint = 0xA5;

unsigned stack_used() {
    uint16_t a = heap_start;
    while (a <= kRamEnd && avr->data[a] == kPaint) ++a;
    return static_cast<unsigned>(kRamEnd - a + 1);
}

bool run_ms(uint32_t ms) {
    const avr_cycle_count_t end = avr->cycle + static_cast<avr_cycle_count_t>(ms) * (kHz / 1000);
    while (avr->cycle < end) {
        const int st = avr_run(avr);
        if (st == cpu_Done || st == cpu_Crashed) {
            std::printf("CPU stopped (state %d) at cycle %llu, pc 0x%05x\n", st,
                        static_cast<unsigned long long>(avr->cycle), avr->pc);
            return false;
        }
    }
    return true;
}

void write_ee(std::vector<uint8_t>& ee, uint16_t at, const void* p, size_t n) {
    std::memcpy(ee.data() + at, p, n);
}

// Straps: PJ6 high (V3.3.1), PJ3-PJ5 low; ADC5: 2.5 V. Hard-wired on the board, so set again after every reset.
void board_inputs() {
    uint8_t& pinj = avr->data[0x103];  // PINJ (avr-libc iom2560.h: _SFR_MEM8(0x103))
    pinj = static_cast<uint8_t>((pinj & ~0x78) | 0x40);
    for (uint8_t b = 3; b <= 5; ++b) avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('J'), b), 0);
    avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('J'), 6), 1);
    avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC5), 2500);
}

int motor_of(char letter) {
    return letter == 'L' ? 0 : letter == 'R' ? 1 : letter == 'B' ? 2 : letter == 'T' ? 3 : -1;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::fprintf(stderr, "usage: harness <firmware.elf> <__heap_start hex> <script>\n");
        return 2;
    }
    heap_start = static_cast<uint16_t>(std::strtoul(argv[2], nullptr, 16) & 0xFFFF);  // avr-nm adds 0x800000

    elf_firmware_t fw{};
    if (elf_read_firmware(argv[1], &fw) != 0) {
        std::fprintf(stderr, "can't read %s\n", argv[1]);
        return 2;
    }
    avr = avr_make_mcu_by_name("atmega2560");
    if (!avr) return 2;
    avr_init(avr);
    avr->frequency = kHz;
    avr->avcc = 5000;
    avr->aref = 5000;
    avr->log = LOG_WARNING;
    avr_load_firmware(avr, &fw);

    // EEPROM as on your arm (test/sim.cpp defaults plus the traces' encoder references).
    std::vector<uint8_t> ee(4096, 0xFF);
    ee[900] = 0;
    const float front = 56.65f, height = 74.55f;
    write_ee(ee, 920, &front, 4);
    write_ee(ee, 910, &height, 4);
    const uint16_t refs[3] = {1655, 2673, 1664};  // struct order b, l, r
    write_ee(ee, 820, refs, 6);
    ee[818] = 0xBB;
    std::memcpy(ee.data() + 1002, "000000000000", 12);
    avr_eeprom_desc_t d{ee.data(), 0, 4096};
    avr_ioctl(avr, AVR_IOCTL_EEPROM_SET, &d);

    // Stack paint.
    for (uint32_t a = heap_start; a <= kRamEnd; ++a) avr->data[a] = kPaint;

    // UART0: no console echo; collect output.
    uint32_t flags = 0;
    avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('0'), &flags);
    flags &= ~AVR_UART_FLAG_STDIO;
    avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &flags);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), on_tx, nullptr);
    avr_cycle_timer_register(avr, kHz / 10000, feed, nullptr);
    if (std::getenv("HARNESS_PROFILE")) avr_cycle_timer_register(avr, kHz / 100000, sample_pc, nullptr);

    board_inputs();

    // Motors and encoders. Initial raw counts as the traces: B1654 L2820 R1403.
    for (Motor& m : motors)
        avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(m.port), m.step), on_step, &m);
    const uint16_t raw0[3] = {2820, 1403, 1654};
    for (int i = 0; i < 3; ++i) {
        Chip& c = chips[i];
        c.motor = i;
        rebase(c, raw0[i]);
        avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(c.port), IOPORT_IRQ_REG_PORT), on_port, &c);
        avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(c.port), IOPORT_IRQ_DIRECTION_ALL), on_ddr,
                                &c);
        drive_line(c);  // pull-up: idle high
    }

    std::ifstream in(argv[3]);
    if (!in) {
        std::fprintf(stderr, "can't read %s\n", argv[3]);
        return 2;
    }
    int failures = 0, lineno = 0;
    auto fail = [&](const std::string& what) {
        std::printf("%s:%d: %s\n", argv[3], lineno, what.c_str());
        ++failures;
    };
    auto unmatched = [&]() {
        while (!out_lines.empty()) {
            fail("unexpected output: " + out_lines.front());
            out_lines.pop_front();
        }
    };
    std::string line;
    while (std::getline(in, line)) {
        ++lineno;
        if (line.empty() || line[0] == '#') continue;
        if (line.rfind("~ ms ", 0) == 0) {
            if (!run_ms(static_cast<uint32_t>(std::atol(line.c_str() + 5)))) return 1;
        } else if (line.rfind("> ", 0) == 0) {
            unmatched();
            for (char ch : line.substr(2)) rx_pending.push_back(static_cast<uint8_t>(ch));
            rx_pending.push_back('\n');
        } else if (line.rfind("<skip ", 0) == 0) {
            const std::regex re(line.substr(6));
            while (!out_lines.empty() && std::regex_match(out_lines.front(), re)) out_lines.pop_front();
        } else if (line.rfind("<re ", 0) == 0 || line.rfind("< ", 0) == 0) {
            const bool re = line[1] == 'r';
            const std::string want = line.substr(re ? 4 : 2);
            if (out_lines.empty()) {
                fail("expected a line, got no output (want " + want + ")");
                continue;
            }
            const std::string got = out_lines.front();
            out_lines.pop_front();
            if (re ? !std::regex_match(got, std::regex(want)) : got != want) fail("got  " + got + "\n    want " + want);
        } else if (line.rfind("= stall ", 0) == 0) {
            unmatched();
            const int m = motor_of(line[8]);
            for (int i = 0; i < 3; ++i) {
                const uint16_t now = angle(chips[i]);
                chips[i].stalled = i == m;
                rebase(chips[i], now);
            }
        } else if (line == "= reset") {
            unmatched();
            avr_reset(avr);
            board_inputs();
            for (Chip& c : chips) {  // all pins are inputs after a reset
                c.ddr_val = 0;
                c.port_val = 0;
                bus_update(c);
            }
            for (Motor& m : motors) m.last = false;
            // avr_reset drops the cycle timers and resets the UART: set the feeder and the flags up again.
            avr_cycle_timer_register(avr, kHz / 10000, feed, nullptr);
            uint32_t uf = 0;
            avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('0'), &uf);
            uf &= ~AVR_UART_FLAG_STDIO;
            avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &uf);
            const int busy = (chips[0].slave_low ? 1 : 0) + (chips[1].slave_low ? 1 : 0) + (chips[2].slave_low ? 1 : 0);
            if (std::getenv("HARNESS_DEBUG")) {
                std::printf("  reset: %d chip(s) holding SDA low;", busy);
                for (const Chip& c : chips)
                    std::printf(" %c state %d bits %d low %d scl %d sda %d", c.port, static_cast<int>(c.s), c.bits,
                                c.slave_low, c.scl_prev, c.sda_prev);
                std::printf("\n");
            }
        } else if (line.rfind("= slip ", 0) == 0) {
            unmatched();
            const int m = motor_of(line[7]);
            const long n = std::atol(line.c_str() + 9);
            rebase(chips[m], static_cast<uint16_t>(((angle(chips[m]) + n) % 4096 + 4096) % 4096));
        } else if (line.rfind("= pin ", 0) == 0) {
            unmatched();
            avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(line[6]), line[7] - '0'), line[9] == '1' ? 1 : 0);
        } else if (line.rfind("? port ", 0) == 0) {
            unmatched();
            const int level = (reg(line[7], false) >> (line[8] - '0')) & 1;
            if (level != line[10] - '0') fail("port bit is " + std::to_string(level));
        } else if (line.rfind("? steps ", 0) == 0) {
            unmatched();
            const int m = motor_of(line[8]);
            long lo = 0, hi = 0;
            std::sscanf(line.c_str() + 10, "%ld %ld", &lo, &hi);
            std::printf("  steps %c = %d\n", line[8], motors[m].steps);
            if (motors[m].steps < lo || motors[m].steps > hi) fail("steps out of range");
        } else if (line.rfind("? en ", 0) == 0) {
            unmatched();
            const Motor& m = motors[motor_of(line[5])];
            const int level = (reg(m.en_port, false) >> m.en) & 1;
            if (level != std::atoi(line.c_str() + 7)) fail("enable pin is " + std::to_string(level));
        } else if (line.rfind("? stack ", 0) == 0) {
            unmatched();
            const unsigned used = stack_used();
            std::printf("  stack high-water mark: %u bytes\n", used);
            if (used > static_cast<unsigned>(std::atoi(line.c_str() + 8))) fail("stack too deep");
        } else {
            fail("bad directive: " + line);
        }
    }
    unmatched();
    if (std::getenv("HARNESS_PROFILE")) {
        FILE* f = std::fopen("build/simavr/pc-samples.txt", "w");
        for (uint32_t pc : pc_samples) std::fprintf(f, "0x%x\n", pc);
        std::fclose(f);
    }
    if (std::getenv("HARNESS_DEBUG"))
        std::printf("pc 0x%05x (word 0x%05x), sp 0x%04x\n", avr->pc, avr->pc / 2,
                    avr->data[0x5D] | avr->data[0x5E] << 8);
    std::printf("%s: %s, %.1f s simulated, stack %u bytes\n", argv[3], failures ? "FAIL" : "PASS",
                static_cast<double>(avr->cycle) / kHz, stack_used());
    return failures ? 1 : 0;
}
