#!/usr/bin/env python3
"""
Host-side check of the SWIM PIO program against a model STM8 target.

Runs the assembled program from the pioasm-generated swim.pio.h on a small,
cycle-level PIO emulator, drives it with a Python port of the framing logic in
swim.c (pio_command / send_frame / rotf_data / wotf), and talks to a model SWIM
target that decodes bits the way UM0470 3.3.2 describes (<= 8 low samples is
'1', >= 9 is '0', every bit starts with a falling edge) and answers at the end
of its own 22-clock bit period. The model can run off-frequency, answer with no
extra delay, NACK frames as if busy, and corrupt the parity of bytes it sends;
the host side can be slowed down to mimic interrupt latency.

This checks bit timing, framing, ACK/NACK handling and FIFO pipelining. It does
not replace a test on a real STM8: entry sequence, sync timing and analogue
line behaviour are not modelled, and the model's answer timing is an
assumption (the earliest a sane target can answer).

Usage: swim_pio_sim.py <build-dir>/swim.pio.h
"""

import random
import re
import sys

CLOCK = 1000            # time units per SWIM clock
PIO_CYCLE = CLOCK // 2  # swim.pio runs two PIO cycles per SWIM clock


# --- PIO emulator (only what swim.pio uses) -------------------------------------

class PIO:
    def __init__(self, program, labels, wrap_target, wrap, time_per_cycle=PIO_CYCLE):
        self.program = program
        self.labels = labels
        self.wrap_target = wrap_target
        self.wrap = wrap
        self.time_per_cycle = time_per_cycle
        self.tx = []
        self.rx = []
        self.listen_log = []       # cycle numbers at which `wait 0 pin` first runs
        self.cycle = 0
        self.restart()

    def restart(self):
        self.pc = self.labels["cmd"]
        self.x = self.y = 0
        self.osr = self.isr = 0
        self.delay = 0
        self.pindir = 0            # 1 = driving the line low
        self.waiting = False
        self.tx.clear()
        self.rx.clear()

    def idle(self):
        return not self.tx and self.pc == self.labels["cmd"] and self.delay == 0

    def _out(self, count):
        value = self.osr >> (32 - count)
        self.osr = (self.osr << count) & 0xFFFFFFFF
        return value

    def step(self, line_high):
        """Execute one PIO cycle. `line_high` is the line as sampled now."""
        self.cycle += 1
        if self.delay:
            self.delay -= 1
            return
        inst = self.program[self.pc]
        opcode = inst >> 13
        delay = (inst >> 8) & 0x1F
        arg_hi = (inst >> 5) & 0x7
        arg_lo = inst & 0x1F
        next_pc = self.pc + 1 if self.pc != self.wrap else self.wrap_target

        if opcode == 0:                          # JMP
            if arg_hi == 0:
                take = True
            elif arg_hi == 2:                    # x--
                take = self.x != 0
                self.x = (self.x - 1) & 0xFFFFFFFF
            elif arg_hi == 4:                    # y--
                take = self.y != 0
                self.y = (self.y - 1) & 0xFFFFFFFF
            else:
                raise NotImplementedError(f"jmp condition {arg_hi}")
            if take:
                next_pc = arg_lo
        elif opcode == 1:                        # WAIT
            polarity = (inst >> 7) & 1
            assert (inst >> 5) & 3 == 1 and arg_lo == 0, "only 'wait <p> pin 0'"
            if polarity == 0 and not self.waiting:
                self.listen_log.append(self.cycle)
            if int(line_high) != polarity:
                self.waiting = True
                return                           # stalled, no delay yet
            self.waiting = False
        elif opcode == 2:                        # IN
            assert arg_hi == 0 and arg_lo == 1, "only 'in pins, 1'"
            self.isr = ((self.isr << 1) | int(line_high)) & 0xFFFFFFFF
        elif opcode == 3:                        # OUT, shifting left
            value = self._out(arg_lo or 32)
            if arg_hi == 1:
                self.x = value
            elif arg_hi == 2:
                self.y = value
            elif arg_hi == 4:
                self.pindir = value & 1
            elif arg_hi == 5:
                next_pc = value
            else:
                raise NotImplementedError(f"out destination {arg_hi}")
        elif opcode == 4:                        # PUSH / PULL (blocking)
            if (inst >> 7) & 1:
                if not self.tx:
                    return                       # stalled on an empty TX FIFO
                self.osr = self.tx.pop(0)
            else:
                if len(self.rx) >= 4:
                    return
                self.rx.append(self.isr)
                self.isr = 0
        elif opcode == 5:                        # MOV: only the 'mov y, y' nop
            assert inst & 0xFF == 0x42, "only 'nop'"
        elif opcode == 7:                        # SET
            assert arg_hi == 4, "only 'set pindirs'"
            self.pindir = arg_lo & 1
        else:
            raise NotImplementedError(f"opcode {opcode}")

        self.pc = next_pc
        self.delay = delay


# --- Model SWIM target (UM0470) ---------------------------------------------------

class Target:
    """Low-speed SWIM slave sampling the line once per target SWIM clock."""

    def __init__(self, time_per_clock, rng, nack_rate=0.0, corrupt_rate=0.0, answer_delay=0):
        self.time_per_clock = time_per_clock
        self.rng = rng
        self.nack_rate = nack_rate
        self.corrupt_rate = corrupt_rate
        self.answer_delay = answer_delay
        self.memory = {a: (a * 7 + 3) & 0xFF for a in range(0x10000)}
        self.drive_low = False
        self.low_samples = 0
        self.since_fall = 0
        self.prev_high = True
        self.bits = []
        self.out = []                            # queued levels to send (True = low)
        self.state = "cmd"
        self.expect_host_answer = False
        self.need_high = False
        self.log = []
        self.stats = {"nacks_sent": 0, "corrupted": 0, "decode_errors": 0, "host_too_early": 0}

    def send_bits(self, bits):
        # Answer at the end of the current 22-clock bit period, plus a delay.
        wave = [False] * (max(0, 22 - self.since_fall) + self.answer_delay)
        for b in bits:
            wave += [True] * (2 if b else 20) + [False] * (20 if b else 2)
        self.out += wave

    def send_data(self):
        value = self.memory[(self.address + self.index) & 0xFFFFFF]
        parity = bin(value).count("1") & 1
        if self.rng.random() < self.corrupt_rate:
            parity ^= 1
            self.stats["corrupted"] += 1
        self.send_bits([1] + [(value >> (7 - i)) & 1 for i in range(8)] + [parity])
        self.expect_host_answer = True

    def frame_length(self):
        if self.expect_host_answer:
            return 1
        return 5 if self.state == "cmd" else 10

    def on_bit(self, bit):
        self.bits.append(bit)
        if len(self.bits) < self.frame_length():
            return
        bits, self.bits = self.bits, []

        if self.expect_host_answer:
            self.expect_host_answer = False
            if bits[0]:
                self.index += 1
                if self.index < self.count:
                    self.send_data()
                else:
                    self.state = "cmd"
            else:
                self.send_data()                 # NACK: same byte again
            return

        header, payload, parity = bits[0], bits[1:-1], bits[-1]
        value = 0
        for b in payload:
            value = (value << 1) | b
        good = header == 0 and (sum(payload) & 1) == parity
        if not good:
            self.stats["decode_errors"] += 1
        if not good or self.rng.random() < self.nack_rate:
            self.stats["nacks_sent"] += 1
            self.send_bits([0])
            return

        self.send_bits([1])
        state = self.state
        if state == "cmd":
            self.command = value
            self.log.append(("cmd", value))
            self.state = "n" if value in (1, 2) else "cmd"
        elif state == "n":
            self.count, self.address, self.state = value, 0, "e"
        elif state in ("e", "h", "l"):
            self.address = (self.address << 8) | value
            self.state = {"e": "h", "h": "l", "l": "data"}[state]
            if self.state == "data":
                self.index = 0
                if self.command == 1:            # ROTF: start sending
                    self.state = "rotf"
                    self.send_data()
        elif state == "data":                    # WOTF payload
            self.memory[(self.address + self.index) & 0xFFFFFF] = value
            self.index += 1
            if self.index == self.count:
                self.state = "cmd"

    def sample(self, line_high):
        """Called once per target clock with the line as seen now.

        Edge-triggered: after sending, the receiver only re-arms once it has
        seen the line high, so a host bit that starts before the target's own
        bit period is over is lost (counted as host_too_early)."""
        self.since_fall += 1
        if self.out:
            self.drive_low = self.out.pop(0)
            if self.drive_low and self.prev_high:
                self.since_fall = 0
            self.prev_high = not self.drive_low
            if not self.out:
                self.need_high = True
            return
        self.drive_low = False
        if self.need_high:
            self.need_high = False
            if not line_high:
                self.stats["host_too_early"] += 1
                self.low_samples = -10**6        # swallow this bit
            self.prev_high = line_high
            return
        if not line_high:
            if self.prev_high:
                self.since_fall = 0
            self.low_samples += 1
        elif self.low_samples:
            if self.low_samples > 0:
                self.on_bit(1 if self.low_samples <= 8 else 0)
            self.low_samples = 0
        self.prev_high = line_high


# --- The wire, the PIO and the target ----------------------------------------------

class Sim:
    def __init__(self, pio, target, cpu_latency=0, rng=None):
        self.pio = pio
        self.target = target
        self.now = 0
        self.next_pio = 0
        self.next_target = PIO_CYCLE // 3        # not phase-locked to the PIO
        self.cpu_latency = cpu_latency
        self.rng = rng or random.Random(0)

    def line_high(self):
        return not (self.pio.pindir or self.target.drive_low)

    def advance(self, duration):
        end = self.now + duration
        while True:
            t = min(self.next_pio, self.next_target)
            if t >= end:
                self.now = end
                return
            self.now = t
            if t == self.next_pio:
                self.pio.step(self.line_high())
                self.next_pio += self.pio.time_per_cycle
            if t == self.next_target:
                self.target.sample(self.line_high())
                self.next_target += self.target.time_per_clock

    def cpu_delay(self):
        if self.cpu_latency:
            self.advance(self.rng.randint(0, self.cpu_latency))

    def put(self, word):
        self.cpu_delay()
        if len(self.pio.tx) >= 4:
            raise RuntimeError("TX FIFO overflow")
        self.pio.tx.append(word)

    def get(self, timeout):
        end = self.now + timeout
        while not self.pio.rx:
            if self.now >= end:
                return None
            self.advance(CLOCK)
        self.cpu_delay()
        return self.pio.rx.pop(0)

    def wait_idle(self, timeout):
        end = self.now + timeout
        while not self.pio.idle():
            if self.now >= end:
                return False
            self.advance(CLOCK)
        return True


# --- Port of the framing logic in swim.c ---------------------------------------------

class Host:
    def __init__(self, sim, labels):
        self.sim = sim
        self.labels = labels

    def pio_command(self, bits, tx_count, rx_count):
        entry = self.labels["tx_start"] if tx_count else self.labels["rx_bit"]
        word = ((tx_count - 1 if tx_count else 0) << 27) \
            | ((rx_count - 1 if rx_count else 0) << 22) | (entry << 17)
        if tx_count:
            nxt = self.labels["rx_bit"] if rx_count else self.labels["cmd"]
            inverted = ~bits & ((1 << tx_count) - 1)
            word |= inverted << (17 - tx_count)
            word |= nxt << (12 - tx_count)
        return word

    def host_frame(self, payload, bits):
        payload &= (1 << bits) - 1
        return self.pio_command((payload << 1) | (bin(payload).count("1") & 1), bits + 2, 1)

    def send_frame(self, payload, bits, data_follows=False, max_nacks=1000):
        for _ in range(max_nacks):
            self.sim.put(self.host_frame(payload, bits))
            if data_follows:
                self.sim.put(self.pio_command(0, 0, 10))
            ack = self.sim.get(5000 * CLOCK)
            if ack is None:
                raise SwimError("no ACK")
            if ack & 1:
                return
            if data_follows:
                self.sim.pio.restart()
        raise SwimError("NACK limit")

    def send_header(self, command, address, count, data_follows):
        self.send_frame(command, 3)
        self.send_frame(count, 8)
        self.send_frame(address >> 16, 8)
        self.send_frame(address >> 8, 8)
        self.send_frame(address, 8, data_follows)

    def rotf(self, address, count):
        self.send_header(1, address, count, True)
        data, bad = [], 0
        while len(data) < count:
            frame = self.sim.get(5000 * CLOCK)
            if frame is None:
                raise SwimError(f"no data at +{len(data)}")
            value = (frame >> 1) & 0xFF
            header_ok = bool(frame & (1 << 9))
            parity_ok = (frame & 1) == (bin(value).count("1") & 1)
            if header_ok and parity_ok:
                data.append(value)
                bad = 0
                answer = self.pio_command(1, 1, 10 if len(data) < count else 0)
            else:
                bad += 1
                if bad > 8:
                    raise SwimError("bad frame")
                answer = self.pio_command(0, 1, 10)
            self.sim.put(answer)
        if not self.sim.wait_idle(1000 * CLOCK):
            raise SwimError("PIO not idle")
        return bytes(data)

    def wotf(self, address, data):
        self.send_header(2, address, len(data), False)
        for b in data:
            self.send_frame(b, 8)


class SwimError(Exception):
    pass


# --- Checks -----------------------------------------------------------------------------

def bit_shapes(pio, host, bits, count):
    """Send `bits` and continue at rx_bit with an idle line. Returns, in SWIM
    clocks: low time and period of each bit, the guard between picking up the
    command and the first falling edge, and when the PIO starts listening for
    the answer counted from the last bit's falling edge."""
    pio.tx.append(host.pio_command(bits, count, 1))
    levels = []
    for _ in range(2 * 22 * (count + 3)):
        levels.append(pio.pindir)
        pio.step(not pio.pindir)
    falls = [i for i in range(1, len(levels)) if levels[i] and not levels[i - 1]]
    rises = [i for i in range(1, len(levels)) if not levels[i] and levels[i - 1]]
    lows = [(r - f) / 2 for f, r in zip(falls, rises)]
    periods = [(b - a) / 2 for a, b in zip(falls, falls[1:])]
    guard = falls[0] / 2           # the pull happens in cycle 0
    listen = (pio.listen_log[0] - 1 - falls[-1]) / 2 if pio.listen_log else None
    return lows, periods, guard, listen


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    header = open(sys.argv[1]).read()
    program = [int(h, 16) for h in re.findall(r"0x([0-9a-f]{4}), //", header)]
    labels = {name: int(v) for name, v in re.findall(r"#define swim_offset_(\w+) (\d+)u", header)}
    wrap_target = int(re.search(r"#define swim_wrap_target (\d+)", header).group(1))
    wrap = int(re.search(r"#define swim_wrap (\d+)", header).group(1))
    print(f"program: {len(program)} instructions, labels {labels}")

    failures = 0

    def report(ok, text):
        nonlocal failures
        failures += not ok
        print(f"[{'PASS' if ok else 'FAIL'}] {text}")

    # 1. Waveform: '1' = 2 clocks low, '0' = 20 low, 22-clock bits, a guard of
    #    at least 14 clocks before sending, and the PIO listening for the answer
    #    by clock 21 of the last bit (it ends at 22) whatever its value.
    for bits, count in ((0b1010011, 7), (0b0110100, 7), (0b1, 1), (0b0, 1)):
        pio = PIO(program, labels, wrap_target, wrap)
        lows, periods, guard, listen = bit_shapes(pio, Host(None, labels), bits, count)
        values = [(bits >> (count - 1 - i)) & 1 for i in range(count)]
        want = [2 if b else 20 for b in values]
        report(lows == want and all(p == 22 for p in periods) and guard >= 14
               and listen is not None and listen <= 21,
               f"bits {values}: low={lows} periods={periods} guard={guard} "
               f"listening at clock {listen} of the last bit")

    # 2. Transfers against the model target.
    scenarios = [
        ("nominal, answers at bit end", dict(ratio=1.00)),
        ("target 3% fast (calibrated)", dict(ratio=0.97)),
        ("target 3% slow (calibrated)", dict(ratio=1.03)),
        ("target 8% fast (uncalibrated)", dict(ratio=0.92)),
        ("target 20% slow", dict(ratio=1.20)),
        ("answers 3 clocks after bit end", dict(ratio=1.00, answer_delay=3)),
        ("busy target NACKs 20% of frames", dict(ratio=1.0, nack_rate=0.2)),
        ("target corrupts 10% of parity bits", dict(ratio=1.0, corrupt_rate=0.1)),
        ("up to 10 us CPU latency per FIFO access", dict(ratio=1.0, cpu_latency=80 * CLOCK)),
        ("all of the above", dict(ratio=0.98, nack_rate=0.1, corrupt_rate=0.05,
                                  cpu_latency=40 * CLOCK, answer_delay=1)),
    ]
    for name, cfg in scenarios:
        rng = random.Random(1234)
        pio = PIO(program, labels, wrap_target, wrap)
        target = Target(round(CLOCK * cfg["ratio"]), rng, cfg.get("nack_rate", 0.0),
                        cfg.get("corrupt_rate", 0.0), cfg.get("answer_delay", 0))
        sim = Sim(pio, target, cfg.get("cpu_latency", 0), rng)
        host = Host(sim, labels)
        try:
            expected = bytes(target.memory[a] for a in range(0x8000, 0x8000 + 300))
            got = host.rotf(0x8000, 255) + host.rotf(0x8000 + 255, 45)
            payload = bytes(rng.randrange(256) for _ in range(128))
            host.wotf(0x0100, payload)
            back = host.rotf(0x0100, 128)
            host.send_frame(0, 3)                # SRST
            ok = (got == expected and back == payload and target.log[-1] == ("cmd", 0)
                  and target.stats["host_too_early"] == 0)
            detail = (f"nacks={target.stats['nacks_sent']} corrupted={target.stats['corrupted']} "
                      f"target_decode_errors={target.stats['decode_errors']} "
                      f"host_too_early={target.stats['host_too_early']} "
                      f"t={sim.now / CLOCK / 8000:.1f} ms")
        except SwimError as e:
            ok, detail = False, f"SwimError: {e}"
        report(ok, f"{name}: {detail}")

    # 3. No target: every wait ends in an error instead of hanging.
    class Absent(Target):
        def sample(self, line_high):
            self.drive_low = False

    sim = Sim(PIO(program, labels, wrap_target, wrap), Absent(CLOCK, random.Random(0)))
    try:
        Host(sim, labels).send_frame(1, 3)
        report(False, "no target: got an ACK from nobody")
    except SwimError as e:
        report(str(e) == "no ACK", f"no target: SwimError: {e}")

    print("ALL PASS" if not failures else f"{failures} FAILED")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
