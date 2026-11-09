# STM8 SWIM

Debug access to STM8 microcontrollers over SWIM (ST UM0470), driven from the
control channel (`CommandSWIM`) and exposed in the Python library as
`swim_enter()`, `swim_read()`, `swim_write()`, `swim_reset()` and `swim_exit()`.

## Wiring

| Signal | Faultier pin                 | RP2040 | STM8 |
|--------|------------------------------|--------|------|
| SWIM   | `IO8`, 20-pin header         | GP13   | SWIM (PD1 on most parts) |
| NRST   | `RST`, 5-pin SWD header      | GP17   | NRST |
| GND    | any GND                      |        | VSS  |

- A **pull-up of about 1 kOhm** (680 Ohm to 2.2 kOhm) from SWIM to the target's
  VDD is required. A '0' from the target is high for only 250 ns before the
  next bit, and the RP2040's internal pull-up is far too weak for that once a
  wire and a target are attached. `swim_enter()` measures the rise time and
  fails with `"... ns rise, add ~1k pull-up"` if it is too slow. (A bare,
  unconnected pin rises fast enough on the internal pull-up, so the check only
  bites once something is wired.)
- **3.3 V only.** RP2040 pins are not 5 V tolerant. Run the STM8 at 3.3 V, or
  put open-drain level shifters (e.g. BSS138 type) on SWIM and NRST and connect
  the pull-up on the target side.
- NRST is optional for `hot_attach=True`, but without it entry fails on
  firmware that turns the SWIM pin into a GPIO.

## Behaviour

`swim_enter()` (default, connect under reset):

1. Holds NRST low.
2. Sends the entry sequence: 16 us low, four 1 kHz pulses, four 2 kHz pulses.
3. Times the target's synchronisation frame (128 SWIM clocks, nominally 16 us)
   and derives the bit clock from it.
4. Writes `SWIM_CSR = 0xA0` (SAFE_MASK | SWIM_DM: whole memory range and SRST
   usable, internal reset sources masked).
5. Releases NRST, waits 2 ms for option byte loading.
6. Sends a communication reset to re-time the now trimmed HSI (UM0470 3.2,
   step 9), then reads `SWIM_CSR` back and checks SWIM_DM.

The core is left stalled. With `hot_attach=True` steps 1 and 5 are skipped and
the firmware keeps running.

- `swim_read()` / `swim_write()` are ROTF / WOTF: raw memory reads and writes.
  Up to 1024 bytes are read and 128 bytes written per control-channel command
  (the Python library splits longer transfers). One write is one WOTF, which
  fits a whole STM8 flash block (64 or 128 bytes). Flash/EEPROM programming
  needs the target's own unlock and programming sequence written through it;
  that is not done here.
- `swim_reset()` sends SRST. The session survives; the Faultier waits for option
  byte loading and re-synchronises.
- `swim_exit()` sets `SWIM_CSR.RST` and sends SRST. The target resets with SWIM
  switched off and runs its firmware. The Faultier releases SWIM and gives NRST
  back to the SWD probe even if the target does not answer.

A NACKed frame is sent again for up to 50 ms (busy target); a corrupt byte from
the target is NACKed so it is sent again (up to 8 times). After any failed
transfer a communication reset is sent, so the next command starts clean.

## Timeouts

Nothing waits for the target without a bound. The PIO program itself blocks on
the line, so the C side times out on the PIO FIFOs and restarts the state
machine:

| Wait | Bound |
|------|-------|
| line high before driving it | 100 us |
| sync frame start / end | 2 ms / 100 us |
| one frame and the target's answer | 5 ms |
| a target NACKing the same frame | 50 ms |
| whole command | 1.5 s |
| host waiting for any SWIM reply | 5 s (Python) |
| gap inside a control-channel frame | 500 ms (`readin()`) |

## Resources and other features

- **PIO1 / SM1**, 16 instructions, loaded on first use. PIO0 belongs to the
  glitcher (its instruction memory is cleared before every glitch) and PIO1 /
  SM0 to the SWD probe; neither is touched.
- **SysTick** runs free as a cycle counter for timing the sync frame. Nothing
  else in the firmware uses it.
- **GP17 / NRST** is shared with the SWD probe, which drives it push-pull high.
  While a SWIM session is active it is open drain instead (an STM8 pulls NRST
  low itself on internal resets, SRST included), and the SWD probe's reset
  request has no effect. GP13 and GP17 are put back exactly as they were found
  on `swim_exit()` or a failed `swim_enter()`.
- **Glitching while SWIM is active**: allowed. The glitcher only uses PIO0 and
  its own pins, and every command is handled in turn on the main loop, so the
  two never run at the same time and the SWIM line stays released during a
  glitch. If the glitch or power cycle resets or browns out the target, SWIM is
  gone on the target side: the next SWIM command fails with an error and
  `swim_enter()` has to be called again.
- The UART bridge and SWD probe are not serviced while a SWIM command runs
  (as during a glitch); a SWIM command takes milliseconds.

## Protocol details

- Low-speed bit format only: 22 SWIM clocks per bit, a '1' 2 clocks low, a '0'
  20 clocks low. High speed (`SWIM_CSR.HS`) is not used.
- The PIO runs two cycles per SWIM clock and samples received bits about 11
  clocks after their falling edge. It listens for an answer from clock 20.5 of
  the host's last bit, and starts sending at least 16 clocks after it last
  received anything, so turnarounds work in both directions.
- See `swim.pio` for the command word layout.

## Testing

`test/swim_pio_sim.py` runs the assembled program (the pioasm-generated
`swim.pio.h`) on a cycle-level PIO emulator against a model UM0470 target:
bit shapes, turnaround timing, target clock error, busy NACKs, corrupted
parity, CPU latency and an absent target.

```
python3 swim/test/swim_pio_sim.py build/swim.pio.h
```

The Python library has unit tests (`tests/test_swim.py`) and a hardware smoke
test (`tests/hardware/swim_hw_check.py`).
