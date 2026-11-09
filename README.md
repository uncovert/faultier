# Faultier: Uncovert fork

This is [Uncovert](https://uncovert.eu)'s fork of Hextree's
[Faultier firmware](https://github.com/hextreeio/faultier). It is upstream
Faultier plus what we needed for our own work:

- **STM8 SWIM debug access.** Put an STM8 into debug mode, read and write its
  memory, reset it and let it run again, from Python (`swim_enter()`,
  `swim_read()`, `swim_write()`, `swim_reset()`, `swim_exit()`). Wiring,
  behaviour and limits: [swim/README.md](swim/README.md).
- **Firmware updates without the BOOTSEL button.** `faultier bootloader` puts
  the board into the USB bootloader; then copy the new `faultier.uf2` onto the
  `RPI-RP2` drive.
- **Fix:** control-channel commands longer than one 64-byte USB packet no
  longer hang the firmware.

Everything else (glitcher, ADC, SWD probe, UART bridge) is unchanged and the
protocol version is still 2, so the released `faultier` Python library keeps
working with this firmware. The SWIM and bootloader calls are in our
[faultier-python fork](https://github.com/uncovert/faultier-python).

| SWIM signal | Faultier pin | RP2040 |
|-------------|--------------|--------|
| SWIM | `IO8` (20-pin header), plus a ~1 kOhm pull-up to the target's 3.3 V | GP13 |
| NRST | `RST` (5-pin SWD header) | GP17 |

Tested on a Faultier v4 with an STM8L151G at 3.3 V; see
[swim/README.md](swim/README.md#tested) for what was and was not tested.
Faultier pins are 3.3 V only: use level shifters for 5 V targets.

## Branches

- `main`: this fork, i.e. upstream plus the changes above.
- `stm8-swim`: the same changes without this notice, kept clean so they can be
  offered upstream.

## Building

```
git submodule update --init
export PICO_SDK_PATH=/path/to/pico-sdk     # tested with pico-sdk 2.1.1
cmake -S . -B build -DPICO_BOARD=pico
make -C build -j                           # -> build/faultier.uf2
```

---

# Faultier

A fault-injection framework for the Raspberry Pi Pico in combination with the Faultier hardware.
