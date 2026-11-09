/*
 * STM8 SWIM debug access for Faultier.
 *
 * Protocol: ST UM0470 "STM8 SWIM communication protocol and debug module".
 * Only the low-speed bit format is used (22 SWIM clocks per bit, ~360 kbit/s).
 *
 * Bit timing lives in swim.pio; this file does entry, framing, ACK/NACK
 * handling, calibration and pin ownership.
 */

#include "swim/swim.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/structs/systick.h"

#include "faultier/faultier.h"
#include "swim.pio.h"

// PIO0 belongs to the glitcher (its instruction memory is cleared before every
// glitch) and PIO1 / SM0 to the SWD probe.
#define SWIM_PIO pio1
#define SWIM_SM 1

// SWIM commands (UM0470 table 1).
#define SWIM_CMD_SRST 0x0
#define SWIM_CMD_ROTF 0x1
#define SWIM_CMD_WOTF 0x2
#define SWIM_CMD_BITS 3

#define SWIM_CSR 0x7F80
#define SWIM_CSR_SAFE_MASK 0x80
#define SWIM_CSR_SWIM_DM 0x20
#define SWIM_CSR_RST 0x04

// ROTF / WOTF carry an 8-bit byte count.
#define SWIM_MAX_OTF_BYTES 255
// A data frame from the target: header '1', 8 data bits, parity.
#define SWIM_DATA_FRAME_BITS 10

// The synchronisation frame is 128 SWIM clocks low.
#define SWIM_SYNC_CLOCKS 128
#define SWIM_NOMINAL_CLOCK_HZ 8000000
// swim.pio runs two PIO cycles per SWIM clock.
#define SWIM_PIO_CYCLES_PER_CLOCK 2

// Entry sequence (UM0470 3.2): a first low pulse, four 1 kHz pulses, four 2 kHz
// pulses, ending with the line released. UM0470 says 16 us for the first pulse,
// but the entry detector runs off the LSI, which is ~38 kHz on the STM8L: on an
// STM8L151G after power-up, 16 us entered 12 times out of 20, 100 us and 1 ms
// 20 out of 20. 1 ms is also what ST-Link-compatible probes send.
#define SWIM_ENTRY_LOW_US 1000
// Upper bound for a caller-supplied first pulse, to keep entry bounded.
#define SWIM_ENTRY_LOW_MAX_US 10000
#define SWIM_ENTRY_SLOW_HALF_US 500
#define SWIM_ENTRY_FAST_HALF_US 250
#define SWIM_ENTRY_PULSES 4

// Communication reset (UM0470 3.6): the line held low for more than 64 SWIM
// clocks (8 us at 8 MHz). The target answers with a new synchronisation frame.
#define SWIM_COMM_RESET_US 32

// Option byte loading after NRST is released or after SRST. UM0470 asks for 1 ms.
#define SWIM_OBL_WAIT_US 2000

// Bounds on every wait for the target.
#define SWIM_IDLE_TIMEOUT_US 100        // line high before we start driving it
#define SWIM_RISE_TIMEOUT_US 50         // line high after we release it
#define SWIM_SYNC_TIMEOUT_US 2000       // start of the synchronisation frame
#define SWIM_SYNC_END_TIMEOUT_US 100    // end of the synchronisation frame
#define SWIM_FRAME_TIMEOUT_US 5000      // a frame plus the target's answer to it
#define SWIM_NACK_RETRY_US 50000        // a busy target may NACK a frame this long
#define SWIM_RX_MAX_RETRIES 8           // corrupt frames NACKed before giving up
#define SWIM_PIO_IDLE_TIMEOUT_US 1000   // PIO finishing its last transmit
// Whole command. Stays well below the host's USB read timeout so the host
// always gets an answer.
#define SWIM_OP_TIMEOUT_US 1500000

// Accepted synchronisation frame lengths: a 4 to 21 MHz SWIM clock. Nominal is
// 16 us (HSI/2 = 8 MHz), 8 us with CLK_SWIMCCR.SWIMCLK set, and the HSI is not
// trimmed yet while the target is held in reset.
#define SWIM_SYNC_MIN_NS 6000
#define SWIM_SYNC_MAX_NS 32000

// A '0' from the target is high for only 2 SWIM clocks (250 ns) before the next
// bit starts. A line slower than this to rise has no usable pull-up. (On real
// hardware without a pull-up, an STM8L151 line rose in 400-450 ns and no frame
// got through.)
#define SWIM_MAX_RISE_NS 250

struct pin_state {
    gpio_function_t function;
    bool output;
    bool level;
    bool pull_up;
    bool pull_down;
};

static struct {
    bool loaded;            // program in PIO1 and SM1 claimed (done once)
    uint offset;
    bool pins_taken;        // SWIM_IO / SWIM_RST state saved and taken over
    bool active;            // target in SWIM active mode with SWIM_DM set
    uint32_t sync_cycles;   // last synchronisation frame, in clk_sys cycles
    struct pin_state io_pin;
    struct pin_state rst_pin;
    absolute_time_t deadline;
} swim;

static char error_message[48];

static swim_status_t fail(swim_status_t status, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(error_message, sizeof(error_message), format, args);
    va_end(args);
    return status;
}

const char *swim_error_message(void)
{
    return error_message;
}

bool swim_is_active(void)
{
    return swim.active;
}

// --- Cycle counter -----------------------------------------------------------

// Nothing else in this firmware uses SysTick, so it runs free (no interrupt) as
// a 24-bit clk_sys cycle counter for timing pulses from the target.
static void cycle_counter_init(void)
{
    systick_hw->csr = 0;
    systick_hw->rvr = M0PLUS_SYST_RVR_RELOAD_BITS;
    systick_hw->cvr = 0;
    systick_hw->csr = M0PLUS_SYST_CSR_CLKSOURCE_BITS | M0PLUS_SYST_CSR_ENABLE_BITS;
}

static inline uint32_t cycle_count(void)
{
    return systick_hw->cvr;
}

// SysTick counts down.
static inline uint32_t cycles_between(uint32_t start, uint32_t end)
{
    return (start - end) & M0PLUS_SYST_RVR_RELOAD_BITS;
}

static uint32_t cycles_to_ns(uint32_t cycles)
{
    return (uint32_t)((uint64_t)cycles * 1000000000ull / clock_get_hz(clk_sys));
}

// --- Pins --------------------------------------------------------------------

static void pin_save(uint pin, struct pin_state *state)
{
    state->function = gpio_get_function(pin);
    state->output = gpio_get_dir(pin);
    state->level = gpio_get_out_level(pin);
    state->pull_up = gpio_is_pulled_up(pin);
    state->pull_down = gpio_is_pulled_down(pin);
}

static void pin_restore(uint pin, const struct pin_state *state)
{
    gpio_set_pulls(pin, state->pull_up, state->pull_down);
    gpio_put(pin, state->level);
    gpio_set_dir(pin, state->output);
    gpio_set_function(pin, state->function);
}

static inline bool line_high(void)
{
    return gpio_get(SWIM_IO);
}

// Only valid while the state machine is idle (stalled on its pull).
static inline void line_drive_low(void)
{
    pio_sm_exec(SWIM_PIO, SWIM_SM, pio_encode_set(pio_pindirs, 1));
}

static inline void line_release(void)
{
    pio_sm_exec(SWIM_PIO, SWIM_SM, pio_encode_set(pio_pindirs, 0));
}

static bool wait_line(bool level, uint32_t timeout_us)
{
    uint32_t start = time_us_32();
    while (line_high() != level) {
        if (time_us_32() - start > timeout_us) {
            return false;
        }
    }
    return true;
}

// NRST is shared with the SWD probe, which drives it push-pull high. The STM8
// pulls NRST low itself on internal resets (SRST included), so while SWIM owns
// the pin it is open drain: output latch 0, released = input with pull-up.
static inline void rst_assert(void)
{
    // The SWD probe may have written the latch since; force it back to 0 first.
    gpio_put(SWIM_RST, 0);
    gpio_set_dir(SWIM_RST, GPIO_OUT);
}

static inline void rst_release(void)
{
    gpio_set_dir(SWIM_RST, GPIO_IN);
}

static void take_pins(void)
{
    if (swim.pins_taken) {
        return;
    }
    pin_save(SWIM_IO, &swim.io_pin);
    pin_save(SWIM_RST, &swim.rst_pin);

    // Release before clearing the latch so a high NRST never glitches low.
    gpio_set_function(SWIM_RST, GPIO_FUNC_SIO);
    gpio_set_pulls(SWIM_RST, true, false);
    rst_release();
    gpio_put(SWIM_RST, 0);
    swim.pins_taken = true;
}

static void release_pins(void)
{
    if (swim.loaded) {
        pio_sm_set_enabled(SWIM_PIO, SWIM_SM, false);
        pio_sm_clear_fifos(SWIM_PIO, SWIM_SM);
        pio_sm_restart(SWIM_PIO, SWIM_SM);
        line_release();
    }
    if (swim.pins_taken) {
        pin_restore(SWIM_IO, &swim.io_pin);
        pin_restore(SWIM_RST, &swim.rst_pin);
        swim.pins_taken = false;
    }
    swim.active = false;
}

// --- PIO -----------------------------------------------------------------------

// Two PIO cycles per SWIM clock, derived from a synchronisation frame of
// `sync_cycles` clk_sys cycles (128 SWIM clocks, so 256 PIO cycles).
static void set_bit_clock(uint32_t sync_cycles)
{
    const uint32_t pio_cycles = SWIM_SYNC_CLOCKS * SWIM_PIO_CYCLES_PER_CLOCK;
    pio_sm_set_clkdiv_int_frac(SWIM_PIO, SWIM_SM,
                               sync_cycles / pio_cycles,
                               (sync_cycles % pio_cycles) * 256 / pio_cycles);
}

static inline uint32_t nominal_sync_cycles(void)
{
    return (uint32_t)((uint64_t)clock_get_hz(clk_sys) * SWIM_SYNC_CLOCKS / SWIM_NOMINAL_CLOCK_HZ);
}

static swim_status_t pio_setup(void)
{
    if (!swim.loaded) {
        if (pio_sm_is_claimed(SWIM_PIO, SWIM_SM) || !pio_can_add_program(SWIM_PIO, &swim_program)) {
            return fail(SWIM_ERR_PIO, "SWIM: PIO1 SM1 or program space taken");
        }
        pio_sm_claim(SWIM_PIO, SWIM_SM);
        swim.offset = pio_add_program(SWIM_PIO, &swim_program);
        cycle_counter_init();
        swim.loaded = true;
    }

    pio_sm_config c = swim_program_get_default_config(swim.offset);
    sm_config_set_set_pins(&c, SWIM_IO, 1);
    sm_config_set_out_pins(&c, SWIM_IO, 1);
    sm_config_set_in_pins(&c, SWIM_IO);
    // MSB first both ways, no autopull / autopush.
    sm_config_set_out_shift(&c, false, false, 32);
    sm_config_set_in_shift(&c, false, false, 32);
    pio_sm_init(SWIM_PIO, SWIM_SM, swim.offset + swim_offset_cmd, &c);
    set_bit_clock(nominal_sync_cycles());

    // Latch low and direction "in" before the PIO gets the pin: it never drives
    // high, and it only drives low when the program says so.
    pio_sm_set_pins_with_mask(SWIM_PIO, SWIM_SM, 0, 1u << SWIM_IO);
    pio_sm_set_pindirs_with_mask(SWIM_PIO, SWIM_SM, 0, 1u << SWIM_IO);
    gpio_set_pulls(SWIM_IO, true, false);
    pio_gpio_init(SWIM_PIO, SWIM_IO);
    pio_sm_set_enabled(SWIM_PIO, SWIM_SM, true);
    return SWIM_OK;
}

// Abandon whatever the state machine was doing and release the line.
static void sm_restart(void)
{
    pio_sm_set_enabled(SWIM_PIO, SWIM_SM, false);
    pio_sm_clear_fifos(SWIM_PIO, SWIM_SM);
    pio_sm_restart(SWIM_PIO, SWIM_SM);
    line_release();
    pio_sm_exec(SWIM_PIO, SWIM_SM, pio_encode_jmp(swim.offset + swim_offset_cmd));
    pio_sm_set_enabled(SWIM_PIO, SWIM_SM, true);
}

static bool fifo_put(uint32_t word)
{
    uint32_t start = time_us_32();
    while (pio_sm_is_tx_fifo_full(SWIM_PIO, SWIM_SM)) {
        if (time_us_32() - start > SWIM_FRAME_TIMEOUT_US) {
            return false;
        }
    }
    pio_sm_put(SWIM_PIO, SWIM_SM, word);
    return true;
}

static bool fifo_get(uint32_t *word, uint32_t timeout_us)
{
    uint32_t start = time_us_32();
    while (pio_sm_is_rx_fifo_empty(SWIM_PIO, SWIM_SM)) {
        if (time_us_32() - start > timeout_us) {
            return false;
        }
    }
    *word = pio_sm_get(SWIM_PIO, SWIM_SM);
    return true;
}

// The state machine is idle once its TX FIFO is empty and it is back on the
// pull at `cmd`, i.e. the last transmit has left the pin.
static bool wait_pio_idle(void)
{
    uint32_t start = time_us_32();
    while (!pio_sm_is_tx_fifo_empty(SWIM_PIO, SWIM_SM)
           || pio_sm_get_pc(SWIM_PIO, SWIM_SM) != swim.offset + swim_offset_cmd) {
        if (time_us_32() - start > SWIM_PIO_IDLE_TIMEOUT_US) {
            return false;
        }
    }
    return true;
}

// Command word for swim.pio (layout described there): send the low `tx_count`
// bits of `bits` MSB first, then receive `rx_count` bits. Either count may be 0,
// not both. At most 12 bits can be sent.
static inline uint32_t pio_command(uint32_t bits, uint tx_count, uint rx_count)
{
    uint entry = tx_count ? swim_offset_tx_start : swim_offset_rx_bit;
    uint32_t word = ((uint32_t)(tx_count ? tx_count - 1 : 0) << 27)
                  | ((uint32_t)(rx_count ? rx_count - 1 : 0) << 22)
                  | ((uint32_t)(swim.offset + entry) << 17);
    if (tx_count) {
        uint next = rx_count ? swim_offset_rx_bit : swim_offset_cmd;
        uint32_t inverted = ~bits & ((1u << tx_count) - 1);
        word |= inverted << (17 - tx_count);
        word |= (uint32_t)(swim.offset + next) << (12 - tx_count);
    }
    return word;
}

// Host frame followed by the receiver's ACK: header '0', payload MSB first,
// parity = XOR of the payload bits, then one bit back from the target.
static inline uint32_t host_frame(uint32_t payload, uint bits)
{
    payload &= (1u << bits) - 1;
    return pio_command((payload << 1) | (__builtin_parity(payload) & 1), bits + 2, 1);
}

// The host's answer to a data frame from the target (1 = ACK, 0 = NACK),
// optionally followed by receiving the next data frame.
static inline uint32_t host_answer(bool ack, bool receive_next)
{
    return pio_command(ack ? 1 : 0, 1, receive_next ? SWIM_DATA_FRAME_BITS : 0);
}

static inline bool op_expired(void)
{
    return time_reached(swim.deadline);
}

// --- Frames ----------------------------------------------------------------------

// Send one host frame and wait for the target's ACK. A NACK means the target is
// busy or saw a parity error; UM0470 has the host send the same frame again.
//
// With `data_follows` set (the last address byte of a ROTF), the receive for the
// target's first data frame is queued right behind the ACK, so that frame is
// caught however long the CPU takes to look at the ACK.
static swim_status_t send_frame(uint32_t payload, uint bits, bool data_follows)
{
    absolute_time_t give_up = make_timeout_time_us(SWIM_NACK_RETRY_US);

    while (true) {
        if (!fifo_put(host_frame(payload, bits))
            || (data_follows && !fifo_put(pio_command(0, 0, SWIM_DATA_FRAME_BITS)))) {
            sm_restart();
            return fail(SWIM_ERR_PIO, "SWIM: PIO did not take the frame");
        }

        uint32_t ack;
        if (!fifo_get(&ack, SWIM_FRAME_TIMEOUT_US)) {
            sm_restart();
            return fail(SWIM_ERR_NO_ACK, "SWIM: no ACK from target");
        }
        if (ack & 1) {
            return SWIM_OK;
        }

        // NACK. No data is coming, so drop the queued receive before resending.
        if (data_follows) {
            sm_restart();
        }
        if (time_reached(give_up) || op_expired()) {
            return fail(SWIM_ERR_NACK, "SWIM: target kept sending NACK");
        }
    }
}

// Data phase of a ROTF. The receive for the first frame was queued by
// send_frame(); every good frame is ACKed with the next receive queued behind
// it, a corrupt one is NACKed so the target sends the byte again.
static swim_status_t rotf_data(uint32_t address, uint8_t *data, size_t count)
{
    size_t i = 0;
    uint bad_frames = 0;

    while (i < count) {
        uint32_t frame;
        if (!fifo_get(&frame, SWIM_FRAME_TIMEOUT_US)) {
            sm_restart();
            return fail(SWIM_ERR_RX_TIMEOUT, "SWIM: no data from target at 0x%06lX",
                        (unsigned long)(address + i));
        }

        uint32_t value = (frame >> 1) & 0xFF;
        bool header_ok = frame & (1u << (SWIM_DATA_FRAME_BITS - 1));
        bool parity_ok = (frame & 1) == (uint32_t)(__builtin_parity(value) & 1);
        uint32_t answer;

        if (header_ok && parity_ok) {
            data[i++] = (uint8_t)value;
            bad_frames = 0;
            // ACK, and listen for the next byte straight away.
            answer = host_answer(true, i < count);
        } else {
            if (++bad_frames > SWIM_RX_MAX_RETRIES || op_expired()) {
                // The target is left waiting for an answer; the caller's comm
                // reset gets it out of that.
                return fail(SWIM_ERR_FRAME, "SWIM: bad %s from target at 0x%06lX",
                            header_ok ? "parity" : "header", (unsigned long)(address + i));
            }
            // NACK: the target sends the same byte again.
            answer = host_answer(false, true);
        }

        if (!fifo_put(answer)) {
            sm_restart();
            return fail(SWIM_ERR_PIO, "SWIM: PIO did not take the frame");
        }
    }

    if (!wait_pio_idle()) {
        sm_restart();
        return fail(SWIM_ERR_PIO, "SWIM: PIO did not finish the ACK");
    }
    return SWIM_OK;
}

static swim_status_t send_header(uint32_t command, uint32_t address, size_t count, bool data_follows)
{
    swim_status_t status;
    if ((status = send_frame(command, SWIM_CMD_BITS, false)) != SWIM_OK
        || (status = send_frame(count, 8, false)) != SWIM_OK
        || (status = send_frame(address >> 16, 8, false)) != SWIM_OK
        || (status = send_frame(address >> 8, 8, false)) != SWIM_OK) {
        return status;
    }
    return send_frame(address, 8, data_follows);
}

// Read on the fly, 1 to 255 bytes.
static swim_status_t rotf(uint32_t address, uint8_t *data, size_t count)
{
    swim_status_t status = send_header(SWIM_CMD_ROTF, address, count, true);
    if (status != SWIM_OK) {
        return status;
    }
    return rotf_data(address, data, count);
}

// Write on the fly, 1 to 255 bytes.
static swim_status_t wotf(uint32_t address, const uint8_t *data, size_t count)
{
    swim_status_t status = send_header(SWIM_CMD_WOTF, address, count, false);
    for (size_t i = 0; status == SWIM_OK && i < count; i++) {
        status = send_frame(data[i], 8, false);
    }
    return status;
}

// --- Synchronisation -------------------------------------------------------------

// Time the target's synchronisation frame, which follows the moment the host
// releases the line, and re-derive the bit clock from it. Interrupts must be
// off so the measurement is not stretched.
static swim_status_t measure_sync(void)
{
    if (!wait_line(true, SWIM_RISE_TIMEOUT_US)) {
        return fail(SWIM_ERR_LINE_LOW, "SWIM: line stays low after release");
    }
    if (!wait_line(false, SWIM_SYNC_TIMEOUT_US)) {
        return fail(SWIM_ERR_NO_SYNC, "SWIM: no sync frame from target");
    }
    uint32_t start = cycle_count();
    bool ended = wait_line(true, SWIM_SYNC_END_TIMEOUT_US);
    uint32_t cycles = cycles_between(start, cycle_count());
    if (!ended) {
        return fail(SWIM_ERR_BAD_SYNC, "SWIM: sync frame does not end");
    }

    uint32_t ns = cycles_to_ns(cycles);
    if (ns < SWIM_SYNC_MIN_NS || ns > SWIM_SYNC_MAX_NS) {
        return fail(SWIM_ERR_BAD_SYNC, "SWIM: sync frame of %lu ns", (unsigned long)ns);
    }
    swim.sync_cycles = cycles;
    set_bit_clock(cycles);

    // At least 300 ns of idle line before the first frame.
    busy_wait_us_32(1);
    return SWIM_OK;
}

static swim_status_t comm_reset(void)
{
    sm_restart();
    if (!wait_line(true, SWIM_IDLE_TIMEOUT_US)) {
        return fail(SWIM_ERR_LINE_LOW, "SWIM: line held low (no power, or pin in use)");
    }
    line_drive_low();
    busy_wait_us_32(SWIM_COMM_RESET_US);

    uint32_t irq = save_and_disable_interrupts();
    line_release();
    swim_status_t status = measure_sync();
    restore_interrupts(irq);
    return status;
}

// Release the line for the first time during entry and check how fast the
// pull-up brings it back: without a strong one nothing else will work, and a
// clear message beats a parity error later.
static swim_status_t release_checking_pullup(uint32_t max_rise_ns, uint32_t *rise_ns)
{
    uint32_t irq = save_and_disable_interrupts();
    line_release();
    uint32_t start = cycle_count();
    bool rose = wait_line(true, SWIM_RISE_TIMEOUT_US);
    *rise_ns = cycles_to_ns(cycles_between(start, cycle_count()));
    restore_interrupts(irq);

    if (!rose) {
        return fail(SWIM_ERR_LINE_LOW, "SWIM: line stays low after release");
    }
    if (*rise_ns > max_rise_ns) {
        return fail(SWIM_ERR_SLOW_RISE, "SWIM: %lu ns rise, add ~1k pull-up", (unsigned long)*rise_ns);
    }
    return SWIM_OK;
}

static swim_status_t entry_sequence(uint32_t entry_low_us, uint32_t max_rise_ns, uint32_t *rise_ns)
{
    if (!wait_line(true, SWIM_IDLE_TIMEOUT_US)) {
        return fail(SWIM_ERR_LINE_LOW, "SWIM: line held low (no power, or pin in use)");
    }

    line_drive_low();
    busy_wait_us_32(entry_low_us);

    for (uint i = 0; i < 2 * SWIM_ENTRY_PULSES; i++) {
        uint32_t half_us = (i < SWIM_ENTRY_PULSES) ? SWIM_ENTRY_SLOW_HALF_US : SWIM_ENTRY_FAST_HALF_US;
        if (i == 0) {
            swim_status_t status = release_checking_pullup(max_rise_ns, rise_ns);
            if (status != SWIM_OK) {
                return status;
            }
        } else {
            line_release();
        }
        busy_wait_us_32(half_us);
        line_drive_low();
        busy_wait_us_32(half_us);
    }

    uint32_t irq = save_and_disable_interrupts();
    line_release();
    swim_status_t status = measure_sync();
    restore_interrupts(irq);
    return status;
}

// --- Public API -------------------------------------------------------------------

static void start_operation(void)
{
    swim.deadline = make_timeout_time_us(SWIM_OP_TIMEOUT_US);
    error_message[0] = '\0';
}

// After a failed transfer the target may be stuck mid-command; a comm reset
// brings it back so the next command starts clean. Keeps the original error.
static swim_status_t finish_operation(swim_status_t status)
{
    if (status != SWIM_OK && swim.active) {
        char original[sizeof(error_message)];
        memcpy(original, error_message, sizeof(original));
        comm_reset();
        memcpy(error_message, original, sizeof(original));
    }
    return status;
}

static swim_status_t check_range(uint32_t address, size_t length, size_t max_length)
{
    if (length == 0 || length > max_length) {
        return fail(SWIM_ERR_ARGUMENT, "SWIM: length must be 1 to %u", (unsigned)max_length);
    }
    if (address > 0xFFFFFF || length > 0x1000000 - address) {
        return fail(SWIM_ERR_ARGUMENT, "SWIM: address range exceeds 24 bits");
    }
    return SWIM_OK;
}

static swim_status_t require_active(void)
{
    if (!swim.active) {
        return fail(SWIM_ERR_NOT_ACTIVE, "SWIM: not active, call swim_enter()");
    }
    return SWIM_OK;
}

swim_status_t swim_enter(const struct swim_enter_options *options, struct swim_enter_result *result)
{
    start_operation();

    bool hot_attach = options->hot_attach;
    uint32_t entry_low_us = options->entry_low_us ? options->entry_low_us : SWIM_ENTRY_LOW_US;
    uint32_t max_rise_ns = options->max_rise_ns ? options->max_rise_ns : SWIM_MAX_RISE_NS;
    uint32_t rise_ns = 0;
    if (entry_low_us > SWIM_ENTRY_LOW_MAX_US) {
        return fail(SWIM_ERR_ARGUMENT, "SWIM: entry_low_us must be <= %u", SWIM_ENTRY_LOW_MAX_US);
    }

    swim.active = false;

    // Save the pins before pio_setup() reconfigures SWIM_IO, so that leaving
    // SWIM puts them back the way they were found.
    take_pins();
    swim_status_t status = pio_setup();
    if (status != SWIM_OK) {
        release_pins();
        return status;
    }

    // Holding NRST low during entry gets in even if the application turns the
    // SWIM pin into a GPIO right after reset (UM0470 3.2).
    if (!hot_attach) {
        rst_assert();
    }

    status = entry_sequence(entry_low_us, max_rise_ns, &rise_ns);

    // Debug mode: whole memory range and SRST usable, internal resets masked.
    if (status == SWIM_OK) {
        uint8_t csr = SWIM_CSR_SAFE_MASK | SWIM_CSR_SWIM_DM;
        status = wotf(SWIM_CSR, &csr, 1);
    }

    if (!hot_attach) {
        // Starts option byte loading; the core stays stalled because SWIM_DM is set.
        rst_release();
        busy_wait_us_32(SWIM_OBL_WAIT_US);
    }

    // The HSI was untrimmed while in reset: re-calibrate against the trimmed one
    // (UM0470 3.2 step 9), then prove the link by reading SWIM_CSR back.
    if (status == SWIM_OK) {
        status = comm_reset();
    }
    if (status == SWIM_OK) {
        uint8_t csr = 0;
        status = rotf(SWIM_CSR, &csr, 1);
        if (status == SWIM_OK && !(csr & SWIM_CSR_SWIM_DM)) {
            status = fail(SWIM_ERR_NOT_DEBUG, "SWIM: SWIM_CSR reads 0x%02X, no SWIM_DM", csr);
        }
    }

    if (status != SWIM_OK) {
        release_pins();
        return status;
    }

    swim.active = true;
    if (result != NULL) {
        result->sync_ns = cycles_to_ns(swim.sync_cycles);
        result->rise_ns = rise_ns;
    }
    return SWIM_OK;
}

swim_status_t swim_exit(void)
{
    start_operation();
    swim_status_t status = SWIM_OK;

    if (swim.active) {
        // SWIM_CSR.RST makes the next SRST take the SWIM interface back to OFF,
        // so the target resets and runs its application without the debugger.
        uint8_t csr = 0;
        status = rotf(SWIM_CSR, &csr, 1);
        if (status == SWIM_OK) {
            csr |= SWIM_CSR_RST;
            status = wotf(SWIM_CSR, &csr, 1);
        }
        if (status == SWIM_OK) {
            status = send_frame(SWIM_CMD_SRST, SWIM_CMD_BITS, false);
        }
        if (status == SWIM_OK && !wait_pio_idle()) {
            status = fail(SWIM_ERR_PIO, "SWIM: PIO did not finish the SRST");
        }
    }

    release_pins();
    return status;
}

swim_status_t swim_srst(void)
{
    start_operation();
    swim_status_t status = require_active();
    if (status != SWIM_OK) {
        return status;
    }

    status = send_frame(SWIM_CMD_SRST, SWIM_CMD_BITS, false);
    if (status == SWIM_OK) {
        // Option bytes reload; then make sure the target is back on the line.
        busy_wait_us_32(SWIM_OBL_WAIT_US);
        status = comm_reset();
    }
    return finish_operation(status);
}

swim_status_t swim_read(uint32_t address, uint8_t *data, size_t length)
{
    start_operation();
    swim_status_t status = require_active();
    if (status == SWIM_OK) {
        status = check_range(address, length, SWIM_MAX_READ);
        if (status != SWIM_OK) {
            return status;
        }
    }

    for (size_t done = 0; status == SWIM_OK && done < length; ) {
        size_t chunk = length - done;
        if (chunk > SWIM_MAX_OTF_BYTES) {
            chunk = SWIM_MAX_OTF_BYTES;
        }
        status = rotf(address + done, data + done, chunk);
        done += chunk;
    }
    return finish_operation(status);
}

swim_status_t swim_write(uint32_t address, const uint8_t *data, size_t length)
{
    start_operation();
    swim_status_t status = require_active();
    if (status == SWIM_OK) {
        status = check_range(address, length, SWIM_MAX_WRITE);
        if (status != SWIM_OK) {
            return status;
        }
    }

    if (status == SWIM_OK) {
        status = wotf(address, data, length);
    }
    return finish_operation(status);
}
