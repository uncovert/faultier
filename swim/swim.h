#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// STM8 SWIM debug access (ST UM0470).
//
// SWIM data runs on header pin IO8 (SWIM_IO), NRST on the SWD header's RST pin
// (SWIM_RST, shared with the SWD probe). Both are driven open drain, so the
// SWIM line needs an external pull-up of about 1 kOhm to the target's VDD, which
// must be 3.3 V: RP2040 pins are not 5 V tolerant.
//
// Everything here runs on PIO1 / SM1 and blocks the caller; every wait for the
// target is bounded, and a failed operation leaves the line released.

// Largest transfers accepted by swim_read() / swim_write().
#define SWIM_MAX_READ 1024
#define SWIM_MAX_WRITE 128

typedef enum {
    SWIM_OK = 0,
    SWIM_ERR_ARGUMENT,
    SWIM_ERR_NOT_ACTIVE,
    SWIM_ERR_PIO,
    SWIM_ERR_LINE_LOW,
    SWIM_ERR_SLOW_RISE,
    SWIM_ERR_NO_SYNC,
    SWIM_ERR_BAD_SYNC,
    SWIM_ERR_NO_ACK,
    SWIM_ERR_NACK,
    SWIM_ERR_RX_TIMEOUT,
    SWIM_ERR_FRAME,
    SWIM_ERR_NOT_DEBUG,
    SWIM_ERR_TIMEOUT,
} swim_status_t;

struct swim_enter_options {
    // Do not hold NRST low during entry; the running firmware keeps running.
    bool hot_attach;
    // First low pulse of the entry sequence, 0 = default (1 ms). Max 10 ms.
    uint32_t entry_low_us;
    // Longest accepted rise time of the released line, 0 = default (250 ns).
    uint32_t max_rise_ns;
};

struct swim_enter_result {
    uint32_t sync_ns;   // length of the target's synchronisation frame
    uint32_t rise_ns;   // rise time of the released line
};

// Put the target into SWIM active mode with debug access (SWIM_CSR = 0xA0).
// Unless hot_attach is set, NRST is held low during entry, so this works even if
// the application reuses the SWIM pin; the core is then left stalled.
swim_status_t swim_enter(const struct swim_enter_options *options, struct swim_enter_result *result);

// Reset the target so it leaves SWIM and runs its application, then hand the
// pins back. The pins are released even if the target does not answer.
swim_status_t swim_exit(void);

// SWIM SRST: system reset of the target. The SWIM session survives it.
swim_status_t swim_srst(void);

swim_status_t swim_read(uint32_t address, uint8_t *data, size_t length);
swim_status_t swim_write(uint32_t address, const uint8_t *data, size_t length);

bool swim_is_active(void);

// Human-readable description of the last failure, short enough for
// ResponseError.message (< 50 characters).
const char *swim_error_message(void);
