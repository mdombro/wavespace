#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/pdm_microphone.h"
#include "tusb.h"

#define PDM_BLOCK_BITS   4096
#define PDM_BLOCK_BYTES  (PDM_BLOCK_BITS / 8)

static uint8_t sample_buffer[PDM_BLOCK_BYTES];
static uint8_t discard_buffer[PDM_BLOCK_BYTES];
static volatile bool block_pending = false;
static volatile int block_length = 0;

// Debug/telemetry state
static volatile uint32_t blocks_ready = 0;
static volatile uint32_t blocks_discarded = 0;
static volatile uint32_t overruns = 0;

static uint32_t blocks_streamed = 0;
static uint32_t bytes_sent = 0;
static uint32_t block_counter = 0;

static bool debug_stats_enabled = true;
static bool debug_dump_samples = false;
static bool stream_binary_data = false;

static uint32_t debug_interval_ms = 1000;
static uint32_t last_debug_ms = 0;
static bool mic_running = false;

static struct pdm_microphone_config config = {
    .gpio_data = 2,
    .gpio_clk = 3,
    .pio = pio0,
    .pio_sm = 0,
    .sample_rate = 2048000,
    .sample_buffer_size = PDM_BLOCK_BYTES,
};

static void clear_stats(void);
static void on_pdm_samples_ready(void);

static void reset_capture_state(void) {
    uint32_t status = save_and_disable_interrupts();
    block_pending = false;
    block_length = 0;
    restore_interrupts(status);

    clear_stats();
    last_debug_ms = to_ms_since_boot(get_absolute_time());
}

static bool restart_microphone(uint32_t new_rate) {
    uint32_t previous_rate = config.sample_rate;
    bool success = false;

    if (mic_running) {
        pdm_microphone_stop();
        mic_running = false;
    }

    pdm_microphone_deinit();

    config.sample_rate = new_rate;

    if (pdm_microphone_init(&config) < 0) {
        printf("Re-init failed at %lu Hz, restoring %lu Hz\n",
               (unsigned long)new_rate,
               (unsigned long)previous_rate);
        config.sample_rate = previous_rate;
        if (pdm_microphone_init(&config) < 0) {
            printf("FATAL: unable to restore microphone configuration\n");
            return false;
        }
    } else {
        success = true;
    }

    pdm_microphone_set_samples_ready_handler(on_pdm_samples_ready);

    if (pdm_microphone_start() < 0) {
        printf("Restart failed at %lu Hz\n", (unsigned long)config.sample_rate);
        success = false;
    } else {
        mic_running = true;
    }

    reset_capture_state();

    if (success) {
        printf("PDM clock running at %lu Hz\n", (unsigned long)config.sample_rate);
    }

    return success;
}

static void on_pdm_samples_ready(void) {
    uint8_t* target = block_pending ? discard_buffer : sample_buffer;
    int read = pdm_microphone_read(target, PDM_BLOCK_BYTES);

    if (!block_pending && read > 0) {
        blocks_ready++;
        block_length = read;
        block_pending = true;
    } else if (block_pending && read > 0) {
        blocks_discarded++;
    }
}

static void print_help(void) {
    printf("\nDebug commands:\n");
    printf("  h/? - show this help\n");
    printf("  b   - toggle binary streaming (currently %s)\n", stream_binary_data ? "ON" : "OFF");
    printf("  d   - toggle per-block sample dump (currently %s)\n", debug_dump_samples ? "ON" : "OFF");
    printf("  s   - toggle periodic stats (currently %s)\n", debug_stats_enabled ? "ON" : "OFF");
    printf("  i   - print current stats immediately\n");
    printf("  c   - clear counters\n\n");
    printf("  1   - set PDM clock to 512 kHz\n");
    printf("  2   - set PDM clock to 1.024 MHz\n");
    printf("  3   - set PDM clock to 2.048 MHz\n");
    printf("  r   - reinitialize microphone with current settings\n\n");
}

static void print_stats(void) {
    printf("Stats: ready=%lu streamed=%lu discarded=%lu overruns=%lu bytes=%lu stream=%s dump=%s pdm_clk=%luHz\n",
           (unsigned long)blocks_ready,
           (unsigned long)blocks_streamed,
           (unsigned long)blocks_discarded,
           (unsigned long)overruns,
           (unsigned long)bytes_sent,
           stream_binary_data ? "ON" : "OFF",
           debug_dump_samples ? "ON" : "OFF",
           (unsigned long)config.sample_rate);
}

static void clear_stats(void) {
    uint32_t status = save_and_disable_interrupts();
    blocks_ready = 0;
    blocks_discarded = 0;
    overruns = 0;
    blocks_streamed = 0;
    bytes_sent = 0;
    block_counter = 0;
    restore_interrupts(status);
}

static void handle_console_input(void) {
    while (tud_cdc_available()) {
        int ch = tud_cdc_read_char();
        if (ch < 0) {
            break;
        }

        switch (ch) {
            case 'h':
            case '?':
                print_help();
                break;
            case 'b':
                stream_binary_data = !stream_binary_data;
                printf("Binary streaming %s\n", stream_binary_data ? "ENABLED" : "DISABLED");
                break;
            case 'd':
                debug_dump_samples = !debug_dump_samples;
                printf("Sample dump %s\n", debug_dump_samples ? "ENABLED" : "DISABLED");
                break;
            case 's':
                debug_stats_enabled = !debug_stats_enabled;
                printf("Periodic stats %s\n", debug_stats_enabled ? "ENABLED" : "DISABLED");
                break;
            case 'i':
                print_stats();
                break;
            case 'c':
                clear_stats();
                printf("Counters cleared\n");
                break;
            case '1':
                if (config.sample_rate != 512000) {
                    restart_microphone(512000);
                } else {
                    printf("PDM clock already at 512 kHz\n");
                }
                break;
            case '2':
                if (config.sample_rate != 1024000) {
                    restart_microphone(1024000);
                } else {
                    printf("PDM clock already at 1.024 MHz\n");
                }
                break;
            case '3':
                if (config.sample_rate != 2048000) {
                    restart_microphone(2048000);
                } else {
                    printf("PDM clock already at 2.048 MHz\n");
                }
                break;
            case 'r':
                restart_microphone(config.sample_rate);
                break;
            case '\r':
            case '\n':
                // ignore line breaks
                break;
            default:
                printf("Unknown command '%c'. Press 'h' for help.\n", ch);
                break;
        }
    }
}

static void dump_block_summary(const uint8_t* data, size_t length, uint32_t block_id) {
    const size_t max_preview = 32;
    size_t preview = length < max_preview ? length : max_preview;
    uint32_t ones = 0;
    uint32_t zeros = 0;

    printf("Block %lu len=%lu:", (unsigned long)block_id, (unsigned long)length);
    for (size_t i = 0; i < preview; i++) {
        printf(" %02X", data[i]);
    }
    if (length > preview) {
        printf(" ...");
    }
    for (size_t i = 0; i < length; i++) {
        uint8_t value = data[i];
        unsigned int pop = (unsigned int)__builtin_popcount((unsigned int)value);
        ones += (uint32_t)pop;
        zeros += (uint32_t)(8 - pop);
    }

    printf(" | ones=%lu zeros=%lu\n", (unsigned long)ones, (unsigned long)zeros);
}

static void usb_write_blocking(const uint8_t* data, size_t length) {
    size_t offset = 0;

    while (offset < length) {
        tud_task();

        if (!tud_cdc_connected()) {
            break;
        }

        uint32_t available = tud_cdc_write_available();
        if (available == 0) {
            sleep_ms(1);
            continue;
        }

        uint32_t chunk = (uint32_t)(length - offset);
        if (chunk > available) {
            chunk = available;
        }

        tud_cdc_write(&data[offset], chunk);
        tud_cdc_write_flush();
        offset += chunk;
    }
}

int main(void) {
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);

    while (!tud_cdc_connected()) {
        tud_task();
        tight_loop_contents();
    }

    printf("PDM capture started\n");
    print_help();

    if (pdm_microphone_init(&config) < 0) {
        printf("PDM microphone initialization failed!\n");
        while (true) {
            tight_loop_contents();
        }
    }

    pdm_microphone_set_samples_ready_handler(on_pdm_samples_ready);

    if (pdm_microphone_start() < 0) {
        printf("PDM microphone start failed!\n");
        while (true) {
            tight_loop_contents();
        }
    }

    mic_running = true;
    reset_capture_state();

    while (true) {
        tud_task();
        handle_console_input();

        if (!block_pending) {
            tight_loop_contents();
            continue;
        }

        uint8_t transfer_buffer[PDM_BLOCK_BYTES];
        int bytes_to_send = 0;

        uint32_t status = save_and_disable_interrupts();
        if (block_pending) {
            bytes_to_send = block_length;
            memcpy(transfer_buffer, sample_buffer, (size_t)bytes_to_send);
            block_length = 0;
            block_pending = false;
        }
        restore_interrupts(status);

        if (bytes_to_send <= 0) {
            overruns++;
            continue;
        }

        uint32_t block_id = block_counter++;

        if (debug_dump_samples) {
            dump_block_summary(transfer_buffer, (size_t)bytes_to_send, block_id);
        }

        if (stream_binary_data) {
            usb_write_blocking(transfer_buffer, (size_t)bytes_to_send);
            bytes_sent += (uint32_t)bytes_to_send;
            blocks_streamed++;
        }

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (debug_stats_enabled && (now_ms - last_debug_ms >= debug_interval_ms)) {
            print_stats();
            last_debug_ms = now_ms;
        }
    }
}
