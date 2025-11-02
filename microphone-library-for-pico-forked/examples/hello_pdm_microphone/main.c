#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "pico/pdm_microphone.h"
#include "tusb.h"

#define PDM_BLOCK_BITS   4096
#define PDM_BLOCK_BYTES  (PDM_BLOCK_BITS / 8)

static uint8_t sample_buffer[PDM_BLOCK_BYTES];
static uint8_t discard_buffer[PDM_BLOCK_BYTES];
static volatile bool block_pending = false;
static volatile int block_length = 0;

static const struct pdm_microphone_config config = {
    .gpio_data = 2,
    .gpio_clk = 3,
    .pio = pio0,
    .pio_sm = 0,
    .sample_rate = 100000,
    .sample_buffer_size = PDM_BLOCK_BYTES,
};

static void on_pdm_samples_ready(void) {
    uint8_t* target = block_pending ? discard_buffer : sample_buffer;
    int read = pdm_microphone_read(target, PDM_BLOCK_BYTES);

    if (!block_pending && read > 0) {
        block_length = read;
        block_pending = true;
    }
}

int main(void) {
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);

    while (!tud_cdc_connected()) {
        tight_loop_contents();
    }

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

    while (true) {
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
            continue;
        }

        size_t written = fwrite(transfer_buffer, 1, (size_t)bytes_to_send, stdout);
        if (written != (size_t)bytes_to_send) {
            sleep_ms(1);
        }
    }
}
