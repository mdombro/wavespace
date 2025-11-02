/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * This examples captures data from a PDM microphone using a sample
 * rate of 8 kHz and prints the sample values over the USB serial
 * connection.
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/pdm_microphone.h"
#include "tusb.h"

#define SAMPLE_BUFFER_SIZE 400

// configuration
const struct pdm_microphone_config config = {
    // GPIO pin for the PDM DAT signal
    .gpio_data = 2,

    // GPIO pin for the PDM CLK signal
    .gpio_clk = 3,

    // PIO instance to use
    .pio = pio0,

    // PIO State Machine instance to use
    .pio_sm = 0,

    // sample rate in Hz
    .sample_rate = 100000,

    // number of samples to buffer
    .sample_buffer_size = SAMPLE_BUFFER_SIZE,
};

// variables
int16_t sample_buffer[SAMPLE_BUFFER_SIZE];
volatile int samples_read = 0;

void on_pdm_samples_ready()
{
    // callback from library when all the samples in the library
    // internal sample buffer are ready for reading 
    samples_read = pdm_microphone_read(sample_buffer, SAMPLE_BUFFER_SIZE);
}

int main( void )
{
    // initialize stdio and wait for USB CDC connect
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);
    while (!tud_cdc_connected()) {
        tight_loop_contents();
    }

    // initialize the PDM microphone
    if (pdm_microphone_init(&config) < 0) {
        printf("PDM microphone initialization failed!\n");
        while (1) { tight_loop_contents(); }
    }

    // set callback that is called when all the samples in the library
    // internal sample buffer are ready for reading
    pdm_microphone_set_samples_ready_handler(on_pdm_samples_ready);
    
     // start capturing data from the PDM microphone
    if (pdm_microphone_start() < 0) {
        printf("PDM microphone start failed!\n");
        while (1) { tight_loop_contents(); }
    }

    while (1) {
        // wait for new samples
        while (samples_read == 0) { tight_loop_contents(); }

        // store and clear the samples read from the callback
        int sample_count = samples_read;
        samples_read = 0;
        
        size_t written = fwrite(
            sample_buffer,
            sizeof(sample_buffer[0]),
            sample_count,
            stdout
        );

        if (written != (size_t)sample_count) {
            sleep_ms(1);
        }
    }

    return 0;
}
