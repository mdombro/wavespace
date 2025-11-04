/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#ifndef _PICO_PDM_MICROPHONE_H_
#define _PICO_PDM_MICROPHONE_H_

#include "hardware/pio.h"

// Application callback invoked from the DMA IRQ when a fresh buffer is ready.
typedef void (*pdm_samples_ready_handler_t)(void);

// Hardware configuration for a single PDM microphone capture stream.
struct pdm_microphone_config {
    uint gpio_data;
    uint gpio_clk;
    PIO pio;
    uint pio_sm;
    uint sample_rate;
    uint sample_buffer_size;
};

// Initialise the library and underlying PIO + DMA resources.
int pdm_microphone_init(const struct pdm_microphone_config* config);
// Release any memory or DMA channels claimed during initialisation.
void pdm_microphone_deinit();

// Begin streaming PDM data into the internal buffers.
int pdm_microphone_start();
// Halt streaming; call start() again to resume without full reinitialisation.
void pdm_microphone_stop();

// Register a callback that fires each time a buffer becomes ready to read.
void pdm_microphone_set_samples_ready_handler(pdm_samples_ready_handler_t handler);

// Copy the next raw buffer into caller-provided memory; returns bytes copied.
int pdm_microphone_read(uint8_t* buffer, size_t max_bytes);

#endif
