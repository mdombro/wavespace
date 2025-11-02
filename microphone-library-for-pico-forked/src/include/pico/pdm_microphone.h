/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#ifndef _PICO_PDM_MICROPHONE_H_
#define _PICO_PDM_MICROPHONE_H_

#include "hardware/pio.h"

typedef void (*pdm_samples_ready_handler_t)(void);

struct pdm_microphone_config {
    uint gpio_data;
    uint gpio_clk;
    PIO pio;
    uint pio_sm;
    uint sample_rate;
    uint sample_buffer_size;
};

int pdm_microphone_init(const struct pdm_microphone_config* config);
void pdm_microphone_deinit();

int pdm_microphone_start();
void pdm_microphone_stop();

void pdm_microphone_set_samples_ready_handler(pdm_samples_ready_handler_t handler);

int pdm_microphone_read(uint8_t* buffer, size_t max_bytes);

#endif
