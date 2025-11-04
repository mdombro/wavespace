/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#include <stdlib.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

#include "pdm_microphone.pio.h"

#include "pico/pdm_microphone.h"

#define PDM_RAW_BUFFER_COUNT 2

// Module-level capture context shared by the simple single-microphone API.
static struct {
    struct pdm_microphone_config config;
    int dma_channel;
    uint8_t* raw_buffer[PDM_RAW_BUFFER_COUNT];
    uint raw_buffer_size;
    volatile uint raw_buffer_write_index;
    volatile uint raw_buffer_read_index;
    volatile uint raw_buffer_ready_count;
    uint dma_irq;
    pdm_samples_ready_handler_t samples_ready_handler;
} pdm_mic;

static void pdm_dma_handler();

// Configure the PIO state machine and DMA to begin buffering raw PDM data.
int pdm_microphone_init(const struct pdm_microphone_config* config) {
    memset(&pdm_mic, 0x00, sizeof(pdm_mic));
    memcpy(&pdm_mic.config, config, sizeof(pdm_mic.config));

    if (config->sample_rate == 0 || config->sample_buffer_size == 0) {
        return -1;
    }

    pdm_mic.raw_buffer_size = config->sample_buffer_size;

    for (int i = 0; i < PDM_RAW_BUFFER_COUNT; i++) {
        pdm_mic.raw_buffer[i] = malloc(pdm_mic.raw_buffer_size);
        if (pdm_mic.raw_buffer[i] == NULL) {
            pdm_microphone_deinit();

            return -1;   
        }
    }

    pdm_mic.dma_channel = dma_claim_unused_channel(true);
    if (pdm_mic.dma_channel < 0) {
        pdm_microphone_deinit();

        return -1;
    }

    uint pio_sm_offset = pio_add_program(config->pio, &pdm_microphone_data_program);

    // PDM program requires a 4x oversampled clock relative to the desired bit rate.
    float clk_div = clock_get_hz(clk_sys) / (config->sample_rate * 4.0f);

    pdm_microphone_data_init(
        config->pio,
        config->pio_sm,
        pio_sm_offset,
        clk_div,
        config->gpio_data,
        config->gpio_clk
    );

    dma_channel_config dma_channel_cfg = dma_channel_get_default_config(pdm_mic.dma_channel);

    channel_config_set_transfer_data_size(&dma_channel_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_channel_cfg, false);
    channel_config_set_write_increment(&dma_channel_cfg, true);
    channel_config_set_dreq(&dma_channel_cfg, pio_get_dreq(config->pio, config->pio_sm, false));

    pdm_mic.dma_irq = DMA_IRQ_0;

    // Prime the DMA engine with the first buffer but leave it disabled until start().
    dma_channel_configure(
        pdm_mic.dma_channel,
        &dma_channel_cfg,
        pdm_mic.raw_buffer[0],
        &config->pio->rxf[config->pio_sm],
        pdm_mic.raw_buffer_size,
        false
    );

    pdm_mic.raw_buffer_write_index = 0;
    pdm_mic.raw_buffer_read_index = 0;
    pdm_mic.raw_buffer_ready_count = 0;

    return 0;
}

// Free DMA-owned buffers and release the claimed channel.
void pdm_microphone_deinit() {
    for (int i = 0; i < PDM_RAW_BUFFER_COUNT; i++) {
        if (pdm_mic.raw_buffer[i]) {
            free(pdm_mic.raw_buffer[i]);

            pdm_mic.raw_buffer[i] = NULL;
        }
    }

    if (pdm_mic.dma_channel > -1) {
        dma_channel_unclaim(pdm_mic.dma_channel);

        pdm_mic.dma_channel = -1;
    }
}

// Prime the DMA ring and enable the PIO state machine.
int pdm_microphone_start() {
    pdm_mic.raw_buffer_write_index = 0;
    pdm_mic.raw_buffer_read_index = 0;
    pdm_mic.raw_buffer_ready_count = 0;

    // DMA IRQ 0 is used by default; ensure pending status is cleared.
    if (pdm_mic.dma_irq == DMA_IRQ_0) {
        dma_hw->ints0 = (1u << pdm_mic.dma_channel);
        dma_channel_set_irq0_enabled(pdm_mic.dma_channel, true);
    } else if (pdm_mic.dma_irq == DMA_IRQ_1) {
        dma_hw->ints1 = (1u << pdm_mic.dma_channel);
        dma_channel_set_irq1_enabled(pdm_mic.dma_channel, true);
    } else {
        return -1;
    }

    irq_set_exclusive_handler(pdm_mic.dma_irq, pdm_dma_handler);
    irq_set_enabled(pdm_mic.dma_irq, true);

    pio_sm_set_enabled(
        pdm_mic.config.pio,
        pdm_mic.config.pio_sm,
        false
    );

    dma_channel_transfer_to_buffer_now(
        pdm_mic.dma_channel,
        pdm_mic.raw_buffer[0],
        pdm_mic.raw_buffer_size
    );

    pio_sm_set_enabled(
        pdm_mic.config.pio,
        pdm_mic.config.pio_sm,
        true
    );

    return 0;
}

// Halt PIO/DMA activity but keep configuration for potential restart.
void pdm_microphone_stop() {
    pio_sm_set_enabled(
        pdm_mic.config.pio,
        pdm_mic.config.pio_sm,
        false
    );

    dma_channel_abort(pdm_mic.dma_channel);

    if (pdm_mic.dma_irq == DMA_IRQ_0) {
        dma_channel_set_irq0_enabled(pdm_mic.dma_channel, false);
    } else if (pdm_mic.dma_irq == DMA_IRQ_1) {
        dma_channel_set_irq1_enabled(pdm_mic.dma_channel, false);
    }

    irq_set_enabled(pdm_mic.dma_irq, false);

    pdm_mic.raw_buffer_ready_count = 0;
}

// DMA completion ISR: advances the ring buffer and notifies the application.
static void pdm_dma_handler() {
    // clear IRQ
    if (pdm_mic.dma_irq == DMA_IRQ_0) {
        dma_hw->ints0 = (1u << pdm_mic.dma_channel);
    } else if (pdm_mic.dma_irq == DMA_IRQ_1) {
        dma_hw->ints1 = (1u << pdm_mic.dma_channel);
    }

    // get the current buffer index
    if (pdm_mic.raw_buffer_ready_count < PDM_RAW_BUFFER_COUNT) {
        pdm_mic.raw_buffer_ready_count++;
    } else {
        pdm_mic.raw_buffer_read_index = (pdm_mic.raw_buffer_read_index + 1) % PDM_RAW_BUFFER_COUNT;
    }

    // get the next capture index to send the dma to start
    pdm_mic.raw_buffer_write_index = (pdm_mic.raw_buffer_write_index + 1) % PDM_RAW_BUFFER_COUNT;

    // give the channel a new buffer to write to and re-trigger it
    dma_channel_transfer_to_buffer_now(
        pdm_mic.dma_channel,
        pdm_mic.raw_buffer[pdm_mic.raw_buffer_write_index],
        pdm_mic.raw_buffer_size
    );

    if (pdm_mic.samples_ready_handler) {
        pdm_mic.samples_ready_handler();
    }
}

// Allow the user to hook the high-priority ready callback.
void pdm_microphone_set_samples_ready_handler(pdm_samples_ready_handler_t handler) {
    pdm_mic.samples_ready_handler = handler;
}

// Copy the next available raw buffer into the caller-provided scratch space.
int pdm_microphone_read(uint8_t* buffer, size_t max_bytes) {
    if (!buffer || max_bytes == 0) {
        return 0;
    }

    if (max_bytes < pdm_mic.raw_buffer_size) {
        return -1;
    }

    // Hold off DMA ISR while copying out the completed buffer.
    uint32_t status = save_and_disable_interrupts();

    if (pdm_mic.raw_buffer_ready_count == 0) {
        restore_interrupts(status);
        return 0;
    }

    memcpy(buffer, pdm_mic.raw_buffer[pdm_mic.raw_buffer_read_index], pdm_mic.raw_buffer_size);

    pdm_mic.raw_buffer_read_index = (pdm_mic.raw_buffer_read_index + 1) % PDM_RAW_BUFFER_COUNT;
    if (pdm_mic.raw_buffer_ready_count > 0) {
        pdm_mic.raw_buffer_ready_count--;
    }

    restore_interrupts(status);

    return (int)pdm_mic.raw_buffer_size;
}
