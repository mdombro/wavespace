/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#ifndef _PICO_PDM_MICROPHONE_H_
#define _PICO_PDM_MICROPHONE_H_

#include <stdbool.h>
#include <stdint.h>

#include "hardware/pio.h"

// Application callback invoked from the DMA IRQ when a fresh buffer is ready.
typedef void (*pdm_samples_ready_handler_t)(void);

// Maximum number of simultaneously sampled microphones supported by the helper library.
#define PDM_MIC_MAX_CHANNELS 2u

#define PDM_MIC_MAX_CHANNELS 2u

// Hardware configuration for one or two PDM microphones sharing the same capture context.
struct pdm_microphone_config {
    uint gpio_data;            // Primary microphone data pin.
    uint gpio_data_secondary;  // Secondary mic data pin (must equal gpio_data + 1 when channels == 2).
    uint gpio_trigger;         // Optional trigger pin (must equal gpio_data + channels when enabled).
    uint gpio_clk;             // Shared clock pin.
    PIO pio;
    uint pio_sm;
    uint channels;             // 1 (default) or 2.
    uint sample_rate;          // PDM clock in Hz.
    uint sample_buffer_size;   // Bytes captured per channel in each block.
    bool capture_trigger;      // When true, capture an auxiliary trigger input alongside PDM data.
};

// Metadata describing when a raw buffer started capturing and how it maps to the
// overall PDM stream.
struct pdm_block_metadata {
    uint64_t block_index;              // Monotonic block sequence number.
    uint64_t first_sample_byte_index;  // Offset of the first captured byte.
    uint64_t capture_end_time_us;    // Time (us) when the buffer finished filling (end-of-block timestamp).
    uint32_t payload_bytes_per_channel;// Bytes per channel in this block.
    uint16_t channel_count;            // Number of channels captured in this block.
    int16_t trigger_first_index;       // Index of the first trigger-high bit within this block (-1 if none).
    uint16_t trigger_tail_high;        // 1 when the final trigger bit in this block was high.
    uint16_t reserved;                 // Reserved for future metadata.
    uint32_t trigger_reserved;         // Reserved/padding for alignment.
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

// Variant of pdm_microphone_read() that also returns capture metadata.
int pdm_microphone_read_with_metadata(
    uint8_t* buffer,
    size_t max_bytes,
    struct pdm_block_metadata* metadata);

#endif
