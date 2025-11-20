/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/regs/timer.h"
#include "hardware/structs/timer.h"
#include "hardware/timer.h"
#include "hardware/sync.h"

#include "pdm_microphone.pio.h"

#include "pico/pdm_microphone.h"

#define PDM_RAW_BUFFER_COUNT 16

struct pdm_buffer_queue {
    uint8_t data[PDM_RAW_BUFFER_COUNT];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
};

// Module-level capture context shared by the simple single-microphone API.
static struct {
    struct pdm_microphone_config config;
    int dma_channel;
    int timestamp_dma_channel;
    uint8_t* raw_buffer[PDM_RAW_BUFFER_COUNT];
    uint raw_buffer_size;      // Bytes returned to callers (channels only).
    uint capture_buffer_size;  // Bytes written by DMA (includes trigger stream).
    uint bytes_per_channel;
    uint capture_bits_per_sample;
    uint dma_irq;
    pdm_samples_ready_handler_t samples_ready_handler;
    struct pdm_block_metadata buffer_metadata[PDM_RAW_BUFFER_COUNT];
    uint64_t next_block_index;
    bool dual_channel_mode;
    bool trigger_enabled;
    bool trigger_tail_carry;
    dma_channel_config timestamp_dma_config;
    struct pdm_buffer_queue ready_queue;
    struct pdm_buffer_queue free_queue;
    int current_dma_index;
} pdm_mic;

static void pdm_dma_handler();
static inline void pdm_record_buffer_metadata(uint index);
static inline void pdm_prepare_timestamp_dma(uint index);
static void pdm_unpack_buffer(uint read_index, uint8_t* dest);
static inline uint8_t pdm_read_bit(const uint8_t* buffer, uint32_t bit_index);
static inline void pdm_write_bit(uint8_t* buffer, uint32_t bit_index, uint8_t value);
static void pdm_compact_channel_payload(const uint8_t* src, uint8_t* dest);
static void pdm_scan_trigger_stream(const uint8_t* src, struct pdm_block_metadata* meta);

static inline void pdm_queue_reset(struct pdm_buffer_queue* queue) {
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}

static inline bool pdm_queue_push(struct pdm_buffer_queue* queue, uint8_t value) {
    if (queue->count >= PDM_RAW_BUFFER_COUNT) {
        return false;
    }
    queue->data[queue->tail] = value;
    queue->tail = (uint8_t)((queue->tail + 1) % PDM_RAW_BUFFER_COUNT);
    queue->count++;
    return true;
}

static inline int pdm_queue_pop(struct pdm_buffer_queue* queue) {
    if (queue->count == 0) {
        return -1;
    }
    int value = queue->data[queue->head];
    queue->head = (uint8_t)((queue->head + 1) % PDM_RAW_BUFFER_COUNT);
    queue->count--;
    return value;
}

static void pdm_reset_buffer_queues(void) {
    pdm_queue_reset(&pdm_mic.ready_queue);
    pdm_queue_reset(&pdm_mic.free_queue);
    for (uint8_t i = 0; i < PDM_RAW_BUFFER_COUNT; i++) {
        pdm_queue_push(&pdm_mic.free_queue, i);
    }
    pdm_mic.current_dma_index = -1;
}

static int pdm_claim_free_buffer(void) {
    int idx = pdm_queue_pop(&pdm_mic.free_queue);
    if (idx >= 0) {
        return idx;
    }
    int dropped = pdm_queue_pop(&pdm_mic.ready_queue);
    if (dropped >= 0) {
        // Drop the oldest unread buffer to make space.
        return dropped;
    }
    return -1;
}

// Configure the PIO state machine and DMA to begin buffering raw PDM data.
int pdm_microphone_init(const struct pdm_microphone_config* config) {
    memset(&pdm_mic, 0x00, sizeof(pdm_mic));
    memcpy(&pdm_mic.config, config, sizeof(pdm_mic.config));
    const struct pdm_microphone_config* cfg = &pdm_mic.config;
    if (pdm_mic.config.channels == 0) {
        pdm_mic.config.channels = 1;
    }

    if (cfg->sample_rate == 0 || cfg->sample_buffer_size == 0) {
        return -1;
    }

    if (pdm_mic.config.channels > PDM_MIC_MAX_CHANNELS) {
        return -1;
    }

    if (pdm_mic.config.channels == 2) {
        if (pdm_mic.config.gpio_data_secondary != pdm_mic.config.gpio_data + 1) {
            return -1;
        }
    }

    pdm_mic.bytes_per_channel = cfg->sample_buffer_size;
    pdm_mic.dual_channel_mode = (cfg->channels == 2);
    pdm_mic.trigger_enabled = cfg->capture_trigger;
    if (pdm_mic.trigger_enabled) {
        uint expected_trigger = pdm_mic.config.gpio_data + pdm_mic.config.channels;
        if (pdm_mic.config.gpio_trigger != expected_trigger) {
            return -1;
        }
    } else {
        pdm_mic.config.gpio_trigger = 0;
    }

    pdm_mic.capture_bits_per_sample = cfg->channels + (pdm_mic.trigger_enabled ? 1u : 0u);
    pdm_mic.raw_buffer_size = pdm_mic.bytes_per_channel * cfg->channels;
    pdm_mic.capture_buffer_size = pdm_mic.bytes_per_channel * pdm_mic.capture_bits_per_sample;

    for (int i = 0; i < PDM_RAW_BUFFER_COUNT; i++) {
        pdm_mic.raw_buffer[i] = malloc(pdm_mic.capture_buffer_size);
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

    pdm_mic.timestamp_dma_channel = dma_claim_unused_channel(true);
    if (pdm_mic.timestamp_dma_channel < 0) {
        pdm_microphone_deinit();
        return -1;
    }

    uint pio_sm_offset;
    if (pdm_mic.trigger_enabled) {
        if (pdm_mic.dual_channel_mode) {
            pio_sm_offset = pio_add_program(cfg->pio, &pdm_microphone_data_dual_trigger_program);
        } else {
            pio_sm_offset = pio_add_program(cfg->pio, &pdm_microphone_data_trigger_program);
        }
    } else if (pdm_mic.dual_channel_mode) {
        pio_sm_offset = pio_add_program(cfg->pio, &pdm_microphone_data_dual_program);
    } else {
        pio_sm_offset = pio_add_program(cfg->pio, &pdm_microphone_data_program);
    }

    // PDM program requires a 4x oversampled clock relative to the desired bit rate.
    float clk_div = clock_get_hz(clk_sys) / (cfg->sample_rate * 4.0f);

    if (pdm_mic.trigger_enabled) {
        if (pdm_mic.dual_channel_mode) {
            pdm_microphone_data_dual_trigger_init(
                cfg->pio,
                cfg->pio_sm,
                pio_sm_offset,
                clk_div,
                cfg->gpio_data,
                cfg->gpio_clk
            );
        } else {
            pdm_microphone_data_trigger_init(
                cfg->pio,
                cfg->pio_sm,
                pio_sm_offset,
                clk_div,
                cfg->gpio_data,
                cfg->gpio_clk
            );
        }
    } else if (pdm_mic.dual_channel_mode) {
        pdm_microphone_data_dual_init(
            cfg->pio,
            cfg->pio_sm,
            pio_sm_offset,
            clk_div,
            cfg->gpio_data,
            cfg->gpio_clk
        );
    } else {
        pdm_microphone_data_init(
            cfg->pio,
            cfg->pio_sm,
            pio_sm_offset,
            clk_div,
            cfg->gpio_data,
            cfg->gpio_clk
        );
    }

    dma_channel_config dma_channel_cfg = dma_channel_get_default_config(pdm_mic.dma_channel);

    channel_config_set_transfer_data_size(&dma_channel_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_channel_cfg, false);
    channel_config_set_write_increment(&dma_channel_cfg, true);
    channel_config_set_dreq(&dma_channel_cfg, pio_get_dreq(cfg->pio, cfg->pio_sm, false));
    channel_config_set_chain_to(&dma_channel_cfg, (uint)pdm_mic.timestamp_dma_channel);

    dma_channel_config timestamp_cfg = dma_channel_get_default_config(pdm_mic.timestamp_dma_channel);
    channel_config_set_transfer_data_size(&timestamp_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&timestamp_cfg, true);
    channel_config_set_write_increment(&timestamp_cfg, true);
    channel_config_set_dreq(&timestamp_cfg, DREQ_FORCE);
    pdm_mic.timestamp_dma_config = timestamp_cfg;
    pdm_reset_buffer_queues();

    pdm_mic.dma_irq = DMA_IRQ_0;

    // Prime the DMA engine with the first buffer but leave it disabled until start().
    dma_channel_configure(
        pdm_mic.dma_channel,
        &dma_channel_cfg,
        pdm_mic.raw_buffer[0],
        &cfg->pio->rxf[cfg->pio_sm],
        pdm_mic.capture_buffer_size,
        false
    );

    pdm_mic.next_block_index = 0;

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

    if (pdm_mic.timestamp_dma_channel > -1) {
        dma_channel_unclaim(pdm_mic.timestamp_dma_channel);

        pdm_mic.timestamp_dma_channel = -1;
    }
}

// Prime the DMA ring and enable the PIO state machine.
int pdm_microphone_start() {
    pdm_reset_buffer_queues();
    pdm_mic.trigger_tail_carry = false;

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

    int initial_index = pdm_claim_free_buffer();
    if (initial_index < 0) {
        return -1;
    }
    pdm_mic.current_dma_index = initial_index;
    pdm_record_buffer_metadata((uint)initial_index);
    pdm_prepare_timestamp_dma((uint)initial_index);

    dma_channel_transfer_to_buffer_now(
        pdm_mic.dma_channel,
        pdm_mic.raw_buffer[initial_index],
        pdm_mic.capture_buffer_size
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

    pdm_reset_buffer_queues();
}

// DMA completion ISR: advances the ring buffer and notifies the application.
static void pdm_dma_handler() {
    if (pdm_mic.dma_irq == DMA_IRQ_0) {
        dma_hw->ints0 = (1u << pdm_mic.dma_channel);
    } else if (pdm_mic.dma_irq == DMA_IRQ_1) {
        dma_hw->ints1 = (1u << pdm_mic.dma_channel);
    }

    if (pdm_mic.current_dma_index >= 0) {
        if (!pdm_queue_push(&pdm_mic.ready_queue, (uint8_t)pdm_mic.current_dma_index)) {
            (void)pdm_queue_pop(&pdm_mic.ready_queue);
            pdm_queue_push(&pdm_mic.ready_queue, (uint8_t)pdm_mic.current_dma_index);
        }
    }

    int next_index = pdm_claim_free_buffer();
    if (next_index < 0) {
        next_index = pdm_mic.current_dma_index;
    }
    pdm_mic.current_dma_index = next_index;

    if (next_index >= 0) {
        pdm_record_buffer_metadata((uint)next_index);
        pdm_prepare_timestamp_dma((uint)next_index);
        dma_channel_transfer_to_buffer_now(
            pdm_mic.dma_channel,
            pdm_mic.raw_buffer[next_index],
            pdm_mic.capture_buffer_size
        );
    }

    if (pdm_mic.samples_ready_handler) {
        pdm_mic.samples_ready_handler();
    }
}

// Allow the user to hook the high-priority ready callback.
void pdm_microphone_set_samples_ready_handler(pdm_samples_ready_handler_t handler) {
    pdm_mic.samples_ready_handler = handler;
}

bool pdm_microphone_acquire_buffer(struct pdm_acquired_buffer* out) {
    if (!out) {
        return false;
    }

    uint32_t status = save_and_disable_interrupts();
    int index = pdm_queue_pop(&pdm_mic.ready_queue);
    restore_interrupts(status);

    if (index < 0) {
        return false;
    }

    out->data = pdm_mic.raw_buffer[index];
    out->length = pdm_mic.raw_buffer_size;
    out->metadata = &pdm_mic.buffer_metadata[index];
    out->buffer_index = (uint8_t)index;
    return true;
}

void pdm_microphone_release_buffer(uint8_t buffer_index) {
    uint32_t status = save_and_disable_interrupts();
    pdm_queue_push(&pdm_mic.free_queue, buffer_index);
    restore_interrupts(status);
}

// Copy the next available raw buffer into the caller-provided scratch space.
int pdm_microphone_read_with_metadata(uint8_t* buffer, size_t max_bytes, struct pdm_block_metadata* metadata) {
    if (!buffer || max_bytes == 0) {
        return 0;
    }

    if (max_bytes < pdm_mic.raw_buffer_size) {
        return -1;
    }

    struct pdm_acquired_buffer acquired = {0};
    if (!pdm_microphone_acquire_buffer(&acquired)) {
        return 0;
    }

    pdm_unpack_buffer(acquired.buffer_index, buffer);

    if (metadata) {
        *metadata = *acquired.metadata;
    }

    pdm_microphone_release_buffer(acquired.buffer_index);

    return (int)pdm_mic.raw_buffer_size;
}

int pdm_microphone_read(uint8_t* buffer, size_t max_bytes) {
    return pdm_microphone_read_with_metadata(buffer, max_bytes, NULL);
}

static inline void pdm_record_buffer_metadata(uint index) {
    struct pdm_block_metadata* meta = &pdm_mic.buffer_metadata[index];
    uint64_t block_index = pdm_mic.next_block_index++;

    meta->block_index = block_index;
    meta->first_sample_byte_index = block_index * (uint64_t)pdm_mic.bytes_per_channel;
    meta->capture_end_time_us = 0;
    meta->payload_bytes_per_channel = pdm_mic.bytes_per_channel;
    meta->channel_count = (uint16_t)pdm_mic.config.channels;
    meta->reserved = 0;
    meta->trigger_first_index = -1;
    meta->trigger_tail_high = 0;
    meta->trigger_reserved = 0;
}

static inline void pdm_prepare_timestamp_dma(uint index) {
    if (pdm_mic.timestamp_dma_channel < 0) {
        return;
    }

    dma_channel_wait_for_finish_blocking((uint)pdm_mic.timestamp_dma_channel);

    struct pdm_block_metadata* meta = &pdm_mic.buffer_metadata[index];
    dma_channel_configure(
        pdm_mic.timestamp_dma_channel,
        &pdm_mic.timestamp_dma_config,
        &meta->capture_end_time_us,
        &timer_hw->timerawl,
        2,
        false
    );
}

static inline uint8_t pdm_read_bit(const uint8_t* buffer, uint32_t bit_index) {
    uint32_t byte_index = bit_index >> 3;
    uint32_t bit_offset = 7u - (bit_index & 7u);
    return (uint8_t)((buffer[byte_index] >> bit_offset) & 0x01u);
}

static inline void pdm_write_bit(uint8_t* buffer, uint32_t bit_index, uint8_t value) {
    uint32_t byte_index = bit_index >> 3;
    uint32_t bit_offset = 7u - (bit_index & 7u);
    uint8_t mask = (uint8_t)(1u << bit_offset);
    if (value) {
        buffer[byte_index] |= mask;
    } else {
        buffer[byte_index] &= (uint8_t)~mask;
    }
}

static void pdm_unpack_buffer(uint read_index, uint8_t* dest) {
    const uint8_t* src = pdm_mic.raw_buffer[read_index];
    struct pdm_block_metadata* meta = &pdm_mic.buffer_metadata[read_index];

    if (!pdm_mic.trigger_enabled) {
        memcpy(dest, src, pdm_mic.raw_buffer_size);
        meta->trigger_first_index = -1;
        meta->trigger_tail_high = 0;
        pdm_mic.trigger_tail_carry = false;
        return;
    }

    pdm_compact_channel_payload(src, dest);
    pdm_scan_trigger_stream(src, meta);
}

static void pdm_compact_channel_payload(const uint8_t* src, uint8_t* dest) {
    memset(dest, 0, pdm_mic.raw_buffer_size);

    const uint32_t channel_count = pdm_mic.config.channels;
    const uint32_t samples_per_channel = pdm_mic.bytes_per_channel * 8u;
    const uint32_t bits_per_sample = pdm_mic.capture_bits_per_sample;
    const uint32_t skip_bits = bits_per_sample > channel_count
        ? bits_per_sample - channel_count
        : 0;

    uint32_t src_bit = 0;
    uint32_t dest_bit = 0;

    for (uint32_t sample = 0; sample < samples_per_channel; sample++) {
        for (uint32_t ch = 0; ch < channel_count; ch++) {
            uint8_t bit = pdm_read_bit(src, src_bit++);
            if (bit) {
                pdm_write_bit(dest, dest_bit, 1u);
            }
            dest_bit++;
        }

        src_bit += skip_bits;
    }
}

static void pdm_scan_trigger_stream(const uint8_t* src, struct pdm_block_metadata* meta) {
    const uint32_t channel_count = pdm_mic.config.channels;
    const uint32_t bits_per_sample = pdm_mic.capture_bits_per_sample;

    if (bits_per_sample <= channel_count) {
        meta->trigger_first_index = -1;
        meta->trigger_tail_high = 0;
        pdm_mic.trigger_tail_carry = false;
        return;
    }

    const uint32_t samples_per_channel = pdm_mic.bytes_per_channel * 8u;
    const uint32_t trigger_stride = bits_per_sample;
    uint32_t trigger_bit_index = channel_count;

    int16_t trigger_index = -1;
    bool carry = pdm_mic.trigger_tail_carry;
    bool zero_word_seen = !carry;
    uint8_t last_trigger = carry ? 1u : 0u;
    uint32_t word_index = 0;
    uint32_t sample = 0;

    while (sample < samples_per_channel) {
        bool word_all_zero = true;
        uint32_t bits_this_word = samples_per_channel - sample;
        if (bits_this_word > 32u) {
            bits_this_word = 32u;
        }

        for (uint32_t bit = 0; bit < bits_this_word; bit++, sample++, trigger_bit_index += trigger_stride) {
            uint8_t trigger_bit = pdm_read_bit(src, trigger_bit_index);
            last_trigger = trigger_bit;
            if (trigger_bit) {
                word_all_zero = false;
            }
        }

        if (word_all_zero) {
            zero_word_seen = true;
        } else {
            if (trigger_index < 0) {
                if (!carry || zero_word_seen) {
                    trigger_index = (int16_t)word_index;
                }
            }
            zero_word_seen = false;
        }

        word_index++;
    }

    meta->trigger_first_index = trigger_index;
    meta->trigger_tail_high = last_trigger != 0;
    pdm_mic.trigger_tail_carry = (last_trigger != 0);
}
