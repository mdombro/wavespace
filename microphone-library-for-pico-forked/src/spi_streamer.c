#include "pico/spi_streamer.h"

#include <string.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/time.h"
#include "spi_streamer.pio.h"

#define SPI_STREAMER_HEADER_BYTES  (sizeof(struct spi_streamer_frame_header))
#define SPI_STREAMER_CRC_BYTES     2u

struct __attribute__((packed)) spi_streamer_frame_header {
    uint8_t sync0;
    uint8_t sync1;
    uint8_t version;
    uint8_t header_length;
    uint64_t start_bit_index;
    uint8_t flags;
    uint8_t channel_mask;
    uint16_t payload_len;
};

static spi_streamer_t* g_spi_streamer_owner = NULL;

static void spi_streamer_gpio_irq_handler(void) {
    spi_streamer_t* ctx = g_spi_streamer_owner;
    if (!ctx || !ctx->initialised) {
        return;
    }
    uint32_t events = gpio_get_irq_event_mask(ctx->config.gpio_cs);
    if (events & GPIO_IRQ_EDGE_RISE) {
        ctx->abort_pending = true;
    }
    if (events) {
        gpio_acknowledge_irq(ctx->config.gpio_cs, events);
    }
}

static bool spi_streamer_mid_transfer(const spi_streamer_t* ctx) {
    if (!ctx) {
        return false;
    }
    if (ctx->dma_in_progress || ctx->active_frame >= 0) {
        return true;
    }
    if (!pio_sm_is_tx_fifo_empty(ctx->config.pio, ctx->config.sm)) {
        return true;
    }
    return false;
}

static inline uint16_t spi_streamer_crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static void spi_streamer_reset_state(spi_streamer_t* ctx) {
    if (!ctx || !ctx->initialised) {
        return;
    }

    if (ctx->dma_in_progress && ctx->dma_channel >= 0) {
        dma_channel_abort((uint)ctx->dma_channel);
        ctx->dma_in_progress = false;
    }

    ctx->frame_read_index = 0;
    ctx->frame_write_index = 0;
    ctx->frame_count = 0;
    ctx->active_frame = -1;
    ctx->partial_bytes = 0;
    ctx->partial_start_bit_index = ctx->bit_index;
    ctx->sticky_flags = 0;
    ctx->abort_pending = false;
    ctx->build_frame_index = -1;
    ctx->dropping_frame = false;
    ctx->last_task_time_us = 0;

    pio_sm_set_enabled(ctx->config.pio, ctx->config.sm, false);
    pio_sm_clear_fifos(ctx->config.pio, ctx->config.sm);
    pio_sm_restart(ctx->config.pio, ctx->config.sm);
    pio_sm_exec(ctx->config.pio, ctx->config.sm, pio_encode_jmp(ctx->program_offset));
    pio_sm_set_enabled(ctx->config.pio, ctx->config.sm, true);
}

static bool spi_streamer_finalize_frame(spi_streamer_t* ctx) {
    if (ctx->build_frame_index < 0) {
        return false;
    }

    struct spi_streamer_frame* frame = &ctx->queue[ctx->build_frame_index];
    struct spi_streamer_frame_header header = {
        .sync0 = SPI_STREAMER_SYNC_WORD0,
        .sync1 = SPI_STREAMER_SYNC_WORD1,
        .version = SPI_STREAMER_PROTOCOL_VERSION,
        .header_length = (uint8_t)(SPI_STREAMER_HEADER_BYTES - 2u),
        .start_bit_index = ctx->partial_start_bit_index,
        .flags = ctx->sticky_flags,
        .channel_mask = ctx->config.channel_mask,
        .payload_len = (uint16_t)ctx->config.payload_bytes,
    };

    ctx->sticky_flags = 0;

    memcpy(frame->buffer, &header, sizeof(header));

    size_t crc_region_len = (SPI_STREAMER_HEADER_BYTES - 2u) + ctx->config.payload_bytes;
    uint16_t crc = spi_streamer_crc16(frame->buffer + 2u, crc_region_len);
    size_t crc_offset = sizeof(header) + ctx->config.payload_bytes;
    frame->buffer[crc_offset] = (uint8_t)(crc & 0xFFu);
    frame->buffer[crc_offset + 1u] = (uint8_t)(crc >> 8);
    frame->length = sizeof(header) + ctx->config.payload_bytes + SPI_STREAMER_CRC_BYTES;

    ctx->frame_write_index = (uint8_t)((ctx->frame_write_index + 1u) % SPI_STREAMER_FRAME_QUEUE_DEPTH);
    ctx->frame_count++;
    ctx->stats.frames_built++;
    ctx->stats.bytes_enqueued += frame->length;
    ctx->stats.last_frame_start_bit = header.start_bit_index;
    ctx->build_frame_index = -1;
    return true;
}

static void spi_streamer_drop_active_frame(spi_streamer_t* ctx) {
    if (ctx->frame_count == 0) {
        return;
    }
    ctx->frame_read_index = (uint8_t)((ctx->frame_read_index + 1u) % SPI_STREAMER_FRAME_QUEUE_DEPTH);
    ctx->frame_count--;
    ctx->active_frame = -1;
    ctx->stats.frames_discarded++;
}

bool spi_streamer_init(spi_streamer_t* ctx, const struct spi_streamer_config* config) {
    if (!ctx || !config) {
        return false;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->config = *config;
    ctx->build_frame_index = -1;
    if (ctx->config.payload_bytes == 0 || ctx->config.payload_bytes > SPI_STREAMER_MAX_PAYLOAD_BYTES) {
        ctx->config.payload_bytes = SPI_STREAMER_MAX_PAYLOAD_BYTES;
    }

    if (ctx->config.channel_mask == 0 && ctx->config.channel_count > 0) {
        ctx->config.channel_mask = (uint8_t)((1u << ctx->config.channel_count) - 1u);
    }

    if (!pio_can_add_program(ctx->config.pio, &spi_streamer_program)) {
        return false;
    }

    ctx->program_offset = pio_add_program(ctx->config.pio, &spi_streamer_program);
    spi_streamer_program_init(
        ctx->config.pio,
        ctx->config.sm,
        ctx->program_offset,
        ctx->config.gpio_sck,
        ctx->config.gpio_miso,
        ctx->config.gpio_cs
    );

    ctx->dma_channel = dma_claim_unused_channel(true);
    if (ctx->dma_channel < 0) {
        pio_remove_program(ctx->config.pio, &spi_streamer_program, ctx->program_offset);
        return false;
    }

    ctx->dma_config = dma_channel_get_default_config((uint)ctx->dma_channel);
    channel_config_set_transfer_data_size(&ctx->dma_config, DMA_SIZE_8);
    channel_config_set_read_increment(&ctx->dma_config, true);
    channel_config_set_write_increment(&ctx->dma_config, false);
    channel_config_set_dreq(
        &ctx->dma_config,
        pio_get_dreq(ctx->config.pio, ctx->config.sm, true)
    );

    if (g_spi_streamer_owner != NULL) {
        pio_remove_program(ctx->config.pio, &spi_streamer_program, ctx->program_offset);
        dma_channel_unclaim((uint)ctx->dma_channel);
        ctx->dma_channel = -1;
        return false;
    }

    gpio_set_irq_enabled(ctx->config.gpio_cs, GPIO_IRQ_EDGE_RISE, false);
    gpio_acknowledge_irq(ctx->config.gpio_cs, GPIO_IRQ_EDGE_RISE);
    gpio_add_raw_irq_handler(ctx->config.gpio_cs, spi_streamer_gpio_irq_handler);
    g_spi_streamer_owner = ctx;
    gpio_set_irq_enabled(ctx->config.gpio_cs, GPIO_IRQ_EDGE_RISE, true);
    irq_set_enabled(IO_IRQ_BANK0, true);

    ctx->initialised = true;
    ctx->enabled = false;
    ctx->active_frame = -1;
    ctx->bit_index = 0;
    spi_streamer_reset_state(ctx);

    return true;
}

void spi_streamer_deinit(spi_streamer_t* ctx) {
    if (!ctx || !ctx->initialised) {
        return;
    }

    gpio_set_irq_enabled(ctx->config.gpio_cs, GPIO_IRQ_EDGE_RISE, false);
    gpio_acknowledge_irq(ctx->config.gpio_cs, GPIO_IRQ_EDGE_RISE);
    gpio_remove_raw_irq_handler(ctx->config.gpio_cs, spi_streamer_gpio_irq_handler);
    if (g_spi_streamer_owner == ctx) {
        g_spi_streamer_owner = NULL;
    }

    if (ctx->dma_channel >= 0) {
        dma_channel_abort((uint)ctx->dma_channel);
        dma_channel_unclaim((uint)ctx->dma_channel);
        ctx->dma_channel = -1;
    }

    pio_sm_set_enabled(ctx->config.pio, ctx->config.sm, false);
    pio_remove_program(ctx->config.pio, &spi_streamer_program, ctx->program_offset);

    ctx->initialised = false;
}

void spi_streamer_reset(spi_streamer_t* ctx) {
    spi_streamer_reset_state(ctx);
}

void spi_streamer_set_bit_index(spi_streamer_t* ctx, uint64_t bit_index) {
    if (!ctx) {
        return;
    }
    ctx->bit_index = bit_index;
    ctx->partial_start_bit_index = bit_index;
}

uint64_t spi_streamer_get_bit_index(const spi_streamer_t* ctx) {
    if (!ctx) {
        return 0;
    }
    return ctx->bit_index;
}

void spi_streamer_set_enabled(spi_streamer_t* ctx, bool enabled) {
    if (!ctx || !ctx->initialised) {
        return;
    }
    if (ctx->enabled == enabled) {
        return;
    }
    ctx->enabled = enabled;
    spi_streamer_reset_state(ctx);
}

bool spi_streamer_push_block(spi_streamer_t* ctx, const uint8_t* data, size_t length) {
    if (!ctx || !ctx->initialised || !data || length == 0) {
        return false;
    }

    bool success = true;

    if (!ctx->enabled) {
        ctx->bit_index += (uint64_t)length * 8u;
        ctx->partial_bytes = 0;
        return true;
    }

    size_t offset = 0;
    while (offset < length) {
        if (ctx->partial_bytes == 0) {
            ctx->partial_start_bit_index = ctx->bit_index;
            ctx->dropping_frame = false;
            ctx->build_frame_index = -1;
            if (ctx->frame_count < SPI_STREAMER_FRAME_QUEUE_DEPTH) {
                ctx->build_frame_index = ctx->frame_write_index;
            } else {
                ctx->dropping_frame = true;
                ctx->stats.queue_overflows++;
                ctx->sticky_flags |= SPI_STREAMER_FLAG_OVERFLOW;
                success = false;
            }
        }

        size_t remaining = length - offset;
        size_t room = ctx->config.payload_bytes - ctx->partial_bytes;
        size_t chunk = remaining < room ? remaining : room;

        if (!ctx->dropping_frame && ctx->build_frame_index >= 0) {
            struct spi_streamer_frame* frame = &ctx->queue[ctx->build_frame_index];
            uint8_t* payload_base = frame->buffer + SPI_STREAMER_HEADER_BYTES;
            memcpy(payload_base + ctx->partial_bytes, data + offset, chunk);
        }

        ctx->partial_bytes += chunk;
        ctx->bit_index += (uint64_t)chunk * 8u;
        offset += chunk;

        if (ctx->partial_bytes == ctx->config.payload_bytes) {
            if (!ctx->dropping_frame && ctx->build_frame_index >= 0) {
                if (!spi_streamer_finalize_frame(ctx)) {
                    success = false;
                }
            } else {
                ctx->stats.frames_discarded++;
            }
            ctx->partial_bytes = 0;
            ctx->build_frame_index = -1;
            ctx->dropping_frame = false;
        }
    }

    return success;
}

static void spi_streamer_start_dma(spi_streamer_t* ctx) {
    if (ctx->frame_count == 0 || ctx->dma_in_progress || ctx->dma_channel < 0) {
        return;
    }
    ctx->active_frame = ctx->frame_read_index;
    struct spi_streamer_frame* frame = &ctx->queue[ctx->frame_read_index];

    dma_channel_configure(
        (uint)ctx->dma_channel,
        &ctx->dma_config,
        &ctx->config.pio->txf[ctx->config.sm],
        frame->buffer,
        frame->length,
        true
    );

    ctx->dma_in_progress = true;
}

void spi_streamer_task(spi_streamer_t* ctx) {
    if (!ctx || !ctx->initialised || !ctx->enabled) {
        return;
    }

    if (ctx->abort_pending) {
        bool mid_transfer = spi_streamer_mid_transfer(ctx);
        ctx->abort_pending = false;
        if (mid_transfer) {
            if (ctx->dma_in_progress && ctx->dma_channel >= 0) {
                dma_channel_abort((uint)ctx->dma_channel);
                ctx->dma_in_progress = false;
            }
            if (ctx->active_frame >= 0) {
                spi_streamer_drop_active_frame(ctx);
            }
            ctx->stats.underruns++;
            ctx->sticky_flags |= SPI_STREAMER_FLAG_UNDERRUN;
            pio_sm_set_enabled(ctx->config.pio, ctx->config.sm, false);
            pio_sm_clear_fifos(ctx->config.pio, ctx->config.sm);
            pio_sm_restart(ctx->config.pio, ctx->config.sm);
            pio_sm_exec(ctx->config.pio, ctx->config.sm, pio_encode_jmp(ctx->program_offset));
            pio_sm_set_enabled(ctx->config.pio, ctx->config.sm, true);
        }
    }

    if (ctx->dma_in_progress && ctx->dma_channel >= 0) {
        if (dma_channel_is_busy((uint)ctx->dma_channel)) {
            return;
        }

        struct spi_streamer_frame* frame = &ctx->queue[ctx->frame_read_index];
        ctx->stats.frames_sent++;
        ctx->stats.bytes_transmitted += frame->length;

        ctx->frame_read_index = (uint8_t)((ctx->frame_read_index + 1u) % SPI_STREAMER_FRAME_QUEUE_DEPTH);
        ctx->frame_count--;
        ctx->dma_in_progress = false;
        ctx->active_frame = -1;
    }

    spi_streamer_start_dma(ctx);
}

void spi_streamer_clear_stats(spi_streamer_t* ctx) {
    if (!ctx) {
        return;
    }
    memset(&ctx->stats, 0, sizeof(ctx->stats));
    ctx->last_task_time_us = 0;
}

struct spi_streamer_stats spi_streamer_get_stats(const spi_streamer_t* ctx) {
    struct spi_streamer_stats stats = {0};
    if (!ctx) {
        return stats;
    }
    stats = ctx->stats;
    stats.last_frame_start_bit = ctx->partial_start_bit_index;
    stats.bytes_enqueued = ctx->stats.bytes_enqueued;
    stats.bytes_transmitted = ctx->stats.bytes_transmitted;
    return stats;
}
