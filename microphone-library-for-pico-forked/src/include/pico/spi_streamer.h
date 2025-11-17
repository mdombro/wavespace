/*
 * High-throughput SPI streaming helper for moving interleaved PDM bytes
 * from the RP2040 into an external host.
 */

#ifndef _PICO_SPI_STREAMER_H_
#define _PICO_SPI_STREAMER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/dma.h"
#include "hardware/pio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_STREAMER_MAX_PAYLOAD_BYTES 512u
#define SPI_STREAMER_FRAME_QUEUE_DEPTH 64u

#define SPI_STREAMER_SYNC_WORD0 0xA5u
#define SPI_STREAMER_SYNC_WORD1 0x5Au
#define SPI_STREAMER_PROTOCOL_VERSION 0u

#define SPI_STREAMER_FLAG_OVERFLOW 0x01u
#define SPI_STREAMER_FLAG_UNDERRUN 0x02u

struct spi_streamer_config {
    PIO pio;
    uint sm;
    uint gpio_sck;
    uint gpio_mosi;
    uint gpio_miso;
    uint gpio_cs;
    size_t payload_bytes;
    uint8_t channel_count;
    uint8_t channel_mask;
};

struct spi_streamer_stats {
    uint32_t frames_built;
    uint32_t frames_sent;
    uint32_t frames_discarded;
    uint32_t queue_overflows;
    uint32_t underruns;
    uint64_t bytes_enqueued;
    uint64_t bytes_transmitted;
    uint64_t last_frame_start_bit;
};

struct spi_streamer_frame {
    uint8_t buffer[SPI_STREAMER_MAX_PAYLOAD_BYTES + 18u];  // header + payload + CRC.
    size_t length;
};

typedef struct spi_streamer {
    struct spi_streamer_config config;
    struct spi_streamer_frame queue[SPI_STREAMER_FRAME_QUEUE_DEPTH];
    uint8_t frame_read_index;
    uint8_t frame_write_index;
    uint8_t frame_count;
    int active_frame;
    int dma_channel;
    dma_channel_config dma_config;
    uint program_offset;
    bool dma_in_progress;
    bool initialised;
    bool enabled;
    volatile bool abort_pending;
    uint8_t sticky_flags;
    uint8_t partial_payload[SPI_STREAMER_MAX_PAYLOAD_BYTES];
    size_t partial_bytes;
    uint64_t partial_start_bit_index;
    uint64_t bit_index;
    struct spi_streamer_stats stats;
} spi_streamer_t;

bool spi_streamer_init(spi_streamer_t* ctx, const struct spi_streamer_config* config);
void spi_streamer_deinit(spi_streamer_t* ctx);
void spi_streamer_reset(spi_streamer_t* ctx);
void spi_streamer_set_bit_index(spi_streamer_t* ctx, uint64_t bit_index);
uint64_t spi_streamer_get_bit_index(const spi_streamer_t* ctx);
void spi_streamer_set_enabled(spi_streamer_t* ctx, bool enabled);
bool spi_streamer_push_block(spi_streamer_t* ctx, const uint8_t* data, size_t length);
void spi_streamer_task(spi_streamer_t* ctx);
void spi_streamer_clear_stats(spi_streamer_t* ctx);
struct spi_streamer_stats spi_streamer_get_stats(const spi_streamer_t* ctx);

#ifdef __cplusplus
}
#endif

#endif
