#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "hardware/sync.h"
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/cyw43_arch.h"
#include "pico/pdm_microphone.h"
#include "pico/spi_streamer.h"
#include "trigger_wave.pio.h"

#define ENABLE_WIFI_STREAMING 0

#if ENABLE_WIFI_STREAMING
#include "lwip/ip_addr.h"
#include "lwip/ip4_addr.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#endif

#include "tusb.h"

// Example entry point: capture PDM audio and stream over USB CDC and/or SPI.

#if ENABLE_WIFI_STREAMING
#ifndef WIFI_SSID
#define WIFI_SSID "Routers of Rohan"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "ridersoftheolan"
#endif

#ifndef WIFI_AUTH_TYPE
#define WIFI_AUTH_TYPE CYW43_AUTH_WPA2_AES_PSK
#endif

#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 30000
#endif

#ifndef WIFI_TARGET_IP
#define WIFI_TARGET_IP "192.168.8.209"
#endif

#ifndef WIFI_TARGET_PORT
#define WIFI_TARGET_PORT 6000
#endif

typedef enum {
    WIFI_TRANSPORT_UDP = 0,
    WIFI_TRANSPORT_TCP = 1,
} wifi_transport_mode_t;

#ifndef WIFI_TRANSPORT_MODE
#define WIFI_TRANSPORT_MODE WIFI_TRANSPORT_UDP
#endif
#endif  // ENABLE_WIFI_STREAMING

#ifndef USB_WAIT_TIMEOUT_MS
#define USB_WAIT_TIMEOUT_MS 3000
#endif

#define PDM_CHANNEL_COUNT       2
#define PDM_BLOCK_BITS_PER_CH   8192
#define PDM_BLOCK_BYTES_PER_CH  (PDM_BLOCK_BITS_PER_CH / 8)
#define PDM_BLOCK_PAYLOAD_BYTES (PDM_BLOCK_BYTES_PER_CH * PDM_CHANNEL_COUNT)
#define PDM_BLOCK_PACKET_BYTES  (PDM_BLOCK_PAYLOAD_BYTES + sizeof(struct pdm_block_metadata))
#define CAPTURE_RING_DEPTH      8

#define SERIAL_UART_ID         uart0
#define SERIAL_UART_TX_PIN     0
#define SERIAL_UART_RX_PIN     1
#define SERIAL_UART_BAUD       2000000
#define SERIAL_WRITE_SPIN_MAX  500000
#define SERIAL_WRITE_BACKOFF_US 5

#define SPI_STREAM_PIO         pio1
#define SPI_STREAM_STATE_MACHINE 0
#define SPI_STREAM_SCK_PIN     10
#define SPI_STREAM_TX_PIN      11  // MISO from Pico perspective.
#define SPI_STREAM_RX_PIN      12  // MOSI (unused but reserved).
#define SPI_STREAM_CS_PIN      13
#define SPI_STREAM_PAYLOAD_BYTES 512

#if ENABLE_WIFI_STREAMING
#define WIFI_QUEUE_CAPACITY      64
#define WIFI_TCP_RETRY_MS        1000
#endif

#ifndef TRIGGER_WAVE_PIO
#define TRIGGER_WAVE_PIO pio1
#endif

#ifndef TRIGGER_WAVE_STATE_MACHINE
#define TRIGGER_WAVE_STATE_MACHINE 1
#endif

#ifndef TRIGGER_WAVE_TRIGGER_PIN
#define TRIGGER_WAVE_TRIGGER_PIN 18
#endif

#ifndef TRIGGER_WAVE_OUTPUT_PIN
#define TRIGGER_WAVE_OUTPUT_PIN 19
#endif

#ifndef TRIGGER_WAVE_FREQUENCY_HZ
#define TRIGGER_WAVE_FREQUENCY_HZ 60000
#endif

#ifndef TRIGGER_WAVE_DURATION_US
#define TRIGGER_WAVE_DURATION_US 5000
#endif

typedef struct {
    struct pdm_block_metadata metadata;
    uint8_t payload[PDM_BLOCK_PAYLOAD_BYTES];
} pdm_block_packet_t;

typedef enum {
    CAPTURE_SLOT_FREE = 0,
    CAPTURE_SLOT_READY = 1,
    CAPTURE_SLOT_SENDING = 2,
} capture_slot_state_t;

typedef struct {
    pdm_block_packet_t packet;
    const uint8_t* dma_payload;
    uint8_t buffer_handle;
    uint16_t length;
    capture_slot_state_t state;
    uint8_t pending_outputs;
    bool has_copy;
} capture_slot_t;

static capture_slot_t capture_ring[CAPTURE_RING_DEPTH];
static volatile uint8_t capture_write_index = 0;
static volatile uint8_t capture_read_index = 0;
static volatile uint8_t capture_count = 0;

static inline const uint8_t* capture_slot_payload_ptr(const capture_slot_t* slot) {
    if (!slot) {
        return NULL;
    }
    return slot->dma_payload ? slot->dma_payload : slot->packet.payload;
}

static inline void capture_slot_release_dma(capture_slot_t* slot) {
    if (!slot || !slot->dma_payload) {
        return;
    }
    pdm_microphone_release_buffer(slot->buffer_handle);
    slot->dma_payload = NULL;
}

static inline void capture_slot_ensure_copy(capture_slot_t* slot) {
    if (!slot || slot->has_copy) {
        return;
    }
    if (slot->length > 0 && slot->dma_payload) {
        memcpy(slot->packet.payload, slot->dma_payload, slot->length);
    }
    slot->has_copy = true;
    capture_slot_release_dma(slot);
}

// Debug/telemetry counters surfaced through the CLI stats.
static volatile uint32_t blocks_ready = 0;
static volatile uint32_t blocks_discarded = 0;
static volatile uint32_t overruns = 0;

static uint32_t blocks_streamed = 0;
static uint32_t bytes_sent = 0;

static bool debug_stats_enabled = true;
static bool debug_dump_samples = false;
static bool stream_binary_data = false;
static bool serial_stream_enabled = false;
static bool spi_stream_enabled = false;
#if ENABLE_WIFI_STREAMING
static bool wifi_stream_enabled = true;
#else
static bool wifi_stream_enabled = false;
#endif

static uint32_t debug_interval_ms = 1000;
static uint32_t last_debug_ms = 0;
static bool mic_running = false;

#if ENABLE_WIFI_STREAMING
static bool wifi_initialised = false;
static bool wifi_ready = false;
static wifi_transport_mode_t wifi_transport = WIFI_TRANSPORT_MODE;
static struct udp_pcb* wifi_udp_socket = NULL;
static struct tcp_pcb* wifi_tcp_pcb = NULL;
static struct tcp_pcb* wifi_tcp_connecting = NULL;
static absolute_time_t wifi_tcp_retry_deadline = {0};
static ip_addr_t wifi_remote_addr;

typedef struct {
    pdm_block_packet_t packet;
    uint16_t length;
    uint16_t sent;
} wifi_packet_t;

static wifi_packet_t wifi_queue[WIFI_QUEUE_CAPACITY];
static uint8_t wifi_queue_head = 0;
static uint8_t wifi_queue_tail = 0;
static uint8_t wifi_queue_count = 0;

static uint32_t wifi_packets_sent = 0;
static uint32_t wifi_bytes_sent = 0;
static uint32_t wifi_send_failures = 0;
static uint32_t wifi_alloc_failures = 0;
static uint32_t wifi_queue_drops = 0;
#endif

static uint32_t serial_packets_sent = 0;
static uint32_t serial_bytes_sent = 0;
static uint32_t serial_failures = 0;

static bool serial_initialised = false;
static bool spi_initialised = false;
static uint32_t spi_failures = 0;
static spi_streamer_t spi_streamer_ctx;

static struct pdm_microphone_config config = {
    .gpio_data = 2,
    .gpio_data_secondary = 3,
    .gpio_trigger = 4,
    .gpio_clk = 5,
    .pio = pio0,
    .pio_sm = 0,
    .sample_rate = 2048000,
    .sample_buffer_size = PDM_BLOCK_BYTES_PER_CH,
    .channels = PDM_CHANNEL_COUNT,
    .capture_trigger = false,
};

static void clear_stats(void);
static void on_pdm_samples_ready(void);
#if ENABLE_WIFI_STREAMING
static bool initialise_wifi(void);
static bool wifi_queue_push(const pdm_block_packet_t* packet, size_t length);
static void wifi_queue_service(void);
static void wifi_queue_clear(void);
static void wifi_tcp_close(void);
static bool wifi_tcp_ready(void);
static err_t wifi_tcp_connected_cb(void* arg, struct tcp_pcb* tpcb, err_t err);
static void wifi_tcp_err_cb(void* arg, err_t err);
#endif
static void initialise_uart_stream(void);
static bool serial_stream_block(const uint8_t* data, size_t length);
static void initialise_spi_stream(void);
static bool initialise_trigger_wave_generator(void);

static inline void capture_slot_release(capture_slot_t* slot) {
    if (!slot) {
        return;
    }
    capture_slot_release_dma(slot);
    slot->buffer_handle = 0;
    slot->length = 0;
    slot->state = CAPTURE_SLOT_FREE;
    slot->pending_outputs = 0;
    slot->has_copy = false;
}

static inline void capture_slot_output_done(capture_slot_t* slot) {
    if (!slot) {
        return;
    }
    if (slot->pending_outputs > 0) {
        slot->pending_outputs--;
    }
    if (slot->pending_outputs == 0) {
        capture_slot_release(slot);
    }
}

// Clear producer/consumer flags so the next DMA callback restarts cleanly.
static void reset_capture_state(void) {
    uint32_t status = save_and_disable_interrupts();
    capture_write_index = 0;
    capture_read_index = 0;
    capture_count = 0;
    for (int i = 0; i < CAPTURE_RING_DEPTH; i++) {
        capture_slot_release(&capture_ring[i]);
    }
    restore_interrupts(status);

    clear_stats();
    last_debug_ms = to_ms_since_boot(get_absolute_time());
#if ENABLE_WIFI_STREAMING
    wifi_queue_clear();
#endif
    if (spi_initialised) {
        spi_streamer_reset(&spi_streamer_ctx);
    }
}

// Reconfigure the microphone library at runtime while preserving state trackers.
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
        if (spi_initialised) {
            spi_streamer_reset(&spi_streamer_ctx);
            spi_streamer_set_bit_index(&spi_streamer_ctx, 0);
        }
    }

    reset_capture_state();

    if (success) {
        printf("PDM clock running at %lu Hz\n", (unsigned long)config.sample_rate);
    }

    return success;
}

#if ENABLE_WIFI_STREAMING
static bool initialise_wifi(void) {
    if (wifi_initialised) {
        return wifi_ready;
    }

    wifi_initialised = true;
    bool success = false;

    if (cyw43_arch_init()) {
        printf("CYW43 init failed\n");
        goto cleanup;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi \"%s\"...\n", WIFI_SSID);
    int status = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, WIFI_AUTH_TYPE, WIFI_CONNECT_TIMEOUT_MS);
    if (status != 0) {
        printf("Wi-Fi connection failed (status=%d)\n", status);
        goto cleanup;
    }

    if (!ipaddr_aton(WIFI_TARGET_IP, &wifi_remote_addr)) {
        printf("Invalid WIFI_TARGET_IP \"%s\"\n", WIFI_TARGET_IP);
        goto cleanup;
    }

    if (wifi_transport == WIFI_TRANSPORT_UDP) {
        cyw43_arch_lwip_begin();
        wifi_udp_socket = udp_new_ip_type(IPADDR_TYPE_ANY);
        if (wifi_udp_socket == NULL) {
            cyw43_arch_lwip_end();
            printf("Failed to allocate UDP socket\n");
            goto cleanup;
        }
        err_t bind_err = udp_bind(wifi_udp_socket, IP_ANY_TYPE, 0);
        cyw43_arch_lwip_end();
        if (bind_err != ERR_OK) {
            printf("UDP bind failed (err=%d)\n", bind_err);
            goto cleanup;
        }
    } else {
        wifi_tcp_pcb = NULL;
        wifi_tcp_connecting = NULL;
        wifi_tcp_retry_deadline = nil_time;
    }

    wifi_ready = true;
    success = true;
    printf("Wi-Fi ready. Streaming %s packets to %s:%d\n",
           wifi_transport == WIFI_TRANSPORT_TCP ? "TCP" : "UDP",
           WIFI_TARGET_IP,
           WIFI_TARGET_PORT);

cleanup:
    if (!success) {
        wifi_queue_clear();
        if (wifi_udp_socket != NULL) {
            cyw43_arch_lwip_begin();
            udp_remove(wifi_udp_socket);
            cyw43_arch_lwip_end();
            wifi_udp_socket = NULL;
        }
        cyw43_arch_deinit();
        wifi_initialised = false;
        wifi_ready = false;
    }

    return success;
}

#endif  // ENABLE_WIFI_STREAMING

static void on_pdm_samples_ready(void) {
    // Library callback from DMA IRQ context when a raw buffer has filled.
    uint8_t write_index = capture_write_index;
    capture_slot_t* slot = &capture_ring[write_index];
    bool slot_available = (slot->state == CAPTURE_SLOT_FREE);
    struct pdm_acquired_buffer buffer = {0};
    if (!pdm_microphone_acquire_buffer(&buffer)) {
        return;
    }

    if (slot_available) {
        slot->packet.metadata = *buffer.metadata;
        slot->length = (uint16_t)buffer.length;
        slot->dma_payload = buffer.data;
        slot->buffer_handle = buffer.buffer_index;
        slot->has_copy = false;
        slot->state = CAPTURE_SLOT_READY;
        slot->pending_outputs = 0;
        capture_write_index = (uint8_t)((capture_write_index + 1) % CAPTURE_RING_DEPTH);
        capture_count++;
        blocks_ready++;
    } else {
        pdm_microphone_release_buffer(buffer.buffer_index);
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
    printf("  4   - set PDM clock to 3.000 MHz\n");
    printf("  5   - set PDM clock to 4.000 MHz\n");
    printf("  6   - set PDM clock to 4.800 MHz\n");
    printf("  r   - reinitialize microphone with current settings\n\n");
#if ENABLE_WIFI_STREAMING
    printf("  w   - toggle Wi-Fi streaming (currently %s)\n", wifi_stream_enabled ? "ON" : "OFF");
#endif
    printf("  t   - toggle UART streaming (currently %s)\n", serial_stream_enabled ? "ON" : "OFF");
    printf("  p   - toggle SPI streaming (currently %s)\n", spi_stream_enabled ? "ON" : "OFF");
    printf("  n   - print transport status information\n\n");
}

static void print_stats(void) {
    const char* serial_state = serial_stream_enabled
        ? (serial_initialised ? "ON" : "init")
        : "OFF";

    const char* spi_state = spi_stream_enabled
        ? (spi_initialised ? "ON" : "init")
        : "OFF";
    struct spi_streamer_stats spi_stats = {0};
    if (spi_initialised) {
        spi_stats = spi_streamer_get_stats(&spi_streamer_ctx);
    }

    printf("Stats: ready=%lu streamed=%lu discarded=%lu overruns=%lu bytes=%lu stream=%s dump=%s pdm_clk=%luHz ",
           (unsigned long)blocks_ready,
           (unsigned long)blocks_streamed,
           (unsigned long)blocks_discarded,
           (unsigned long)overruns,
           (unsigned long)bytes_sent,
           stream_binary_data ? "ON" : "OFF",
           debug_dump_samples ? "ON" : "OFF",
           (unsigned long)config.sample_rate);
#if ENABLE_WIFI_STREAMING
    const char* wifi_state = wifi_ready
        ? (wifi_stream_enabled ? (wifi_transport == WIFI_TRANSPORT_TCP ? "tcp" : "udp") : "paused")
        : "OFF";
    printf("wifi=%s pkt=%lu bytes=%lu err=%lu alloc_fail=%lu drop=%lu ",
           wifi_state,
           (unsigned long)wifi_packets_sent,
           (unsigned long)wifi_bytes_sent,
           (unsigned long)wifi_send_failures,
           (unsigned long)wifi_alloc_failures,
           (unsigned long)wifi_queue_drops);
#else
    printf("wifi=DISABLED ");
#endif
    uint32_t spi_task_avg_us = spi_stats.task_interval_samples
        ? (uint32_t)(spi_stats.task_interval_total_us / spi_stats.task_interval_samples)
        : 0u;

    printf("spi=%s built=%lu sent=%lu drop=%lu overflow=%lu underrun=%lu last_bit=%llu bytes=%llu "
           "task_last=%luus task_avg=%luus "
           "uart=%s pkt=%lu bytes=%lu err=%lu\n",
           spi_state,
           (unsigned long)spi_stats.frames_built,
           (unsigned long)spi_stats.frames_sent,
           (unsigned long)spi_stats.frames_discarded,
           (unsigned long)spi_stats.queue_overflows,
           (unsigned long)spi_stats.underruns,
           (unsigned long long)spi_stats.last_frame_start_bit,
           (unsigned long long)spi_stats.bytes_transmitted,
           (unsigned long)spi_stats.task_interval_last_us,
           (unsigned long)spi_task_avg_us,
           serial_state,
           (unsigned long)serial_packets_sent,
           (unsigned long)serial_bytes_sent,
           (unsigned long)serial_failures);
}

static void print_transport_status(void) {
#if ENABLE_WIFI_STREAMING
    printf("Wi-Fi streaming %s via %s\n",
           wifi_stream_enabled ? "ENABLED" : "DISABLED",
           wifi_transport == WIFI_TRANSPORT_TCP ? "TCP" : "UDP");
    printf("  initialised: %s\n", wifi_initialised ? "YES" : "NO");
    printf("  ready:       %s\n", wifi_ready ? "YES" : "NO");
    printf("  target:      %s:%d\n", WIFI_TARGET_IP, WIFI_TARGET_PORT);
    printf("  packets:     %lu\n", (unsigned long)wifi_packets_sent);
    printf("  bytes:       %lu\n", (unsigned long)wifi_bytes_sent);
    printf("  send error:  %lu\n", (unsigned long)wifi_send_failures);
    printf("  alloc fail:  %lu\n", (unsigned long)wifi_alloc_failures);
    printf("  queue depth: %u / %u\n", wifi_queue_count, WIFI_QUEUE_CAPACITY);
    printf("  queue drops: %lu\n", (unsigned long)wifi_queue_drops);
#else
    printf("Wi-Fi streaming DISABLED (compile-time)\n");
#endif
    struct spi_streamer_stats spi_stats = {0};
    if (spi_initialised) {
        spi_stats = spi_streamer_get_stats(&spi_streamer_ctx);
    }
    printf("SPI streaming %s\n", spi_stream_enabled ? "ENABLED" : "DISABLED");
    printf("  initialised: %s\n", spi_initialised ? "YES" : "NO");
    printf("  backend:     PIO%d SM%d (Mode 3, payload=%u bytes)\n",
           SPI_STREAM_PIO == pio0 ? 0 : 1,
           SPI_STREAM_STATE_MACHINE,
           SPI_STREAM_PAYLOAD_BYTES);
    printf("  pins:        SCK=GP%d TX=GP%d RX=GP%d CS=GP%d\n",
           SPI_STREAM_SCK_PIN,
           SPI_STREAM_TX_PIN,
           SPI_STREAM_RX_PIN,
           SPI_STREAM_CS_PIN);
    printf("  frames:      %lu built / %lu sent / %lu dropped\n",
           (unsigned long)spi_stats.frames_built,
           (unsigned long)spi_stats.frames_sent,
           (unsigned long)spi_stats.frames_discarded);
    printf("  queue:       %lu overflow events\n",
           (unsigned long)spi_stats.queue_overflows);
    printf("  bytes:       %llu enqueued / %llu transmitted\n",
           (unsigned long long)spi_stats.bytes_enqueued,
           (unsigned long long)spi_stats.bytes_transmitted);
    printf("  underruns:   %lu (failures=%lu)\n",
           (unsigned long)spi_stats.underruns,
           (unsigned long)spi_failures);
    uint32_t spi_task_avg_us = spi_stats.task_interval_samples
        ? (uint32_t)(spi_stats.task_interval_total_us / spi_stats.task_interval_samples)
        : 0u;
    printf("  task loop:   last=%lu us avg=%lu us (%lu samples)\n",
           (unsigned long)spi_stats.task_interval_last_us,
           (unsigned long)spi_task_avg_us,
           (unsigned long)spi_stats.task_interval_samples);
    printf("  last frame:  start_bit=%llu\n",
           (unsigned long long)spi_stats.last_frame_start_bit);

    printf("UART streaming %s\n", serial_stream_enabled ? "ENABLED" : "DISABLED");
    printf("  initialised: %s\n", serial_initialised ? "YES" : "NO");
    printf("  packets:     %lu\n", (unsigned long)serial_packets_sent);
    printf("  bytes:       %lu\n", (unsigned long)serial_bytes_sent);
    printf("  failures:    %lu\n", (unsigned long)serial_failures);
}

static void clear_stats(void) {
    uint32_t status = save_and_disable_interrupts();
    blocks_ready = 0;
    blocks_discarded = 0;
    overruns = 0;
    blocks_streamed = 0;
    bytes_sent = 0;
    restore_interrupts(status);

#if ENABLE_WIFI_STREAMING
    wifi_packets_sent = 0;
    wifi_bytes_sent = 0;
    wifi_send_failures = 0;
    wifi_alloc_failures = 0;
    wifi_queue_drops = 0;
#endif

    serial_packets_sent = 0;
    serial_bytes_sent = 0;
    serial_failures = 0;
    spi_failures = 0;
    if (spi_initialised) {
        spi_streamer_clear_stats(&spi_streamer_ctx);
    }
}

// Poll the USB CDC RX path for single-character debug commands.
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
#if ENABLE_WIFI_STREAMING
            case 'w':
                wifi_stream_enabled = !wifi_stream_enabled;
                printf("Wi-Fi streaming %s\n", wifi_stream_enabled ? "ENABLED" : "DISABLED");
                if (!wifi_stream_enabled) {
                    wifi_queue_clear();
                } else if (!wifi_ready) {
                    if (!initialise_wifi()) {
                        printf("Wi-Fi init failed; disabling streaming\n");
                        wifi_stream_enabled = false;
                    }
                }
                break;
#endif
            case 't':
                serial_stream_enabled = !serial_stream_enabled;
                if (serial_stream_enabled && !serial_initialised) {
                    initialise_uart_stream();
                }
                printf("UART streaming %s (TX=GP%d, %u baud)\n",
                       serial_stream_enabled ? "ENABLED" : "DISABLED",
                       SERIAL_UART_TX_PIN,
                       (unsigned int)SERIAL_UART_BAUD);
                break;
            case 'p':
                spi_stream_enabled = !spi_stream_enabled;
                if (spi_stream_enabled) {
                    if (!spi_initialised) {
                        initialise_spi_stream();
                    } else {
                        spi_streamer_set_enabled(&spi_streamer_ctx, true);
                    }
                } else if (spi_initialised) {
                    spi_streamer_set_enabled(&spi_streamer_ctx, false);
                }
                printf("SPI streaming %s (PIO%d SM%d, SCK=GP%d TX=GP%d RX=GP%d CS=GP%d)\n",
                       spi_stream_enabled ? "ENABLED" : "DISABLED",
                       SPI_STREAM_PIO == pio0 ? 0 : 1,
                       SPI_STREAM_STATE_MACHINE,
                       SPI_STREAM_SCK_PIN,
                       SPI_STREAM_TX_PIN,
                       SPI_STREAM_RX_PIN,
                       SPI_STREAM_CS_PIN);
                break;
            case 'n':
                print_transport_status();
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
            case '4':
                if (config.sample_rate != 3000000) {
                    restart_microphone(3000000);
                } else {
                    printf("PDM clock already at 3.000 MHz\n");
                }
                break;
            case '5':
                if (config.sample_rate != 4000000) {
                    restart_microphone(4000000);
                } else {
                    printf("PDM clock already at 4.000 MHz\n");
                }
                break;
            case '6':
                if (config.sample_rate != 4800000) {
                    restart_microphone(4800000);
                } else {
                    printf("PDM clock already at 4.800 MHz\n");
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

static void dump_block_summary(const pdm_block_packet_t* packet, size_t length) {
    const size_t max_preview = 32;
    size_t preview = length < max_preview ? length : max_preview;
    uint32_t ones = 0;
    uint32_t zeros = 0;
    const uint8_t* data = packet->payload;
    const struct pdm_block_metadata* meta = &packet->metadata;

    printf("Block %llu len=%lu t_end=%llu us trig_idx=%d tail=%s:",
           (unsigned long long)meta->block_index,
           (unsigned long)length,
           (unsigned long long)meta->capture_end_time_us,
           (int)meta->trigger_first_index,
           meta->trigger_tail_high ? "HIGH" : "LOW");
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

static void initialise_uart_stream(void) {
    if (serial_initialised) {
        return;
    }

    uart_init(SERIAL_UART_ID, SERIAL_UART_BAUD);
    gpio_set_function(SERIAL_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(SERIAL_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(SERIAL_UART_ID, false, false);
    uart_set_format(SERIAL_UART_ID, 8, 1, UART_PARITY_NONE);
    serial_initialised = true;
}

#if ENABLE_WIFI_STREAMING
static void wifi_tcp_close(void) {
    cyw43_arch_lwip_begin();
    if (wifi_tcp_pcb) {
        tcp_arg(wifi_tcp_pcb, NULL);
        tcp_err(wifi_tcp_pcb, NULL);
        tcp_sent(wifi_tcp_pcb, NULL);
        tcp_poll(wifi_tcp_pcb, NULL, 0);
        if (tcp_close(wifi_tcp_pcb) != ERR_OK) {
            tcp_abort(wifi_tcp_pcb);
        }
        wifi_tcp_pcb = NULL;
    }
    if (wifi_tcp_connecting) {
        tcp_abort(wifi_tcp_connecting);
        wifi_tcp_connecting = NULL;
    }
    cyw43_arch_lwip_end();
    wifi_tcp_retry_deadline = make_timeout_time_ms(WIFI_TCP_RETRY_MS);
}

static void wifi_queue_clear(void) {
    wifi_queue_head = 0;
    wifi_queue_tail = 0;
    wifi_queue_count = 0;
    if (wifi_transport == WIFI_TRANSPORT_TCP) {
        wifi_tcp_close();
    }
}

static bool wifi_tcp_ready(void) {
    if (wifi_transport != WIFI_TRANSPORT_TCP) {
        return false;
    }
    if (wifi_tcp_pcb) {
        return true;
    }
    if (wifi_tcp_connecting) {
        return false;
    }
    if (!wifi_ready) {
        return false;
    }
    if (!is_nil_time(wifi_tcp_retry_deadline) && !time_reached(wifi_tcp_retry_deadline)) {
        return false;
    }

    cyw43_arch_lwip_begin();
    struct tcp_pcb* pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (!pcb) {
        cyw43_arch_lwip_end();
        wifi_tcp_retry_deadline = make_timeout_time_ms(WIFI_TCP_RETRY_MS);
        return false;
    }

    tcp_arg(pcb, NULL);
    tcp_err(pcb, wifi_tcp_err_cb);
    err_t err = tcp_connect(pcb, ip_2_ip4(&wifi_remote_addr), WIFI_TARGET_PORT, wifi_tcp_connected_cb);
    cyw43_arch_lwip_end();
    if (err != ERR_OK) {
        cyw43_arch_lwip_begin();
        tcp_abort(pcb);
        cyw43_arch_lwip_end();
        wifi_tcp_retry_deadline = make_timeout_time_ms(WIFI_TCP_RETRY_MS);
        return false;
    }

    wifi_tcp_connecting = pcb;
    return false;
}

static err_t wifi_tcp_connected_cb(void* arg, struct tcp_pcb* tpcb, err_t err) {
    (void)arg;
    if (err != ERR_OK) {
        tcp_abort(tpcb);
        wifi_tcp_connecting = NULL;
        wifi_tcp_retry_deadline = make_timeout_time_ms(WIFI_TCP_RETRY_MS);
        return err;
    }
    wifi_tcp_pcb = tpcb;
    wifi_tcp_connecting = NULL;
    return ERR_OK;
}

static void wifi_tcp_err_cb(void* arg, err_t err) {
    (void)arg;
    (void)err;
    wifi_tcp_pcb = NULL;
    wifi_tcp_connecting = NULL;
    wifi_tcp_retry_deadline = make_timeout_time_ms(WIFI_TCP_RETRY_MS);
}

static bool wifi_queue_push(const pdm_block_packet_t* packet, size_t length) {
    if (!wifi_stream_enabled || !wifi_ready || length == 0 || packet == NULL) {
        return false;
    }

    if (length > sizeof(wifi_queue[0].packet)) {
        return false;
    }

    if (wifi_queue_count >= WIFI_QUEUE_CAPACITY) {
        wifi_queue_drops++;
        return false;
    }

    wifi_packet_t* slot = &wifi_queue[wifi_queue_tail];
    memcpy(&slot->packet, packet, length);
    slot->length = (uint16_t)length;
    slot->sent = 0;

    wifi_queue_tail = (uint8_t)((wifi_queue_tail + 1) % WIFI_QUEUE_CAPACITY);
    wifi_queue_count++;
    return true;
}

static void wifi_queue_service(void) {
    if (!wifi_ready || !wifi_stream_enabled || wifi_queue_count == 0) {
        return;
    }

    if (wifi_transport == WIFI_TRANSPORT_UDP) {
        if (wifi_udp_socket == NULL) {
            return;
        }
        while (wifi_queue_count > 0) {
            wifi_packet_t* pkt = &wifi_queue[wifi_queue_head];
            cyw43_arch_lwip_begin();
            struct pbuf* buffer = pbuf_alloc(PBUF_TRANSPORT, pkt->length, PBUF_RAM);
            if (buffer == NULL) {
                cyw43_arch_lwip_end();
                wifi_alloc_failures++;
                break;
            }
            memcpy(buffer->payload, &pkt->packet, pkt->length);
            err_t err = udp_sendto(wifi_udp_socket, buffer, &wifi_remote_addr, WIFI_TARGET_PORT);
            pbuf_free(buffer);
            cyw43_arch_lwip_end();
            if (err != ERR_OK) {
                wifi_send_failures++;
                break;
            }
            wifi_packets_sent++;
            wifi_bytes_sent += pkt->length;
            wifi_queue_head = (uint8_t)((wifi_queue_head + 1) % WIFI_QUEUE_CAPACITY);
            wifi_queue_count--;
        }
        return;
    }

    if (!wifi_tcp_ready()) {
        return;
    }

    while (wifi_queue_count > 0) {
        wifi_packet_t* pkt = &wifi_queue[wifi_queue_head];
        uint16_t remaining = (uint16_t)(pkt->length - pkt->sent);
        if (remaining == 0) {
            pkt->sent = 0;
            wifi_queue_head = (uint8_t)((wifi_queue_head + 1) % WIFI_QUEUE_CAPACITY);
            wifi_queue_count--;
            wifi_packets_sent++;
            wifi_bytes_sent += pkt->length;
            continue;
        }

        cyw43_arch_lwip_begin();
        u16_t avail = wifi_tcp_pcb ? tcp_sndbuf(wifi_tcp_pcb) : 0;
        cyw43_arch_lwip_end();
        if (avail == 0) {
            break;
        }

        // lwIP's TCP send buffer (tcp_sndbuf) can be smaller than a full block, so stream each packet in chunks.
        uint16_t chunk = remaining;
        if (chunk > avail) {
            chunk = (uint16_t)avail;
        }

        uint8_t* base = (uint8_t*)&pkt->packet;
        u8_t flags = TCP_WRITE_FLAG_COPY;
        if (chunk < remaining) {
            flags |= TCP_WRITE_FLAG_MORE;
        }

        cyw43_arch_lwip_begin();
        err_t err = tcp_write(wifi_tcp_pcb, base + pkt->sent, chunk, flags);
        if (err == ERR_OK) {
            err = tcp_output(wifi_tcp_pcb);
        }
        cyw43_arch_lwip_end();

        if (err != ERR_OK) {
            wifi_send_failures++;
            wifi_tcp_close();
            break;
        }

        pkt->sent = (uint16_t)(pkt->sent + chunk);
        if (pkt->sent >= pkt->length) {
            pkt->sent = 0;
            wifi_queue_head = (uint8_t)((wifi_queue_head + 1) % WIFI_QUEUE_CAPACITY);
            wifi_queue_count--;
            wifi_packets_sent++;
            wifi_bytes_sent += pkt->length;
        }
    }
}

#endif  // ENABLE_WIFI_STREAMING

static bool serial_stream_block(const uint8_t* data, size_t length) {
    if (!serial_stream_enabled || length == 0) {
        return false;
    }

    if (!serial_initialised) {
        initialise_uart_stream();
    }

    size_t offset = 0;
    uint32_t spin_loops = 0;

    while (offset < length) {
        if (uart_is_writable(SERIAL_UART_ID)) {
            uart_putc_raw(SERIAL_UART_ID, data[offset]);
            offset++;
            spin_loops = 0;
            continue;
        }

        if (++spin_loops > SERIAL_WRITE_SPIN_MAX) {
            return false;
        }

        sleep_us(SERIAL_WRITE_BACKOFF_US);
    }

    serial_packets_sent++;
    serial_bytes_sent += (uint32_t)length;
    return true;
}

static void initialise_spi_stream(void) {
    if (spi_initialised) {
        return;
    }
    struct spi_streamer_config stream_cfg = {
        .pio = SPI_STREAM_PIO,
        .sm = SPI_STREAM_STATE_MACHINE,
        .gpio_sck = SPI_STREAM_SCK_PIN,
        .gpio_mosi = SPI_STREAM_RX_PIN,
        .gpio_miso = SPI_STREAM_TX_PIN,
        .gpio_cs = SPI_STREAM_CS_PIN,
        .payload_bytes = SPI_STREAM_PAYLOAD_BYTES,
        .channel_count = config.channels,
        .channel_mask = 0,
    };

    if (!spi_streamer_init(&spi_streamer_ctx, &stream_cfg)) {
        printf("Failed to initialise SPI streamer\n");
        return;
    }

    spi_streamer_set_enabled(&spi_streamer_ctx, spi_stream_enabled);
    spi_initialised = true;
}

static bool initialise_trigger_wave_generator(void) {
    const uint32_t frequency_hz = TRIGGER_WAVE_FREQUENCY_HZ;
    const uint32_t duration_us = TRIGGER_WAVE_DURATION_US;

    if (frequency_hz == 0 || duration_us == 0) {
        printf("Trigger wave generator disabled (freq=%lu duration=%lu)\n",
               (unsigned long)frequency_hz,
               (unsigned long)duration_us);
        return false;
    }

    uint64_t cycles64 = ((uint64_t)frequency_hz * duration_us) / 1000000u;
    if (cycles64 == 0) {
        cycles64 = 1;
    }
    uint32_t cycles = (uint32_t)cycles64;
    uint32_t iterations = (cycles > 0) ? (cycles - 1u) : 0u;

    PIO pio = TRIGGER_WAVE_PIO;
    uint sm = TRIGGER_WAVE_STATE_MACHINE;

    if (!pio_can_add_program(pio, &trigger_wave_program)) {
        printf("Trigger wave generator: insufficient instruction memory\n");
        return false;
    }

    uint program_offset = pio_add_program(pio, &trigger_wave_program);
    float clk_div = (float)clock_get_hz(clk_sys) / (2.0f * (float)frequency_hz);
    if (clk_div < 1.0f) {
        clk_div = 1.0f;
    }

    pio_sm_set_enabled(pio, sm, false);
    trigger_wave_program_init(
        pio,
        sm,
        program_offset,
        clk_div,
        TRIGGER_WAVE_TRIGGER_PIN,
        TRIGGER_WAVE_OUTPUT_PIN
    );
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_put_blocking(pio, sm, iterations);
    pio_sm_set_enabled(pio, sm, true);

    printf("Trigger wave generator ready: trigger=GP%d output=GP%d freq=%luHz duration=%luus cycles=%lu clkdiv=%.2f\n",
           TRIGGER_WAVE_TRIGGER_PIN,
           TRIGGER_WAVE_OUTPUT_PIN,
           (unsigned long)frequency_hz,
           (unsigned long)duration_us,
           (unsigned long)cycles,
           clk_div);

    return true;
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
            // Back-pressure until the host drains its buffers.
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

#if ENABLE_WIFI_STREAMING
    bool wifi_started = initialise_wifi();
    if (!wifi_started) {
        printf("Wi-Fi not available; streaming disabled.\n");
        wifi_stream_enabled = false;
    }
#else
    wifi_stream_enabled = false;
#endif

    bool usb_connected = false;
    if (USB_WAIT_TIMEOUT_MS > 0) {
        absolute_time_t usb_wait_deadline = make_timeout_time_ms(USB_WAIT_TIMEOUT_MS);
        while (!tud_cdc_connected()) {
            tud_task();
            if (time_reached(usb_wait_deadline)) {
                break;
            }
            tight_loop_contents();
        }
    } else {
        while (!tud_cdc_connected()) {
            tud_task();
            tight_loop_contents();
        }
    }

    usb_connected = tud_cdc_connected();
    if (usb_connected) {
        printf("USB CDC connected\n");
    } else {
        printf("Proceeding without USB CDC connection\n");
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
    initialise_trigger_wave_generator();

    while (true) {
        tud_task();
        handle_console_input();
#if ENABLE_WIFI_STREAMING
        wifi_queue_service();
#endif
        if (spi_initialised) {
            spi_streamer_task(&spi_streamer_ctx);
        }

        if (capture_count == 0) {
            tight_loop_contents();
            continue;
        }

        capture_slot_t* slot = NULL;
        pdm_block_packet_t* packet = NULL;
        int bytes_to_send = 0;
        size_t packet_bytes = 0;

        uint32_t status = save_and_disable_interrupts();
        if (capture_count > 0) {
            capture_slot_t* candidate = &capture_ring[capture_read_index];
            if (candidate->state == CAPTURE_SLOT_READY && candidate->length > 0) {
                slot = candidate;
                slot->state = CAPTURE_SLOT_SENDING;
                bytes_to_send = slot->length;
                packet = &slot->packet;
                capture_read_index = (uint8_t)((capture_read_index + 1) % CAPTURE_RING_DEPTH);
                capture_count--;
                uint8_t pending = 0;
#if ENABLE_WIFI_STREAMING
                if (wifi_stream_enabled) {
                    pending++;
                }
#endif
                if (spi_stream_enabled) {
                    pending++;
                }
                if (serial_stream_enabled) {
                    pending++;
                }
                if (stream_binary_data) {
                    pending++;
                }
                slot->pending_outputs = pending;
            }
        }
        restore_interrupts(status);

        if (!slot || bytes_to_send <= 0) {
            overruns++;
            continue;
        }

        if (debug_dump_samples) {
            dump_block_summary(packet, (size_t)bytes_to_send);
        }

        packet_bytes = sizeof(slot->packet.metadata) + (size_t)bytes_to_send;
        const uint8_t* packet_data = (const uint8_t*)&slot->packet;

        if (spi_stream_enabled && slot) {
            if (!spi_initialised) {
                initialise_spi_stream();
            }
            const uint8_t* spi_data = capture_slot_payload_ptr(slot);
            if (!spi_data) {
                capture_slot_ensure_copy(slot);
                spi_data = capture_slot_payload_ptr(slot);
            }
            if (!spi_initialised || !spi_streamer_push_block(&spi_streamer_ctx, spi_data, (size_t)bytes_to_send)) {
                spi_failures++;
            }
            if (slot->pending_outputs == 1 && slot->dma_payload) {
                capture_slot_release_dma(slot);
            }
            capture_slot_output_done(slot);
            if (slot && slot->state == CAPTURE_SLOT_FREE) {
                slot = NULL;
            }
        }

#if ENABLE_WIFI_STREAMING
        if (wifi_stream_enabled && slot) {
            capture_slot_ensure_copy(slot);
            wifi_queue_push(&slot->packet, packet_bytes);
            wifi_queue_service();
            capture_slot_output_done(slot);
            if (slot && slot->state == CAPTURE_SLOT_FREE) {
                slot = NULL;
            }
        }
#endif

        if (serial_stream_enabled && slot) {
            capture_slot_ensure_copy(slot);
            if (!serial_stream_block(packet_data, packet_bytes)) {
                serial_failures++;
            }
            capture_slot_output_done(slot);
            if (slot && slot->state == CAPTURE_SLOT_FREE) {
                slot = NULL;
            }
        }

        if (stream_binary_data && slot) {
            capture_slot_ensure_copy(slot);
            usb_write_blocking(packet_data, packet_bytes);
            bytes_sent += (uint32_t)bytes_to_send;
            blocks_streamed++;
            capture_slot_output_done(slot);
            if (slot && slot->state == CAPTURE_SLOT_FREE) {
                slot = NULL;
            }
        }

        if (slot && slot->pending_outputs == 0) {
            capture_slot_release(slot);
            slot = NULL;
        }

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (debug_stats_enabled && (now_ms - last_debug_ms >= debug_interval_ms)) {
            print_stats();
            last_debug_ms = now_ms;
        }
    }
}
