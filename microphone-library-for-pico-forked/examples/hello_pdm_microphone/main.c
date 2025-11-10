#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/sync.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/cyw43_arch.h"
#include "pico/pdm_microphone.h"

#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include "tusb.h"

// Example entry point: capture PDM audio and stream over USB CDC and/or UDP.

#ifndef WIFI_SSID
// #define WIFI_SSID "Routers of Rohan"
#define WIFI_SSID "chack-2.4"
#endif

#ifndef WIFI_PASSWORD
// #define WIFI_PASSWORD "ridersoftheolan"
#define WIFI_PASSWORD "redblueyellowgreen"
#endif

#ifndef WIFI_AUTH_TYPE
#define WIFI_AUTH_TYPE CYW43_AUTH_WPA2_AES_PSK
#endif

#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 30000
#endif

#ifndef WIFI_TARGET_IP
// #define WIFI_TARGET_IP "192.168.8.209"
#define WIFI_TARGET_IP "192.168.1.190"
#endif

#ifndef WIFI_TARGET_PORT
#define WIFI_TARGET_PORT 6000
#endif

#ifndef USB_WAIT_TIMEOUT_MS
#define USB_WAIT_TIMEOUT_MS 3000
#endif

#define PDM_CHANNEL_COUNT       2
#define PDM_BLOCK_BITS_PER_CH   8192
#define PDM_BLOCK_BYTES_PER_CH  (PDM_BLOCK_BITS_PER_CH / 8)
#define PDM_BLOCK_PAYLOAD_BYTES (PDM_BLOCK_BYTES_PER_CH * PDM_CHANNEL_COUNT)
#define PDM_BLOCK_PACKET_BYTES  (PDM_BLOCK_PAYLOAD_BYTES + sizeof(struct pdm_block_metadata))

#define SERIAL_UART_ID         uart0
#define SERIAL_UART_TX_PIN     0
#define SERIAL_UART_RX_PIN     1
#define SERIAL_UART_BAUD       2000000
#define SERIAL_WRITE_SPIN_MAX  500000
#define SERIAL_WRITE_BACKOFF_US 5

#define USB_VENDOR_SPIN_MAX    20000
#define USB_VENDOR_BACKOFF_US  10

static uint8_t sample_buffer[PDM_BLOCK_PAYLOAD_BYTES];
static uint8_t discard_buffer[PDM_BLOCK_PAYLOAD_BYTES];
static struct pdm_block_metadata sample_metadata;
static struct pdm_block_metadata discard_metadata;
static volatile bool block_pending = false;
static volatile int block_length = 0;

// Debug/telemetry counters surfaced through the CLI stats.
static volatile uint32_t blocks_ready = 0;
static volatile uint32_t blocks_discarded = 0;
static volatile uint32_t overruns = 0;

static uint32_t blocks_streamed = 0;
static uint32_t bytes_sent = 0;

static bool debug_stats_enabled = true;
static bool debug_dump_samples = false;
static bool stream_binary_data = false;
static bool usb_vendor_stream_enabled = false;
static bool serial_stream_enabled = false;

static uint32_t debug_interval_ms = 1000;
static uint32_t last_debug_ms = 0;
static bool mic_running = false;

static bool wifi_initialised = false;
static bool wifi_ready = false;
static bool wifi_stream_enabled = true;
static struct udp_pcb* wifi_udp_socket = NULL;
static ip_addr_t wifi_remote_addr;

#define WIFI_QUEUE_CAPACITY 128

typedef struct {
    struct pdm_block_metadata metadata;
    uint8_t payload[PDM_BLOCK_PAYLOAD_BYTES];
} pdm_block_packet_t;

typedef struct {
    uint8_t data[PDM_BLOCK_PACKET_BYTES];
    uint16_t length;
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

static uint32_t usb_vendor_packets_sent = 0;
static uint32_t usb_vendor_bytes_sent = 0;
static uint32_t usb_vendor_backpressure_events = 0;
static uint32_t usb_vendor_failures = 0;

static uint32_t serial_packets_sent = 0;
static uint32_t serial_bytes_sent = 0;
static uint32_t serial_failures = 0;

static bool serial_initialised = false;

static struct pdm_microphone_config config = {
    .gpio_data = 2,
    .gpio_data_secondary = 3,
    .gpio_clk = 4,
    .pio = pio0,
    .pio_sm = 0,
    .sample_rate = 2048000,
    .sample_buffer_size = PDM_BLOCK_BYTES_PER_CH,
    .channels = PDM_CHANNEL_COUNT,
};

static void clear_stats(void);
static void on_pdm_samples_ready(void);
static bool initialise_wifi(void);
static bool wifi_queue_push(const uint8_t* data, size_t length);
static void wifi_queue_service(void);
static void wifi_queue_clear(void);
static void initialise_uart_stream(void);
static bool usb_vendor_stream_block(const uint8_t* data, size_t length);
static bool serial_stream_block(const uint8_t* data, size_t length);

// Clear producer/consumer flags so the next DMA callback restarts cleanly.
static void reset_capture_state(void) {
    uint32_t status = save_and_disable_interrupts();
    block_pending = false;
    block_length = 0;
    restore_interrupts(status);

    clear_stats();
    last_debug_ms = to_ms_since_boot(get_absolute_time());
    wifi_queue_clear();
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
    }

    reset_capture_state();

    if (success) {
        printf("PDM clock running at %lu Hz\n", (unsigned long)config.sample_rate);
    }

    return success;
}

static bool initialise_wifi(void) {
    if (wifi_initialised) {
        return wifi_ready;
    }

    // Perform a one-time bring-up of the CYW43 stack and UDP socket.
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

    cyw43_arch_lwip_begin();
    wifi_udp_socket = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (wifi_udp_socket == NULL) {
        cyw43_arch_lwip_end();
        printf("Failed to allocate UDP socket\n");
        goto cleanup;
    }

    // Bind to an ephemeral local port; destination is set per-packet.
    err_t bind_err = udp_bind(wifi_udp_socket, IP_ANY_TYPE, 0);
    cyw43_arch_lwip_end();

    if (bind_err != ERR_OK) {
        printf("UDP bind failed (err=%d)\n", bind_err);
        goto cleanup;
    }

    wifi_ready = true;
    success = true;
    printf("Wi-Fi ready. Streaming UDP blocks to %s:%d\n", WIFI_TARGET_IP, WIFI_TARGET_PORT);

cleanup:
    if (!success) {
        if (wifi_udp_socket != NULL) {
            // Tear down any partially initialised lwIP state before retrying.
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

static void on_pdm_samples_ready(void) {
    // Library callback from DMA IRQ context when a raw buffer has filled.
    uint8_t* target = block_pending ? discard_buffer : sample_buffer;
    struct pdm_block_metadata* target_metadata = block_pending ? &discard_metadata : &sample_metadata;
    int read = pdm_microphone_read_with_metadata(target, PDM_BLOCK_PAYLOAD_BYTES, target_metadata);

    if (!block_pending && read > 0) {
        blocks_ready++;
        block_length = read;
        block_pending = true;
    } else if (block_pending && read > 0) {
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
    printf("  w   - toggle Wi-Fi streaming (currently %s)\n", wifi_stream_enabled ? "ON" : "OFF");
    printf("  u   - toggle USB vendor streaming (currently %s)\n", usb_vendor_stream_enabled ? "ON" : "OFF");
    printf("  t   - toggle UART streaming (currently %s)\n", serial_stream_enabled ? "ON" : "OFF");
    printf("  n   - print transport status information\n\n");
}

static void print_stats(void) {
    const char* wifi_state = wifi_ready
        ? (wifi_stream_enabled ? "ON" : "paused")
        : (wifi_initialised ? "init-failed" : "not-started");

    const char* usb_state = usb_vendor_stream_enabled
        ? (tud_vendor_mounted() ? "ON" : "waiting")
        : "OFF";

    const char* serial_state = serial_stream_enabled
        ? (serial_initialised ? "ON" : "init")
        : "OFF";

    printf("Stats: ready=%lu streamed=%lu discarded=%lu overruns=%lu bytes=%lu stream=%s dump=%s pdm_clk=%luHz "
           "wifi=%s pkt=%lu bytes=%lu err=%lu alloc_fail=%lu queue=%u drop=%lu "
           "usb=%s pkt=%lu bytes=%lu err=%lu backp=%lu "
           "uart=%s pkt=%lu bytes=%lu err=%lu\n",
           (unsigned long)blocks_ready,
           (unsigned long)blocks_streamed,
           (unsigned long)blocks_discarded,
           (unsigned long)overruns,
           (unsigned long)bytes_sent,
           stream_binary_data ? "ON" : "OFF",
           debug_dump_samples ? "ON" : "OFF",
           (unsigned long)config.sample_rate,
           wifi_state,
           (unsigned long)wifi_packets_sent,
           (unsigned long)wifi_bytes_sent,
           (unsigned long)wifi_send_failures,
           (unsigned long)wifi_alloc_failures,
           wifi_queue_count,
           (unsigned long)wifi_queue_drops,
           usb_state,
           (unsigned long)usb_vendor_packets_sent,
           (unsigned long)usb_vendor_bytes_sent,
           (unsigned long)usb_vendor_failures,
           (unsigned long)usb_vendor_backpressure_events,
           serial_state,
           (unsigned long)serial_packets_sent,
           (unsigned long)serial_bytes_sent,
           (unsigned long)serial_failures);
}

static void print_transport_status(void) {
    // Dump transport counters without toggling streaming state.
    printf("Wi-Fi streaming %s\n", wifi_stream_enabled ? "ENABLED" : "DISABLED");
    printf("  initialised: %s\n", wifi_initialised ? "YES" : "NO");
    printf("  ready:       %s\n", wifi_ready ? "YES" : "NO");
    printf("  target:      %s:%d\n", WIFI_TARGET_IP, WIFI_TARGET_PORT);
    printf("  packets:     %lu\n", (unsigned long)wifi_packets_sent);
    printf("  bytes:       %lu\n", (unsigned long)wifi_bytes_sent);
    printf("  send error:  %lu\n", (unsigned long)wifi_send_failures);
    printf("  alloc fail:  %lu\n", (unsigned long)wifi_alloc_failures);
    printf("  queue depth: %u / %u\n", wifi_queue_count, WIFI_QUEUE_CAPACITY);
    printf("  queue drops: %lu\n", (unsigned long)wifi_queue_drops);

    printf("USB vendor streaming %s\n", usb_vendor_stream_enabled ? "ENABLED" : "DISABLED");
    printf("  mounted:     %s\n", tud_vendor_mounted() ? "YES" : "NO");
    printf("  packets:     %lu\n", (unsigned long)usb_vendor_packets_sent);
    printf("  bytes:       %lu\n", (unsigned long)usb_vendor_bytes_sent);
    printf("  backpressure:%lu\n", (unsigned long)usb_vendor_backpressure_events);
    printf("  failures:    %lu\n", (unsigned long)usb_vendor_failures);

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

    wifi_packets_sent = 0;
    wifi_bytes_sent = 0;
    wifi_send_failures = 0;
    wifi_alloc_failures = 0;
    wifi_queue_drops = 0;

    usb_vendor_packets_sent = 0;
    usb_vendor_bytes_sent = 0;
    usb_vendor_backpressure_events = 0;
    usb_vendor_failures = 0;

    serial_packets_sent = 0;
    serial_bytes_sent = 0;
    serial_failures = 0;
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
            case 'w':
                wifi_stream_enabled = !wifi_stream_enabled;
                printf("Wi-Fi streaming %s\n", wifi_stream_enabled ? "ENABLED" : "DISABLED");
                if (!wifi_stream_enabled) {
                    wifi_queue_clear();
                }
                break;
            case 'u':
                usb_vendor_stream_enabled = !usb_vendor_stream_enabled;
                printf("USB vendor streaming %s\n", usb_vendor_stream_enabled ? "ENABLED" : "DISABLED");
                break;
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

    printf("Block %llu len=%lu t_end=%llu us:",
           (unsigned long long)meta->block_index,
           (unsigned long)length,
           (unsigned long long)meta->capture_end_time_us);
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

static bool wifi_queue_push(const uint8_t* data, size_t length) {
    if (!wifi_stream_enabled || !wifi_ready || wifi_udp_socket == NULL || length == 0) {
        return false;
    }

    if (length > PDM_BLOCK_PACKET_BYTES) {
        return false;
    }

    if (wifi_queue_count >= WIFI_QUEUE_CAPACITY) {
        wifi_queue_drops++;
        return false;
    }

    wifi_packet_t* slot = &wifi_queue[wifi_queue_tail];
    memcpy(slot->data, data, length);
    slot->length = (uint16_t)length;

    wifi_queue_tail = (uint8_t)((wifi_queue_tail + 1) % WIFI_QUEUE_CAPACITY);
    wifi_queue_count++;
    return true;
}

static void wifi_queue_service(void) {
    if (!wifi_ready || !wifi_stream_enabled || wifi_udp_socket == NULL) {
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

        memcpy(buffer->payload, pkt->data, pkt->length);
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
}

static void wifi_queue_clear(void) {
    wifi_queue_head = 0;
    wifi_queue_tail = 0;
    wifi_queue_count = 0;
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

static bool usb_vendor_stream_block(const uint8_t* data, size_t length) {
    if (!usb_vendor_stream_enabled || length == 0 || !tud_vendor_mounted()) {
        return false;
    }

    size_t offset = 0;
    uint32_t spin_loops = 0;

    while (offset < length) {
        tud_task();

        if (!tud_vendor_mounted()) {
            return false;
        }

        uint32_t available = tud_vendor_write_available();
        if (available == 0) {
            if (++spin_loops > USB_VENDOR_SPIN_MAX) {
                return false;
            }
            usb_vendor_backpressure_events++;
            sleep_us(USB_VENDOR_BACKOFF_US);
            continue;
        }

        uint32_t chunk = (uint32_t)(length - offset);
        if (chunk > available) {
            chunk = available;
        }

        uint32_t written = tud_vendor_write(&data[offset], chunk);
        tud_vendor_flush();

        if (written == 0) {
            if (++spin_loops > USB_VENDOR_SPIN_MAX) {
                return false;
            }
            usb_vendor_backpressure_events++;
            sleep_us(USB_VENDOR_BACKOFF_US);
            continue;
        }

        offset += written;
        spin_loops = 0;
    }

    usb_vendor_packets_sent++;
    usb_vendor_bytes_sent += (uint32_t)length;
    return true;
}

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

    bool wifi_started = initialise_wifi();
    if (!wifi_started) {
        printf("Wi-Fi not available; UDP streaming disabled.\n");
    }

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

    while (true) {
        tud_task();
        handle_console_input();
        wifi_queue_service();

        if (!block_pending) {
            tight_loop_contents();
            continue;
        }

        pdm_block_packet_t transfer_packet;
        size_t packet_bytes = 0;
        int bytes_to_send = 0;

        uint32_t status = save_and_disable_interrupts();
        if (block_pending) {
            bytes_to_send = block_length;
            if (bytes_to_send > 0) {
                memcpy(transfer_packet.payload, sample_buffer, (size_t)bytes_to_send);
                transfer_packet.metadata = sample_metadata;
            }
            block_length = 0;
            block_pending = false;
        }
        restore_interrupts(status);

        if (bytes_to_send <= 0) {
            overruns++;
            continue;
        }

        packet_bytes = sizeof(transfer_packet.metadata) + (size_t)bytes_to_send;

        if (debug_dump_samples) {
            dump_block_summary(&transfer_packet, (size_t)bytes_to_send);
        }

        wifi_queue_push((const uint8_t*)&transfer_packet, packet_bytes);
        wifi_queue_service();

        if (usb_vendor_stream_enabled) {
            if (!usb_vendor_stream_block((const uint8_t*)&transfer_packet, packet_bytes)) {
                usb_vendor_failures++;
            }
        }

        if (serial_stream_enabled) {
            if (!serial_stream_block((const uint8_t*)&transfer_packet, packet_bytes)) {
                serial_failures++;
            }
        }

        if (stream_binary_data) {
            usb_write_blocking((const uint8_t*)&transfer_packet, packet_bytes);
            bytes_sent += (uint32_t)bytes_to_send;
            blocks_streamed++;
        }

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (debug_stats_enabled && (now_ms - last_debug_ms >= debug_interval_ms)) {
            print_stats();
            last_debug_ms = now_ms;
        }
    }
}
