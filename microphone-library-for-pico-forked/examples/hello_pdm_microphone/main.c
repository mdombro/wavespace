#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/sync.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/pdm_microphone.h"

#include "tusb.h"

// Example entry point: capture PDM audio and stream over USB CDC and/or UDP.

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

#define USB_VENDOR_QUEUE_CAPACITY 16

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
    uint16_t length;
    capture_slot_state_t state;
    uint8_t pending_outputs;
} capture_slot_t;

static capture_slot_t capture_ring[CAPTURE_RING_DEPTH];
static volatile uint8_t capture_write_index = 0;
static volatile uint8_t capture_read_index = 0;
static volatile uint8_t capture_count = 0;
static pdm_block_packet_t discard_packet;

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

typedef struct {
    capture_slot_t* slot;
    uint32_t length;
    uint32_t offset;
} usb_vendor_queue_entry_t;

static usb_vendor_queue_entry_t usb_vendor_queue[USB_VENDOR_QUEUE_CAPACITY];
static uint8_t usb_vendor_queue_head = 0;
static uint8_t usb_vendor_queue_tail = 0;
static uint8_t usb_vendor_queue_count = 0;

static uint32_t usb_vendor_packets_sent = 0;
static uint32_t usb_vendor_bytes_sent = 0;
static uint32_t usb_vendor_backpressure_events = 0;
static uint32_t usb_vendor_failures = 0;
static uint32_t usb_vendor_queue_drops = 0;

static uint32_t serial_packets_sent = 0;
static uint32_t serial_bytes_sent = 0;
static uint32_t serial_failures = 0;

static bool serial_initialised = false;

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
static void initialise_uart_stream(void);
static bool usb_vendor_queue_push_slot(capture_slot_t* slot, size_t length);
static void usb_vendor_queue_service(void);
static void usb_vendor_queue_clear(void);
static bool serial_stream_block(const uint8_t* data, size_t length);

static inline void capture_slot_release(capture_slot_t* slot) {
    if (!slot) {
        return;
    }
    slot->length = 0;
    slot->state = CAPTURE_SLOT_FREE;
    slot->pending_outputs = 0;
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
        capture_ring[i].length = 0;
        capture_ring[i].state = CAPTURE_SLOT_FREE;
        capture_ring[i].pending_outputs = 0;
    }
    restore_interrupts(status);

    clear_stats();
    last_debug_ms = to_ms_since_boot(get_absolute_time());
    usb_vendor_queue_clear();
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

static void on_pdm_samples_ready(void) {
    // Library callback from DMA IRQ context when a raw buffer has filled.
    uint8_t write_index = capture_write_index;
    capture_slot_t* slot = &capture_ring[write_index];
    bool slot_available = (slot->state == CAPTURE_SLOT_FREE);
    pdm_block_packet_t* target_packet = slot_available ? &slot->packet : &discard_packet;
    int read = pdm_microphone_read_with_metadata(
        target_packet->payload,
        PDM_BLOCK_PAYLOAD_BYTES,
        &target_packet->metadata
    );

    if (read > 0) {
        if (slot_available) {
            slot->length = (uint16_t)read;
            slot->state = CAPTURE_SLOT_READY;
            slot->pending_outputs = 0;
            capture_write_index = (uint8_t)((capture_write_index + 1) % CAPTURE_RING_DEPTH);
            capture_count++;
            blocks_ready++;
        } else {
            blocks_discarded++;
        }
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
    printf("  u   - toggle USB vendor streaming (currently %s)\n", usb_vendor_stream_enabled ? "ON" : "OFF");
    printf("  t   - toggle UART streaming (currently %s)\n", serial_stream_enabled ? "ON" : "OFF");
    printf("  n   - print transport status information\n\n");
}

static void print_stats(void) {
    const char* usb_state = usb_vendor_stream_enabled
        ? (tud_vendor_mounted() ? "ON" : "waiting")
        : "OFF";

    const char* serial_state = serial_stream_enabled
        ? (serial_initialised ? "ON" : "init")
        : "OFF";

    printf("Stats: ready=%lu streamed=%lu discarded=%lu overruns=%lu bytes=%lu stream=%s dump=%s pdm_clk=%luHz "
           "usb=%s pkt=%lu bytes=%lu err=%lu backp=%lu drop=%lu "
           "uart=%s pkt=%lu bytes=%lu err=%lu\n",
           (unsigned long)blocks_ready,
           (unsigned long)blocks_streamed,
           (unsigned long)blocks_discarded,
           (unsigned long)overruns,
           (unsigned long)bytes_sent,
           stream_binary_data ? "ON" : "OFF",
           debug_dump_samples ? "ON" : "OFF",
           (unsigned long)config.sample_rate,
           usb_state,
           (unsigned long)usb_vendor_packets_sent,
           (unsigned long)usb_vendor_bytes_sent,
           (unsigned long)usb_vendor_failures,
            (unsigned long)usb_vendor_backpressure_events,
            (unsigned long)usb_vendor_queue_drops,
           serial_state,
           (unsigned long)serial_packets_sent,
           (unsigned long)serial_bytes_sent,
           (unsigned long)serial_failures);
}

static void print_transport_status(void) {
    printf("USB vendor streaming %s\n", usb_vendor_stream_enabled ? "ENABLED" : "DISABLED");
    printf("  mounted:     %s\n", tud_vendor_mounted() ? "YES" : "NO");
    printf("  packets:     %lu\n", (unsigned long)usb_vendor_packets_sent);
    printf("  bytes:       %lu\n", (unsigned long)usb_vendor_bytes_sent);
    printf("  backpressure:%lu\n", (unsigned long)usb_vendor_backpressure_events);
    printf("  failures:    %lu\n", (unsigned long)usb_vendor_failures);
    printf("  queue drops: %lu\n", (unsigned long)usb_vendor_queue_drops);

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

    usb_vendor_packets_sent = 0;
    usb_vendor_bytes_sent = 0;
    usb_vendor_backpressure_events = 0;
    usb_vendor_failures = 0;
    usb_vendor_queue_drops = 0;

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
            case 'u':
                usb_vendor_stream_enabled = !usb_vendor_stream_enabled;
                printf("USB vendor streaming %s\n", usb_vendor_stream_enabled ? "ENABLED" : "DISABLED");
                if (!usb_vendor_stream_enabled) {
                    usb_vendor_queue_clear();
                }
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

static void usb_vendor_queue_drop_head(void) {
    if (usb_vendor_queue_count == 0) {
        return;
    }

    usb_vendor_queue_entry_t* entry = &usb_vendor_queue[usb_vendor_queue_head];
    if (entry->slot) {
        capture_slot_output_done(entry->slot);
        entry->slot = NULL;
    }
    entry->length = 0;
    entry->offset = 0;
    usb_vendor_queue_head = (uint8_t)((usb_vendor_queue_head + 1) % USB_VENDOR_QUEUE_CAPACITY);
    usb_vendor_queue_count--;
    usb_vendor_queue_drops++;
}

static void usb_vendor_queue_clear(void) {
    while (usb_vendor_queue_count > 0) {
        usb_vendor_queue_drop_head();
    }
    usb_vendor_queue_head = 0;
    usb_vendor_queue_tail = 0;
    usb_vendor_queue_count = 0;
}

static bool usb_vendor_queue_push_slot(capture_slot_t* slot, size_t length) {
    if (!slot || length == 0) {
        return true;
    }

    if (!tud_vendor_mounted()) {
        return false;
    }

    if (length > PDM_BLOCK_PACKET_BYTES) {
        return false;
    }

    if (usb_vendor_queue_count >= USB_VENDOR_QUEUE_CAPACITY) {
        // Give the service loop a chance to flush before dropping.
        usb_vendor_queue_service();
        if (usb_vendor_queue_count >= USB_VENDOR_QUEUE_CAPACITY) {
            usb_vendor_queue_drop_head();
            if (usb_vendor_queue_count >= USB_VENDOR_QUEUE_CAPACITY) {
                return false;
            }
        }
    }

    usb_vendor_queue_entry_t* entry = &usb_vendor_queue[usb_vendor_queue_tail];
    entry->slot = slot;
    entry->length = (uint32_t)length;
    entry->offset = 0;
    slot->pending_outputs++;

    usb_vendor_queue_tail = (uint8_t)((usb_vendor_queue_tail + 1) % USB_VENDOR_QUEUE_CAPACITY);
    usb_vendor_queue_count++;
    return true;
}

static void usb_vendor_queue_service(void) {
    if (!usb_vendor_stream_enabled || usb_vendor_queue_count == 0) {
        return;
    }

    if (!tud_vendor_mounted()) {
        usb_vendor_queue_clear();
        return;
    }

    while (usb_vendor_queue_count > 0) {
        usb_vendor_queue_entry_t* pkt = &usb_vendor_queue[usb_vendor_queue_head];
        capture_slot_t* slot = pkt->slot;
        if (!slot) {
            usb_vendor_queue_head = (uint8_t)((usb_vendor_queue_head + 1) % USB_VENDOR_QUEUE_CAPACITY);
            usb_vendor_queue_count--;
            continue;
        }

        if (pkt->offset >= pkt->length) {
            usb_vendor_packets_sent++;
            usb_vendor_bytes_sent += pkt->length;
            capture_slot_output_done(slot);
            pkt->slot = NULL;
            pkt->offset = 0;
            pkt->length = 0;
            usb_vendor_queue_head = (uint8_t)((usb_vendor_queue_head + 1) % USB_VENDOR_QUEUE_CAPACITY);
            usb_vendor_queue_count--;
            continue;
        }

        tud_task();
        uint32_t available = tud_vendor_write_available();
        if (available == 0) {
            usb_vendor_backpressure_events++;
            break;
        }

        const uint8_t* data = (const uint8_t*)&slot->packet;
        uint32_t remaining = pkt->length - pkt->offset;
        uint32_t chunk = remaining < available ? remaining : available;
        uint32_t written = tud_vendor_write(&data[pkt->offset], chunk);
        tud_vendor_flush();

        if (written == 0) {
            usb_vendor_backpressure_events++;
            break;
        }

        pkt->offset += written;
    }
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
        usb_vendor_queue_service();

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
                slot->pending_outputs = 1;  // main loop holds one reference
                bytes_to_send = slot->length;
                packet = &slot->packet;
                packet_bytes = sizeof(packet->metadata) + (size_t)bytes_to_send;
                capture_read_index = (uint8_t)((capture_read_index + 1) % CAPTURE_RING_DEPTH);
                capture_count--;
            }
        }
        restore_interrupts(status);

        if (packet == NULL || bytes_to_send <= 0) {
            overruns++;
            continue;
        }

        if (debug_dump_samples) {
            dump_block_summary(packet, (size_t)bytes_to_send);
        }

        const uint8_t* packet_data = (const uint8_t*)packet;

        if (usb_vendor_stream_enabled) {
            if (!usb_vendor_queue_push_slot(slot, packet_bytes)) {
                usb_vendor_failures++;
            }
            usb_vendor_queue_service();
        }

        if (serial_stream_enabled) {
            if (!serial_stream_block(packet_data, packet_bytes)) {
                serial_failures++;
            }
        }

        if (stream_binary_data) {
            usb_write_blocking(packet_data, packet_bytes);
            bytes_sent += (uint32_t)bytes_to_send;
            blocks_streamed++;
        }

        capture_slot_output_done(slot);

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (debug_stats_enabled && (now_ms - last_debug_ms >= debug_interval_ms)) {
            print_stats();
            last_debug_ms = now_ms;
        }
    }
}
