// File: main.c

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "pico/stdio_usb.h"

#include "pdm_pdm_capture.pio.h"  // generated from .pio

// --------- Pin / channel configuration ---------
#define MIC_COUNT           2

#define PDM_DATA_PIN_0      2   // Mic 0 data (GPIO 2)
#define PDM_DATA_PIN_1      3   // Mic 1 data (GPIO 3, adjacent)
#define PDM_CLK_PIN         4   // PDM clock pin driven by pdm_clock SM
#define PDM_TRIGGER_PIN     5   // Trigger pin shared by both mics

#define SPEAKER_OUTPUT_PIN  6   // Speaker drive GPIO (PIO sideset)
#define SPEAKER_TRIGGER_PIN PDM_TRIGGER_PIN
#define SPEAKER_TONE_HZ     100000u    // Speaker tone frequency
#define SPEAKER_DURATION_MS 5u     // Speaker tone duration per trigger

static const uint g_data_pins[MIC_COUNT] = { PDM_DATA_PIN_0, PDM_DATA_PIN_1 };

// --------- PDM timing / capture parameters ---------
#define PDM_BITRATE         5000000u       // 4 MHz PDM bit clock

#define CAPTURE_MS_DEFAULT  15u            // default capture window
#define CAPTURE_MS_MAX      30u            // maximum allowed capture length

// We allocate enough buffer for the maximum window.
#define MAX_CAPTURE_BITS    ((uint64_t)PDM_BITRATE * CAPTURE_MS_MAX / 1000u)
#define MAX_CAPTURE_WORDS   ((MAX_CAPTURE_BITS + 31u) / 32u)
#define MAX_CAPTURE_BYTES   (MAX_CAPTURE_WORDS * sizeof(uint32_t))

// --------- USB framing ---------
typedef struct __attribute__((packed)) {
    uint32_t magic;      // Magic constant to identify frame
    uint32_t index;      // Capture index
    uint32_t bit_count;  // Number of valid bits (32 * word_count)
    uint32_t channel_count; // Number of PDM data channels bundled
} capture_header_t;

#define CAPTURE_MAGIC   0xABCD1234u

// --------- Globals ---------
static PIO      g_pio   = pio0;
static uint     g_sm_clk;
static uint     g_sm_cap[MIC_COUNT];
static uint     g_sm_speaker;
static uint     g_offset_clk;
static uint     g_offset_cap;
static uint     g_offset_speaker;

static int      g_dma_chan[MIC_COUNT];

static volatile bool     g_capture_done[MIC_COUNT] = {false};
static uint32_t          g_capture_index = 0;

// Configurable capture length (ms) and derived parameters
static uint32_t          g_capture_ms    = CAPTURE_MS_DEFAULT;
static uint32_t          g_capture_words = 0;
static uint32_t          g_speaker_cycles = 0;

// Capture buffers (max size per mic)
static uint32_t g_capture_buf[MIC_COUNT][MAX_CAPTURE_WORDS];

// --------- DMA IRQ handler ---------
void __isr dma_irq_handler(void) {
    uint32_t ints = dma_hw->ints0;
    uint32_t clear_mask = 0;

    for (size_t i = 0; i < MIC_COUNT; ++i) {
        uint32_t mask = 1u << g_dma_chan[i];
        if (ints & mask) {
            clear_mask |= mask;
            g_capture_done[i] = true;
        }
    }

    if (clear_mask) {
        dma_hw->ints0 = clear_mask;  // clear handled interrupts
    }
}

// --------- Capture length configuration ---------
static void update_capture_params(uint32_t new_ms) {
    if (new_ms == 0) {
        new_ms = 1;  // minimum 1 ms
    }
    if (new_ms > CAPTURE_MS_MAX) {
        new_ms = CAPTURE_MS_MAX;
    }
    g_capture_ms = new_ms;

    // Compute desired bits
    uint64_t bits = (uint64_t)PDM_BITRATE * g_capture_ms / 1000u;

    // Convert to 32-bit words (round up)
    g_capture_words = (uint32_t)((bits + 31u) / 32u);
    if (g_capture_words > MAX_CAPTURE_WORDS) {
        g_capture_words = MAX_CAPTURE_WORDS;
    }
}

// --------- USB send function ---------
static void send_capture_over_usb(uint32_t index,
                                  size_t words,
                                  size_t channel_count,
                                  const uint32_t (*buffers)[MAX_CAPTURE_WORDS])
{
    capture_header_t hdr = {
        .magic     = CAPTURE_MAGIC,
        .index     = index,
        .bit_count = (uint32_t)(words * 32u),
        .channel_count = (uint32_t)channel_count,
    };

    // Send header and payload over stdio USB as raw bytes
    fwrite(&hdr, sizeof(hdr), 1, stdout);
    for (size_t ch = 0; ch < channel_count; ++ch) {
        fwrite(buffers[ch], sizeof(uint32_t), words, stdout);
    }
    fflush(stdout);
}

// --------- PIO init helpers ---------
static void pdm_clock_pio_init(PIO pio,
                               uint sm,
                               uint offset,
                               uint clk_pin)
{
    pio_sm_config c = pdm_clock_program_get_default_config(offset);

    sm_config_set_sideset_pins(&c, clk_pin);

    float clkdiv = (float)clock_get_hz(clk_sys) / (float)(PDM_BITRATE * 2u);
    sm_config_set_clkdiv(&c, clkdiv);

    pio_gpio_init(pio, clk_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, clk_pin, 1, true); // clock pin driven as output

    pio_sm_init(pio, sm, offset, &c);
}


static void pdm_capture_pio_init(PIO pio,
                                 uint sm,
                                 uint offset,
                                 uint data_pin,
                                 uint trigger_pin)
{
    pio_sm_config c = pdm_trigger_capture_program_get_default_config(offset);

    // IN base pin is data_pin for this mic; jmp pin is the shared trigger.
    sm_config_set_in_pins(&c, data_pin);
    sm_config_set_jmp_pin(&c, trigger_pin);

    // Shift configuration: shift right, no autopush, threshold 32 bits
    sm_config_set_in_shift(&c, true, false, 32);

    // Same clock divider as clock SM so timing stays in phase
    float clkdiv = (float)clock_get_hz(clk_sys) / (float)(PDM_BITRATE * 2u);
    sm_config_set_clkdiv(&c, clkdiv);

    // Map GPIOs to PIO (just function multiplexer, not direction)
    pio_gpio_init(pio, data_pin);      // GPIO 2
    pio_gpio_init(pio, trigger_pin);   // GPIO 4

    // DO NOT set directions here (GPIOs default to input, which we want for data/trigger)
    // This avoids messing with GPIO 3, which is driven as output by the clock SM.

    pio_sm_init(pio, sm, offset, &c);
}

static void speaker_pio_init(void)
{
    float clkdiv = (float)clock_get_hz(clk_sys) / (float)(SPEAKER_TONE_HZ * 2u);
    if (clkdiv < 1.0f) {
        clkdiv = 1.0f;
    }

    trigger_wave_program_init(g_pio, g_sm_speaker, g_offset_speaker,
                              clkdiv, SPEAKER_TRIGGER_PIN, SPEAKER_OUTPUT_PIN);

    g_speaker_cycles =
        (uint32_t)((uint64_t)SPEAKER_TONE_HZ * SPEAKER_DURATION_MS / 1000u);
    if (g_speaker_cycles == 0) {
        g_speaker_cycles = 1;
    }

    // Provide loop count to PIO before enabling.
    pio_sm_put_blocking(g_pio, g_sm_speaker, g_speaker_cycles);
}


// --------- DMA setup helper ---------
static void dma_init_channel(size_t ch)
{
    g_dma_chan[ch] = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(g_dma_chan[ch]);

    channel_config_set_read_increment(&cfg, false);  // PIO FIFO address fixed
    channel_config_set_write_increment(&cfg, true);  // write through buffer
    channel_config_set_dreq(&cfg, pio_get_dreq(g_pio, g_sm_cap[ch], false)); // PIO RX
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);

    dma_channel_configure(
        g_dma_chan[ch],
        &cfg,
        g_capture_buf[ch],           // write address
        &g_pio->rxf[g_sm_cap[ch]],   // read from capture SM RX FIFO
        MAX_CAPTURE_WORDS,           // max transfer count (override per capture)
        false                        // don't start yet
    );

    dma_channel_set_irq0_enabled(g_dma_chan[ch], true);
}

static void dma_init_for_pio_capture(void)
{
    for (size_t ch = 0; ch < MIC_COUNT; ++ch) {
        dma_init_channel(ch);
    }
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
}

// --------- Main ---------
int main() {
    stdio_init_all();   // optional (USB debug)
    stdio_set_translate_crlf(&stdio_usb, false);  // binary framing over USB CDC

    // Wait for USB CDC connection so host doesn't miss the first capture
    while (!stdio_usb_connected()) {
        sleep_ms(50);
    }

    // Trigger pin: pull-down so it's defined when idle
    gpio_init(PDM_TRIGGER_PIN);
    gpio_pull_down(PDM_TRIGGER_PIN);

    // PIO program setup
    g_pio = pio0;

    // Load programs into PIO instruction memory
    g_offset_clk = pio_add_program(g_pio, &pdm_clock_program);
    g_offset_cap = pio_add_program(g_pio, &pdm_trigger_capture_program);
    g_offset_speaker = pio_add_program(g_pio, &trigger_wave_program);

    // Claim state machines (one clock + MIC_COUNT capture SMs)
    g_sm_clk = pio_claim_unused_sm(g_pio, true);
    for (size_t ch = 0; ch < MIC_COUNT; ++ch) {
        g_sm_cap[ch] = pio_claim_unused_sm(g_pio, true);
    }
    g_sm_speaker = pio_claim_unused_sm(g_pio, true);

    // Initialize (not yet enabled)
    pdm_clock_pio_init(g_pio, g_sm_clk, g_offset_clk, PDM_CLK_PIN);
    for (size_t ch = 0; ch < MIC_COUNT; ++ch) {
        pdm_capture_pio_init(g_pio, g_sm_cap[ch], g_offset_cap,
                             g_data_pins[ch], PDM_TRIGGER_PIN);
    }
    speaker_pio_init();

    // Configure default capture window
    update_capture_params(CAPTURE_MS_DEFAULT);

    // DMA setup for capture SM
    dma_init_for_pio_capture();

    // Start both SMs in sync so they share a phase reference
    uint32_t sm_mask = (1u << g_sm_clk) | (1u << g_sm_speaker);
    for (size_t ch = 0; ch < MIC_COUNT; ++ch) {
        sm_mask |= (1u << g_sm_cap[ch]);
    }
    pio_enable_sm_mask_in_sync(g_pio, sm_mask);

    // Main loop: arm capture, wait for trigger, send over USB CDC
    while (true) {
        for (size_t ch = 0; ch < MIC_COUNT; ++ch) {
            g_capture_done[ch] = false;
        }

        // Re-configure DMA for this capture
        for (size_t ch = 0; ch < MIC_COUNT; ++ch) {
            dma_channel_set_read_addr(g_dma_chan[ch], &g_pio->rxf[g_sm_cap[ch]], false);
            dma_channel_set_write_addr(g_dma_chan[ch], g_capture_buf[ch], false);
            dma_channel_set_trans_count(g_dma_chan[ch], g_capture_words, false);
        }

        // Start DMA before capture so it drains FIFO as words arrive
        uint32_t dma_mask = 0;
        for (size_t ch = 0; ch < MIC_COUNT; ++ch) {
            dma_mask |= (1u << g_dma_chan[ch]);
        }
        dma_start_channel_mask(dma_mask);

        // Tell each capture SM how many 32-bit words to capture
        for (size_t ch = 0; ch < MIC_COUNT; ++ch) {
            pio_sm_put_blocking(g_pio, g_sm_cap[ch], g_capture_words);
        }

        // PIO now:
        //   pull length -> wait for trigger low -> wait for trigger high ->
        //   capture N words -> push them to RX FIFO
        // DMA drains RX FIFO into g_capture_buf.

        // Wait for DMA completion
        while (true) {
            bool all_done = true;
            for (size_t ch = 0; ch < MIC_COUNT; ++ch) {
                if (!g_capture_done[ch]) {
                    all_done = false;
                    break;
                }
            }
            if (all_done) {
                break;
            }
            tight_loop_contents();
        }

        // Send this capture over USB
        send_capture_over_usb(g_capture_index, g_capture_words,
                              MIC_COUNT, g_capture_buf);
        g_capture_index++;

        // Optional small pause to avoid hammering the host
        sleep_ms(10);
    }

    return 0;
}
