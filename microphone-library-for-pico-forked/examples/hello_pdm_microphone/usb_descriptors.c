#include <string.h>

#include "pico/unique_id.h"
#include "tusb.h"

#define USB_VID 0x2E8A
#define USB_PID 0x4001
#define USB_BCD 0x0200

enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_VENDOR,
    ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_VENDOR_DESC_LEN)

#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT 0x02
#define EPNUM_CDC_IN 0x82
#define EPNUM_VENDOR_OUT 0x03
#define EPNUM_VENDOR_IN 0x83

static tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = USB_BCD,

    // Use the IAD class so the host treats the CDC interfaces as a single function.
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,

    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,

    .bNumConfigurations = 0x01,
};

uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*)&desc_device;
}

static uint8_t const configuration_descriptor[] = {
    // Config number, interface count, string index, total length, attribute, power in mA.
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // CDC interface (control + data) for the CLI.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),

    // Vendor-specific interface for raw PDM streaming.
    TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, 5, EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN, 64),
};

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return configuration_descriptor;
}

static char serial_id[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
static uint16_t string_buf[32];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    const char* str = NULL;
    uint8_t chr_count = 0;

    switch (index) {
        case 0:
            string_buf[1] = 0x0409;  // English (United States)
            string_buf[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 + 2));
            return string_buf;
        case 1:
            str = "WaveSpace Audio";
            break;
        case 2:
            str = "PDM Capture Interface";
            break;
        case 3:
            pico_get_unique_board_id_string(serial_id, sizeof(serial_id));
            str = serial_id;
            break;
        case 4:
            str = "Debug CLI";
            break;
        case 5:
            str = "PDM Stream";
            break;
        default:
            return NULL;
    }

    chr_count = (uint8_t)strlen(str);
    if (chr_count > 31) {
        chr_count = 31;
    }

    for (uint8_t i = 0; i < chr_count; i++) {
        string_buf[1 + i] = (uint16_t)str[i];
    }

    string_buf[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (uint16_t)((chr_count * 2) + 2));
    return string_buf;
}

