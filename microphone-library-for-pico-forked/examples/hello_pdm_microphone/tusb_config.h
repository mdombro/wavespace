#ifndef _HELLO_PDM_TUSB_CONFIG_H_
#define _HELLO_PDM_TUSB_CONFIG_H_

// TinyUSB configuration for the hello_pdm_microphone example.
// Only the CDC class is enabled for the interactive CLI.

#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE 64
#endif

#define CFG_TUD_CDC 1
#define CFG_TUD_VENDOR 0

// CDC buffers large enough for the interactive CLI.
#define CFG_TUD_CDC_RX_BUFSIZE 512
#define CFG_TUD_CDC_TX_BUFSIZE 512
#define CFG_TUD_CDC_EP_BUFSIZE 64

#endif  // _HELLO_PDM_TUSB_CONFIG_H_
