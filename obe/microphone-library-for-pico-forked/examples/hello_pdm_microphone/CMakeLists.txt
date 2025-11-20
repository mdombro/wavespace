cmake_minimum_required(VERSION 3.12)

# rest of your project
add_executable(hello_pdm_microphone
    main.c
    usb_descriptors.c
)

target_link_libraries(hello_pdm_microphone
    pico_pdm_microphone
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    tinyusb_device
    tinyusb_board
    hardware_spi
    hardware_pio
    hardware_clocks
)

target_include_directories(hello_pdm_microphone PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_compile_definitions(hello_pdm_microphone PRIVATE
    CFG_TUSB_CONFIG_FILE=\"tusb_config.h\"
    PICO_STDIO_USB_ENABLE_TINYUSB_INIT=1
    PICO_STDIO_USB_USE_DEFAULT_DESCRIPTORS=0
)

# enable usb output, disable uart output
pico_enable_stdio_usb(hello_pdm_microphone 1)
pico_enable_stdio_uart(hello_pdm_microphone 0)

# generate PIO headers for the trigger wave program
pico_generate_pio_header(hello_pdm_microphone ${CMAKE_CURRENT_LIST_DIR}/../../src/trigger_wave.pio)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(hello_pdm_microphone)
