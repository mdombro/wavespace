# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_pdm_capture C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico_pdm_capture
    main.c
)

pico_generate_pio_header(pico_pdm_capture ${CMAKE_CURRENT_LIST_DIR}/pdm_pdm_capture.pio)

target_link_libraries(pico_pdm_capture
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_uart
    hardware_clocks
)

pico_add_extra_outputs(pico_pdm_capture)

pico_enable_stdio_usb(pico_pdm_capture 1)
pico_enable_stdio_uart(pico_pdm_capture 0)
