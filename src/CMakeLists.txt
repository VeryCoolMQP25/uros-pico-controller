add_executable(${NAME}
        main.c
        uart_logging.c
        pico_ros_usb.c
        actuators.c
        controls.c
        nav.c
        sensors.c
        )

# generate h file from pio description.
pico_generate_pio_header(${NAME} ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME}
	 pico_stdlib
     micro_ros
     hardware_pwm
     hardware_pio
     hardware_adc
     pico_multicore
     haw_mpu6050
	)

target_include_directories(${NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR})


# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
