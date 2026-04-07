add_executable (gpio_printer gpio_printer.cpp)
target_link_libraries(gpio_printer gpioevent_static)
target_include_directories(gpio_printer PRIVATE ..)
