idf_component_register(
    SRCS "src/mcp3008_driver.cpp"
         "src/mcp3008_linesensor.cpp"
    INCLUDE_DIRS "src"
    REQUIRES "esp_driver_spi" "esp_driver_gpio"
)
