#pragma once

#include <vector>
#include <atomic>

#include <driver/spi_master.h>

namespace mcp3008 {

/**
 * \brief The LineSensor device - one MCP3008 chip.
 *        It is not thread-safe, you have to make sure the methods are called
 *        from one thread at a time only.
 *        The @{install} method has to be called before you can use any other methods.
 */
class LineSensor {
public:
    static constexpr int CHANNELS = 8; //!< Amount of channels on the chip
    static constexpr uint16_t MAX_VAL = 1023; //!< Maximum value returned by from the chip (10bits).

    /**
     * \brief The LineSensor SPI configuration.
     */
    struct Config {
        Config(int freq = 1350000, spi_host_device_t spi_dev = HSPI_HOST,
            gpio_num_t pin_cs = GPIO_NUM_25, gpio_num_t pin_mosi = GPIO_NUM_33,
            gpio_num_t pin_miso = GPIO_NUM_32, gpio_num_t pin_sck = GPIO_NUM_26) {
            this->freq = freq;
            this->spi_dev = spi_dev;

            this->pin_cs = pin_cs;
            this->pin_mosi = pin_mosi;
            this->pin_miso = pin_miso;
            this->pin_sck = pin_sck;
        }

        int freq; //!< SPI communication frequency
        spi_host_device_t spi_dev; //!< Which ESP32 SPI device to use.

        gpio_num_t pin_cs;
        gpio_num_t pin_mosi;
        gpio_num_t pin_miso;
        gpio_num_t pin_sck;
    };

    LineSensor();
    virtual ~LineSensor(); //!< The @{uninstall} method is called from the destructor.

    /**
     * \brief Initialize the SPI bus. Must be called before any other methods,
     *        otherwise they will return ESP_FAIL.
     *
     * \param cfg the SPI bus configuration.
     * \return ESP_OK or any error code encountered during the inialization.
     *         Will return ESP_FAIL if called when already installed.
     */
    esp_err_t install(const Config& cfg = Config());

    /**
     * \brief Free the SPI bus. Must be called last, or other methods return ESP_FAIL.
     *
     * \return ESP_OK or any error code encountered during the freeing.
     *         Will return ESP_FAIL if called when not installed.
     */
    esp_err_t uninstall();

    /**
     * \brief Read values from the chip. Returns values in range <0; @{MAX_VAL}>.
     *
     * \param results the results will be APPENDED to this vector.
     *        It will be unchanged unless the ESP_OK result is returned (except possibly its capacity).
     *        Between 0 and @{CHANNELS} values are appended, depending on the @{channels_mask} param.
     * \param differential return differential readings, as specified in the MCP3008 datasheet.
     * \param channels_mask which channels to read. Example:
     *        (1 << 0) | (1 << 2) - read channels 0 and 2. Two values will be appended to the results vector.
     *        The values will be in the order of their channel id - value from channel 0 will always be before
     *        value from the channel 2 in this example.
     * \return ESP_OK or any error code encountered during reading.
     *         Will return ESP_FAIL if called when not installed.
     */
    esp_err_t read(std::vector<uint16_t>& results, bool differential = false, uint8_t channels_mask = 0xFF);

    /**
     * \brief See the other @{read} method.
     *
     * \param dest array MUST be big enough to accomodate all the channels specified by @{channels_mask}!
     */
    esp_err_t read(uint16_t *dest, bool differential = false, uint8_t channels_mask = 0xFF);

    /**
     * \brief Try to determine a black line's position under the sensors.
     *
     * \param white_line the line is white on black background instead of default black on white background.
     * \param channels_mask which channels to read. Example:
     *        (1 << 0) | (1 << 2) - read channels 0 and 2. Two values will be appended to the results vector.
     *        The values will be in the order of their channel id - value from channel 0 will always be before
     *        value from the channel 2 in this example.
     * \param noise_limit values below this threshold will be considered noise and ignored.
     *        The value is a fraction, 0.05 == 5% == (1023*0.05) == values < ~50 will be ignored.
     * \param line_threshold values above this threshild will be considered "on the line".
     *        At least one sensor has to have value above this threshold, otherwise NaN is returned.
     *        The value is a fraction, 0.20 == 20% == (1023*0.20) == 200.
     * \return A number in range <-1,1>, where -1 means the line is under the channel with smallest ID,
     *         0 means in the middle and 1 means it is under the one with the greatest ID.
     *         Examples, assuming using 8 channels:
     *             -1.0: under channel 0
     *              0.0: under channel 3-4
     *              1.0: under channel 7
     *         Returns NaN when the line is not found (see @{line_threshold}).
     */
    float readLine(bool white_line = false, uint8_t channels_mask=0xFF, float noise_limit=0.05f, float line_threshold=0.20f);

private:
    LineSensor(const LineSensor&) = delete;

    bool m_installed;
    spi_device_handle_t m_spi;
    spi_host_device_t m_spi_dev;
};

}; // namespace mcp3008
