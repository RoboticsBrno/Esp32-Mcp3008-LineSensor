#pragma once

#include <vector>
#include <atomic>

#include <driver/spi_master.h>

namespace mcp3008 {

class LineSensorCalibrator;

/**
 * \brief The LineSensor device - one MCP3008 chip.
 *
 * This class is not thread-safe, you have to make sure the methods are called
 * from one thread at a time only.
 * The @{install} method has to be called before you can use any other methods.
 */
class LineSensor {
public:
    static constexpr int CHANNELS = 8; //!< Amount of channels on the chip
    static constexpr uint16_t MAX_VAL = 1023; //!< Maximum value returned by from the chip (10bits).

    /**
     * \brief The LineSensor SPI configuration.
     */
    struct Config {
        Config(gpio_num_t pin_cs = GPIO_NUM_25, gpio_num_t pin_mosi = GPIO_NUM_33,
            gpio_num_t pin_miso = GPIO_NUM_32, gpio_num_t pin_sck = GPIO_NUM_26,
            uint8_t channels_mask = 0xFF, int freq = 1350000, spi_host_device_t spi_dev = HSPI_HOST) {
            this->freq = freq;
            this->spi_dev = spi_dev;
            this->channels_mask = channels_mask;

            this->pin_cs = pin_cs;
            this->pin_mosi = pin_mosi;
            this->pin_miso = pin_miso;
            this->pin_sck = pin_sck;
        }

        int freq; //!< SPI communication frequency
        spi_host_device_t spi_dev; //!< Which ESP32 SPI device to use.
        uint8_t channels_mask; //!< Which channels to use, bit mask:
                               //!< (1 << 0) | (1 << 2) == channels 0 and 2 only.

        gpio_num_t pin_cs;
        gpio_num_t pin_mosi;
        gpio_num_t pin_miso;
        gpio_num_t pin_sck;
    };

    /**
     * \brief The LineSensor's calibration data
     *
     * You should have no reason to edit these values directly,
     * use @{LineSensorCalibrator} instead.
     */
    struct CalibrationData {
        CalibrationData() {
            for(int i = 0; i < CHANNELS; ++i) {
                min[i] = 0;
                range[i] = MAX_VAL;
            }
        }

        uint16_t min[CHANNELS];
        uint16_t range[CHANNELS];
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

    uint8_t getChannelsMask() const { return m_channels_mask; } //<! Get the channel mask, specified in @{Config}

    /**
     * \brief Start the sensor line calibration procedure.
     *
     * \return @{LineSensorCalibrator} object used for calibration.
     */
    LineSensorCalibrator startCalibration();

    /**
     * \brief Get the calibration data used by linesensor, feel free to save this
     *        structure somewhere and load it afterwards using @{setCalibration}.
     *
     * \return @{CalibrationData} reference
     */
    const CalibrationData& getCalibration() const { return m_calibration; }

    /**
     * \brief Set calibration data used by the line sensor.
     *
     * \param data the calibration data obtained previously from @{getCalibration}.
     */
    void setCalibration(const CalibrationData& data) { m_calibration = data; }

    /**
     * \brief Read values from the chip. Returns values in range <0; @{MAX_VAL}>.
     *
     * \param results the results will be APPENDED to this vector.
     *        It will be unchanged unless the ESP_OK result is returned (except possibly its capacity).
     *        Between 0 and @{CHANNELS} values are appended, depending on the @{channels_mask} param.
     * \param useCalibration scale sensor values with calibration data. In default, uncalibrated state,
     *        read returns the same values regardless of this param.
     *        Use @{startCalibration} and @{LineSensorCalibrator} to generate the calibration data.
     * \param differential return differential readings, as specified in the MCP3008 datasheet.
     * \return ESP_OK or any error code encountered during reading.
     *         Will return ESP_FAIL if called when not installed.
     */
    esp_err_t read(std::vector<uint16_t>& results, bool useCalibration = true, bool differential = false) const;

    /**
     * \brief See the other @{read} method.
     *
     * \param dest array MUST be big enough to accomodate all the channels specified by @{channels_mask}!
     */
    esp_err_t read(uint16_t *dest, bool useCalibration = true, bool differential = false) const;

    /**
     * \brief Try to determine a black line's position under the sensors.
     *
     * \param white_line the line is white on black background instead of default black on white background.
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
    float readLine(bool white_line = false, float noise_limit=0.05f, float line_threshold=0.20f) const;

private:
    LineSensor(const LineSensor&) = delete;

    int requestToChannel(int request) const;

    CalibrationData m_calibration;
    spi_device_handle_t m_spi;
    spi_host_device_t m_spi_dev;
    bool m_installed;
    uint8_t m_channels_mask;
};

/**
 * \brief This class represents a single sensor calibration session.
 *
 * Typical calibration session consists of moving the sensors over
 * the line while calling the @{record} method over and over,
 * and then storing the data to the LineSensor instance via the
 * @{save} method.
 *
 * The parent LineSensor is modified only by the @{save} method.
 * This calibrator can be reused multiple times by calling
 * the @{reset} method between each session.
 *
 * Instances of this class are created via LineSensor's @{startCalibration}.
 * It must not outlive the parent LineSensor object.
 */
class LineSensorCalibrator {
    friend class LineSensor;
public:
    ~LineSensorCalibrator();

    void reset(); //!< Reset this calibrator to the initial state.

    /**
     * \brief Record current sensor values.
     *
     * Call this repeatedly while moving the sensors over the line.
     *
     * \return potential error returned by LineSensor's @{read}
     */
    esp_err_t record();

    void save(); //!< Store the calibrated values to the parent LineSensor.

private:
    LineSensorCalibrator(LineSensor& sensor);

    LineSensor& m_sensor;
    LineSensor::CalibrationData m_data;
    uint16_t m_max[LineSensor::CHANNELS];
};

}; // namespace mcp3008
