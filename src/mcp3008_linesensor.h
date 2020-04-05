#pragma once

#include <driver/spi_master.h>
#include <vector>

#include "mcp3008_driver.h"

namespace mcp3008 {

class LineSensorCalibrator;

/**
 * \brief The LineSensor device, based on MCP3008 chip.
 *
 * This class is not thread-safe, you have to make sure the methods are called
 * from one thread at a time only.
 */
class LineSensor : public Driver {
public:
    /**
     * \brief The LineSensor's calibration data
     *
     * You should have no reason to edit these values directly,
     * use LineSensorCalibrator instead.
     */
    struct CalibrationData {
        uint16_t min[Driver::CHANNELS];
        uint16_t range[Driver::CHANNELS];
    } __attribute__((packed));

    LineSensor();
    virtual ~LineSensor();

    /**
     * \brief Start the sensor line calibration procedure.
     *
     * \return LineSensorCalibrator object used for calibration.
     */
    LineSensorCalibrator startCalibration();

    /**
     * \brief Get the calibration data used by linesensor, feel free to save this
     *        structure somewhere and load it afterwards using setCalibration().
     *
     * \return CalibrationData reference
     */
    const CalibrationData& getCalibration() const { return m_calibration; }

    /**
     * \brief Set calibration data used by the line sensor.
     *
     * \param data the calibration data obtained previously from getCalibration().
     * \return if the result is false, calibration data were invalid (out of range)
     */
    bool setCalibration(const CalibrationData& data);

    /**
     * \brief Try to determine a black line's position under the sensors.
     *
     * \param white_line the line is white on black background instead of default black on white background.
     * \param line_threshold values above this threshild will be considered "on the line".
     *        At least one sensor has to have value above this threshold, otherwise NaN is returned.
     *        The value is a fraction, 0.20 == 20% == (1023*0.20) == 200.
     * \return A number in range <-1,1>, where -1 means the line is under the channel with smallest ID,
     *         0 means in the middle and 1 means it is under the one with the greatest ID.
     *         Examples, assuming using 8 channels: \n
     *             -1.0: under channel 0 \n
     *              0.0: under channel 3-4 \n
     *              1.0: under channel 7 \n
     *         Returns NaN when the line is not found (see \p line_threshold).
     */
    float readLine(bool white_line = false, float line_threshold = 0.20f) const;

    /**
     * \brief Same as Driver::read(), but returns calibrated result if possible
     *
     * \param results the results will be APPENDED to this vector.
     *        It will be unchanged unless the ESP_OK result is returned (except possibly its capacity).
     *        Between 0 and Driver::CHANNELS values are appended, depending on Config::channels_mask.
     * \return ESP_OK or any error code encountered during reading.
     *         Will return ESP_FAIL if called when not installed.
     */
    esp_err_t calibratedRead(std::vector<uint16_t>& results) const;

    /**
     * \brief See the other calibratedRead(std::vector<uint16_t>&) const method.
     *
     * \param dest array MUST be big enough to accomodate all the channels specified by Config::channels_mask!
     */
    esp_err_t calibratedRead(uint16_t* dest) const;

    /**
     * \brief Same as Driver::readChannel(), but returns calibrated data.
     *
     * \param result result code will be written here, may be null.
     * \return measured value or 0xFFFF on error.
     */
    uint16_t calibratedReadChannel(uint8_t channel, esp_err_t* result = nullptr) const;

private:
    LineSensor(const LineSensor&) = delete;

    void calibrateResults(uint16_t* dest) const;
    inline uint16_t calibrateValue(int chan, uint16_t val) const;

    CalibrationData m_calibration;
};

/**
 * \brief This class represents a single sensor calibration session.
 *
 * Typical calibration session consists of moving the sensors over
 * the line while calling the record() method over and over,
 * and then storing the data to the LineSensor instance via the
 * save() method.
 *
 * The parent LineSensor is modified only by the save() method.
 * This calibrator can be reused multiple times by calling
 * the reset() method between each session.
 *
 * Instances of this class are created via LineSensor's startCalibration().
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
     * \return potential error returned by LineSensor's read()
     */
    esp_err_t record();

    void save(); //!< Store the calibrated values to the parent LineSensor.

private:
    LineSensorCalibrator(LineSensor& sensor);

    LineSensor& m_sensor;
    LineSensor::CalibrationData m_data;
    uint16_t m_max[Driver::CHANNELS];
};

}; // namespace mcp3008
