#pragma once

#include <vector>
#include <driver/spi_master.h>

#include "driver.h"

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
        CalibrationData() {
            for(int i = 0; i < Driver::CHANNELS; ++i) {
                min[i] = 0;
                range[i] = Driver::MAX_VAL;
            }
        }

        uint16_t min[Driver::CHANNELS];
        uint16_t range[Driver::CHANNELS];
    };

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
     */
    void setCalibration(const CalibrationData& data) { m_calibration = data; }

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
     *         Examples, assuming using 8 channels: \n
     *             -1.0: under channel 0 \n
     *              0.0: under channel 3-4 \n
     *              1.0: under channel 7 \n
     *         Returns NaN when the line is not found (see \p line_threshold).
     */
    float readLine(bool white_line = false, float noise_limit=0.05f, float line_threshold=0.20f) const;

private:
    LineSensor(const LineSensor&) = delete;

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
