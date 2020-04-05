#include <cmath>
#include <esp_log.h>

#include "mcp3008_linesensor.h"

#define TAG "Mcp3008LineSensor"

namespace mcp3008 {

LineSensor::LineSensor()
    : Driver() {
    for (int i = 0; i < Driver::CHANNELS; ++i) {
        m_calibration.min[i] = 0;
        m_calibration.range[i] = Driver::MAX_VAL;
    }
}

LineSensor::~LineSensor() {
}

LineSensorCalibrator LineSensor::startCalibration() {
    return LineSensorCalibrator(*this);
}

float LineSensor::readLine(bool white_line, float line_threshold) const {
    std::vector<uint16_t> vals;
    auto res = this->calibratedRead(vals);
    if (res != ESP_OK || vals.size() == 0) {
        ESP_LOGE(TAG, "read() failed: %d", res);
        return nanf("");
    }

    uint32_t weighted = 0;
    uint16_t sum = 0;

    const uint16_t threshold = line_threshold * MAX_VAL;

    uint16_t min = MAX_VAL;
    uint16_t max = 0;
    for (size_t i = 0; i < vals.size(); ++i) {
        auto val = vals[i];
        if (white_line)
            val = MAX_VAL - val;

        if (val < min)
            min = val;
        if (val > max)
            max = val;
    }

    const uint16_t range = max - min;
    if (max < threshold || range < threshold)
        return nanf("");

    for (size_t i = 0; i < vals.size(); ++i) {
        auto val = vals[i];
        if (white_line)
            val = MAX_VAL - val;

        val = float(val - min) / range * MAX_VAL;

        weighted += uint32_t(val) * i * MAX_VAL;
        sum += val;
    }

    if (sum == 0)
        return nanf("");

    const int16_t middle = float(vals.size() - 1) / 2 * MAX_VAL;
    const int16_t result = (weighted / sum) - middle;

    return std::min(1.f, std::max(-1.f, float(result) / float(middle)));
}

bool LineSensor::setCalibration(const LineSensor::CalibrationData& data) {
    for (int i = 0; i < Driver::CHANNELS; ++i) {
        if ((getChannelsMask() & (1 << i)) == 0)
            continue;
        if (data.min[i] > Driver::MAX_VAL || data.range[i] > Driver::MAX_VAL || (data.min[i] + data.range[i]) > Driver::MAX_VAL) {
            ESP_LOGE(TAG, "invalid data in setCalibration at channel %d: %hu+%hu, ignoring calibration!",
                i, data.min[i], data.range[i]);
            return false;
        }
    }

    m_calibration = data;
    return true;
}

void LineSensor::calibrateResults(uint16_t* dest) const {
    const auto mask = getChannelsMask();
    int resIdx = 0;
    for (int chan = 0; chan < CHANNELS; ++chan) {
        if (((1 << chan) & mask) == 0)
            continue;
        dest[resIdx] = calibrateValue(chan, dest[resIdx]);
        ++resIdx;
    }
}

uint16_t LineSensor::calibrateValue(int chan, uint16_t val) const {
    if (val <= m_calibration.min[chan]) {
        return 0;
    } else {
        val = int32_t(val - m_calibration.min[chan]) * MAX_VAL / m_calibration.range[chan];
        return std::min(MAX_VAL, val);
    }
}

esp_err_t LineSensor::calibratedRead(std::vector<uint16_t>& results) const {
    const auto res = read(results);
    if (res == ESP_OK)
        calibrateResults(results.data());
    return res;
}

esp_err_t LineSensor::calibratedRead(uint16_t* dest) const {
    const auto res = read(dest);
    if (res == ESP_OK)
        calibrateResults(dest);
    return res;
}

uint16_t LineSensor::calibratedReadChannel(uint8_t channel, esp_err_t* result) const {
    auto val = readChannel(channel, false, result);
    return calibrateValue(channel, val);
}

LineSensorCalibrator::LineSensorCalibrator(LineSensor& sensor)
    : m_sensor(sensor) {
    reset();
}

LineSensorCalibrator::~LineSensorCalibrator() {
}

void LineSensorCalibrator::reset() {
    for (int i = 0; i < Driver::CHANNELS; ++i) {
        m_data.min[i] = Driver::MAX_VAL;
        m_max[i] = 0;
    }
}

esp_err_t LineSensorCalibrator::record() {
    uint16_t vals[Driver::CHANNELS];
    esp_err_t res = m_sensor.read(vals);
    if (res != 0) {
        return res;
    }

    int idx = 0;
    const auto mask = m_sensor.getChannelsMask();
    for (int i = 0; i < Driver::CHANNELS; ++i) {
        if (((1 << i) & mask) == 0)
            continue;

        if (vals[idx] < m_data.min[i])
            m_data.min[i] = vals[idx];
        if (vals[idx] > m_max[i])
            m_max[i] = vals[idx];
        ++idx;
    }

    return ESP_OK;
}

void LineSensorCalibrator::save() {
    for (int i = 0; i < Driver::CHANNELS; ++i) {
        m_data.range[i] = m_max[i] - m_data.min[i];
    }
    m_sensor.setCalibration(m_data);
}

}; // namespace mcp3008
