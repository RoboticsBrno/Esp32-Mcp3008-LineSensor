#include <cmath>
#include <esp_log.h>

#include "mcp3008_linesensor.h"

#define TAG "Mcp3008LineSensor"

namespace mcp3008 {

LineSensor::LineSensor() : Driver() {
    for(int i = 0; i < Driver::CHANNELS; ++i) {
        m_calibration.min[i] = 0;
        m_calibration.range[i] = Driver::MAX_VAL;
    }
}

LineSensor::~LineSensor() {

}

LineSensorCalibrator LineSensor::startCalibration() {
    return LineSensorCalibrator(*this);
}

float LineSensor::readLine(bool white_line, float noise_limit, float line_threshold) const {
    std::vector<uint16_t> vals;
    auto res = this->read(vals, false);
    if(res != ESP_OK) {
        ESP_LOGE(TAG, "read() failed: %d", res);
        return std::nanf("");
    }

    uint32_t weighted = 0;
    uint16_t sum = 0;

    const uint16_t noise = noise_limit * MAX_VAL;
    const uint16_t threshold = line_threshold * MAX_VAL;

    bool on_line = false;
    for(size_t i = 0; i < vals.size(); ++i) {
        auto val = vals[i];
        if(white_line)
            val = MAX_VAL - val;

        if(val < noise)
            continue;

        if(val >= threshold)
            on_line = true;

        weighted += uint32_t(val) * i * MAX_VAL;
        sum += val;
    }

    if(sum == 0 || !on_line)
        return std::nanf("");

    const int16_t middle = float(vals.size()-1)/2 * MAX_VAL;
    const int16_t result = (weighted / sum) - middle;
    return std::min(1.f, std::max(-1.f, float(result) / float(middle)));
}

bool LineSensor::setCalibration(const LineSensor::CalibrationData& data) {
    for(int i = 0; i < Driver::CHANNELS; ++i) {
        if(data.min[i] > Driver::MAX_VAL || data.range[i] > Driver::MAX_VAL ||
            data.min[i] + m_calibration.range[i] > Driver::MAX_VAL) {
            ESP_LOGE(TAG, "invalid data in setCalibration at channel %d, ignoring calibration!", i);
            return false;
        }
    }

    m_calibration = data;
    return true;
}

LineSensorCalibrator::LineSensorCalibrator(LineSensor& sensor) : m_sensor(sensor) {
    reset();
}

LineSensorCalibrator::~LineSensorCalibrator() {

}

void LineSensorCalibrator::reset() {
    for(int i = 0; i < Driver::CHANNELS; ++i) {
        m_data.min[i] = Driver::MAX_VAL;
        m_max[i] = 0;
    }
}

esp_err_t LineSensorCalibrator::record() {
    uint16_t vals[Driver::CHANNELS];
    esp_err_t res = m_sensor.read(vals);
    if(res != 0) {
        return res;
    }

    int idx = 0;
    const auto mask = m_sensor.getChannelsMask();
    for(int i = 0; i < Driver::CHANNELS; ++i) {
        if(((1 << i) & mask) == 0)
            continue;

        if(vals[idx] < m_data.min[i])
            m_data.min[i] = vals[idx];
        if(vals[idx] > m_max[i])
            m_max[i] = vals[idx];
        ++idx;
    }

    return ESP_OK;
}

void LineSensorCalibrator::save() {
    for(int i = 0; i < Driver::CHANNELS; ++i) {
        m_data.range[i] = m_max[i] - m_data.min[i];
    }
    m_sensor.setCalibration(m_data);
}

}; // namespace mcp3008
