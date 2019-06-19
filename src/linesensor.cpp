#include <cmath>
#include <esp_log.h>

#include "linesensor.h"

#define TAG "Mcp3008LineSensor"

namespace mcp3008 {

LineSensor::LineSensor() : m_spi(NULL), m_spi_dev(HSPI_HOST), m_installed(false), m_channels_mask(0xFF) {
}

LineSensor::~LineSensor() {
    uninstall();
}

esp_err_t LineSensor::install(const LineSensor::Config& cfg) {
    if(m_installed)
        return ESP_OK;

    esp_err_t ret;
    spi_bus_config_t buscfg = { 0 };
    buscfg.miso_io_num = cfg.pin_miso;
    buscfg.mosi_io_num = cfg.pin_mosi;
    buscfg.sclk_io_num = cfg.pin_sck;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    spi_device_interface_config_t devcfg = { 0 };
    devcfg.clock_speed_hz = cfg.freq;
    devcfg.mode = 0;
    devcfg.spics_io_num = cfg.pin_cs;
    devcfg.queue_size = CHANNELS;

    ret = spi_bus_initialize(cfg.spi_dev, &buscfg, 1);
    if(ret != ESP_OK) {
        return ret;
    }

    ret = spi_bus_add_device(cfg.spi_dev, &devcfg, &m_spi);
    if(ret != ESP_OK) {
        spi_bus_free(cfg.spi_dev);
        return ret;
    }

    m_spi_dev = cfg.spi_dev;
    m_channels_mask = cfg.channels_mask;
    m_installed = true;
    return ESP_OK;
}

esp_err_t LineSensor::uninstall() {
    if(!m_installed)
        return ESP_OK;

    esp_err_t res = spi_bus_remove_device(m_spi);
    if(res != ESP_OK)
        return res;

    res = spi_bus_free(m_spi_dev);
    if(res != ESP_OK)
        return res;

    m_installed = false;
    return ESP_OK;
}

LineSensorCalibrator LineSensor::startCalibration() {
    return LineSensorCalibrator(*this);
}

int LineSensor::requestToChannel(int request) const {
    if(m_channels_mask == 0xFF)
        return request;

    int requested = 0;
    for(int i = 0; i < CHANNELS; ++i) {
        if(((1 << i) & m_channels_mask) != 0) {
            if(requested++ == request)
                return i;
        }
    }

    ESP_LOGE(TAG, "Invalid requestToChannel call %d", request);
    return 0;
}

esp_err_t LineSensor::read(std::vector<uint16_t>& results, bool useCalibration, bool differential) const {
    int requested = 0;
    for(int i = 0; i < CHANNELS; ++i) {
        if(((1 << i) & m_channels_mask) != 0)
            ++requested;
    }

    const size_t orig_size = results.size();
    results.resize(orig_size + requested);

    esp_err_t res = this->read(results.data() + orig_size, useCalibration, differential);
    if(res != ESP_OK) {
        results.resize(orig_size);
        return res;
    }
    return ESP_OK;
}

esp_err_t LineSensor::read(uint16_t *dest, bool useCalibration, bool differential) const {
    if(!m_installed)
        return ESP_FAIL;

    int requested = 0;
    spi_transaction_t transactions[CHANNELS] = { 0 };
    for(int i = 0; i < CHANNELS; ++i) {
        if(((1 << i) & m_channels_mask) == 0)
            continue;

        auto &t = transactions[i];
        t.user = (void*)requested;
        t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
        t.length = 3 * 8;
        t.tx_data[0] = 1;
        t.tx_data[1] = (!differential << 7) | ((i & 0x07) << 4);

        esp_err_t res = spi_device_queue_trans(m_spi, &transactions[i], 100);
        if(res != ESP_OK)
            return res;
        ++requested;
    }

    if(requested == 0)
        return ESP_OK;

    spi_transaction_t *trans = NULL;
    for(int i = 0; i < requested; ++i) {
        esp_err_t res = spi_device_get_trans_result(m_spi, &trans, portMAX_DELAY);
        if(res != ESP_OK) {
            return res;
        }

        const int idx = (int)trans->user;
        uint16_t val = ((trans->rx_data[1] & 0x03) << 8) | trans->rx_data[2];
        if(useCalibration) {
            const int chan = requestToChannel(i);
            if(val <= m_calibration.min[chan]) {
                val = 0;
            } else {
                val = int32_t(val - m_calibration.min[chan]) * MAX_VAL / m_calibration.range[chan];
                val = std::min(MAX_VAL, val);
            }
        }
        dest[idx] = val;
    }
    return ESP_OK;
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


LineSensorCalibrator::LineSensorCalibrator(LineSensor& sensor) : m_sensor(sensor) {
    reset();
}

LineSensorCalibrator::~LineSensorCalibrator() {

}

void LineSensorCalibrator::reset() {
    for(int i = 0; i < LineSensor::CHANNELS; ++i) {
        m_data.min[i] = LineSensor::MAX_VAL;
        m_max[i] = 0;
    }
}

esp_err_t LineSensorCalibrator::record() {
    uint16_t vals[LineSensor::CHANNELS];
    esp_err_t res = m_sensor.read(vals);
    if(res != 0) {
        return res;
    }

    int idx = 0;
    const auto mask = m_sensor.getChannelsMask();
    for(int i = 0; i < LineSensor::CHANNELS; ++i) {
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
    for(int i = 0; i < LineSensor::CHANNELS; ++i) {
        m_data.range[i] = m_max[i] - m_data.min[i];
    }
    m_sensor.setCalibration(m_data);
}

}; // namespace mcp3008
