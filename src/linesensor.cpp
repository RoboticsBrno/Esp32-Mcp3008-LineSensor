#include <cmath>
#include <esp_log.h>

#include "linesensor.h"

#define TAG "Mcp3008LineSensor"

namespace mcp3008 {

LineSensor::LineSensor() : m_installed(false), m_spi(NULL), m_spi_dev(HSPI_HOST) {

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

esp_err_t LineSensor::read(std::vector<uint16_t>& results, bool differential, uint8_t channels_mask) {
    int requested = 0;
    for(int i = 0; i < CHANNELS; ++i) {
        if(((1 << i) & channels_mask) != 0)
            ++requested;
    }

    const size_t orig_size = results.size();
    results.resize(orig_size + requested);

    esp_err_t res = this->read(results.data() + orig_size, differential, channels_mask);
    if(res != ESP_OK) {
        results.resize(orig_size);
        return res;
    }
    return ESP_OK;
}

esp_err_t LineSensor::read(uint16_t *dest, bool differential, uint8_t channels_mask) {
    if(!m_installed)
        return ESP_FAIL;

    int requested = 0;
    spi_transaction_t transactions[CHANNELS] = { 0 };
    for(int i = 0; i < CHANNELS; ++i) {
        if(((1 << i) & channels_mask) == 0)
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
        dest[idx] = ((trans->rx_data[1] & 0x03) << 8) | trans->rx_data[2];
    }
    return ESP_OK;
}

float LineSensor::readLine(bool white_line, uint8_t channels_mask, float noise_limit, float line_threshold) {
    std::vector<uint16_t> vals;
    auto res = this->read(vals, false, channels_mask);
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

}; // namespace mcp3008
