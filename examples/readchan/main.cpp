#include "mcp3008_linesensor.h"
#include <Arduino.h>

using namespace mcp3008;

void setup() {
    LineSensor ls;

    ESP_ERROR_CHECK(ls.install());

    for (int i = 0; i < Driver::CHANNELS; ++i) {
        printf("%d: %hu\n", i, ls.readChannel(i));
    }
}

void loop() {
}
