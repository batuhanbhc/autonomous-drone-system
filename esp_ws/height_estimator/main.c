#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void) {
    while (1) {
        printf("SENSOR:%.2f,%.2f,%.2f\n", 1.0, 1.0, 1.0);
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}