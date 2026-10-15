#include <stdio.h>
#include "esp_log.h"
#include "lidar.h"

static const char* TAG = "System";
extern "C" void app_main(void)
{
    ESP_LOGI(TAG,"Online");
    static Lidar lidar;

    if(!lidar.init(UART_NUM_1,GPIO_NUM_27,GPIO_NUM_26))
    {
        ESP_LOGE(TAG,"Lidar init failed");
        return;
    }

    ESP_LOGI(TAG,"Lidar initialized");
    lidar.setMotorSpeed(600);
    while(true)
    {
        if(lidar.HAS_NEW_DATA) 
        { 
            lidar.getMeasurements();
            for(int i = 0; i<360; i++)
            {
                printf("SCAN,%d,%u,%u\n",
                i,
                lidar.scan_data[i].distance,
                lidar.scan_data[i].quality);
            }

        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
