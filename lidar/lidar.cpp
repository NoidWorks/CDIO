#include "lidar.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

bool Lidar::HAS_NEW_DATA = false;
std::array<Point, 360> Lidar::scan_data{};
static const char* TAG = "Lidar";


bool Lidar::init(uart_port_t uart_num, gpio_num_t tx, gpio_num_t rx)
{
    PORT = uart_num;
    uart_config_t config = 
    {
        .baud_rate = 460800,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    esp_err_t err = uart_param_config(
        PORT,&config
    );

    if (err != ESP_OK) return false;

    err = uart_set_pin(
        PORT,
        tx,
        rx,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );

    if(err != ESP_OK) return false;

        err = uart_driver_install(
        PORT,
        2048,   // RX buffer
        2048,   // TX buffer
        0,
        nullptr,
        0
    );

    if (err != ESP_OK)
        return false;

    ESP_LOGI(TAG,"UART initialized, sending boot commands...");
        // Boot sequence
    sendCommand(0x25);
    vTaskDelay(pdMS_TO_TICKS(3000));

    sendCommand(0x40);
    vTaskDelay(pdMS_TO_TICKS(5000));
    setMotorSpeed(600);
    vTaskDelay(pdMS_TO_TICKS(500));

    sendCommand(0x20);
    vTaskDelay(pdMS_TO_TICKS(200));

    // Clear any old data in RX buffer
    uart_flush_input(PORT);
    ESP_LOGI(TAG,"Start sequene done, flushing RX buffer...");
    ESP_LOGI(TAG,"Creating sub-task...");
    BaseType_t result = xTaskCreate(
        Lidar::lidarTask,
        "lidar_task",
        4096,
        this,
        5,
        nullptr
    );

    if(result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create lidar sub-task.");
        return false;
    }

    ESP_LOGI(TAG, "Sub-task created");
    ESP_LOGW(TAG,"Motor set to 600rpm, warm up for 2min for best results");
    return true;
}

void Lidar::sendCommand(uint8_t cmd)
{
    uint8_t message[] = {
        0xA5,
        cmd
    };

    uart_write_bytes(
        PORT,
        message,
        sizeof(message)
    );
}

void Lidar::setMotorSpeed(uint16_t rpm)
{
    uint8_t lsb = rpm & 0xFF;
    uint8_t msb = (rpm>>8) & 0xFF;

    uint8_t checksum = 0xA5^0xA8^0x02^lsb^msb;

    uint8_t packet[]=
    {
        0xA5,
        0xA8,
        0x02,
        lsb,
        msb,
        checksum
    };

    uart_write_bytes(
        PORT,
        packet,
        sizeof(packet)
    );
}

void Lidar::takeMeasurements()
{
    uint8_t data[5];

    int bytesRead = uart_read_bytes(
        PORT,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100)
    );

    if(bytesRead != sizeof(data)) return;

    //parsing 4bit Q, 2byte angle 2byte distance
    uint8_t quality = data[0] >> 2;
    if(quality < q_limit) return;

    uint16_t angle = (((data[2]<<8)|data[1]))/64;
    if(angle >= 360) return;

    //uint16_t distance = ((data[4]<<8)|data[3])/4;

    measurements[angle] = 
    { 
        .distance =  uint16_t(((data[4]<<8)|data[3])/4), 
        .quality = quality
    };

    HAS_NEW_DATA = true;
}

void Lidar::lidarTask(void* param)
{
    auto* lidar = static_cast<Lidar*>(param);

    while(true)
    {
        lidar->takeMeasurements();

        vTaskDelay(pdMS_TO_TICKS(lidar->scan_delayMS));
    }
}

