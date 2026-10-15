#ifndef LIDAR_H
#define LIDAR_H

#include <array>
#include <cstdint>
#include "driver/gpio.h"
#include "driver/uart.h"

struct Point
{
    uint16_t distance = 0;
    uint8_t quality = 0;
};

class Lidar
{
    public:
    bool init(uart_port_t uart_num, gpio_num_t tx, gpio_num_t rx);
    void setMotorSpeed(uint16_t rpm);
    void pause(); //turns of lidar systems
    void resume();//boot up with existing settings
    int scan_delayMS = 5;
    int q_limit = 2;
    static bool HAS_NEW_DATA;
    static std::array<Point,360> scan_data;

    void getMeasurements()
    {
        HAS_NEW_DATA = false;
        scan_data = measurements;
    }

    private:

    uart_port_t PORT;
    std::array<Point, 360> measurements;

    void sendCommand(uint8_t cmd);
    void takeMeasurements();

    static void lidarTask(void* param);
};

#endif