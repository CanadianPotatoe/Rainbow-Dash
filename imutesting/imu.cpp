#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bno08x.h"
#include "TMC429.h" 

int main() {
    stdio_init_all();
    i2c_inst_t* i2c_port = i2c1;
    i2c_init(i2c_port, 400000);
    gpio_set_function(11, GPIO_FUNC_I2C);
    gpio_set_function(10, GPIO_FUNC_I2C);
    gpio_pull_up(10);
    gpio_pull_up(11);

    BNO08x IMU;

    const uint8_t addr = 0x4A;   // Default BNO08x I2C address

    while (!IMU.begin(addr, i2c_port)) {
        printf("BNO08x not detected. Check wiring.\n");
        sleep_ms(1000);
    }

    IMU.enableGameRotationVector();

    while (true) {

        float yaw = 0.0f;

        if (IMU.getSensorEvent()) {
            if (IMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
                yaw = IMU.getYaw();
            }
        }

        printf("Yaw: %.4f rad\n", yaw);
        sleep_ms(50);
    }
}
