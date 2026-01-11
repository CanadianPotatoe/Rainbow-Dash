#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "TMC429.h"

// --- Hardware Configuration ---
#define TMC429_CS_PIN 5
#define TMC429_SCK_PIN 2
#define TMC429_SDI_PIN 3    
#define TMC429_SDO_PIN 4    
#define CLOCK_GEN_PIN 6     
#define DRIVER_EN_PIN 7      // GPIO 7 connected to TMC2209 EN pin
#define MOTOR_INDEX 0       

void start_tmc429_clock(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice_num, 7);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 4);
    pwm_set_enabled(slice_num, true);
}

int hardware_init(){
// 1. Initialize Driver Enable Pin
    printf("Initializing Driver Enable on GPIO %d...\n", DRIVER_EN_PIN);
    gpio_init(DRIVER_EN_PIN);
    gpio_set_dir(DRIVER_EN_PIN, GPIO_OUT);
    gpio_put(DRIVER_EN_PIN, 0); // Set LOW to enable motor initially

    // 2. Start TMC429 Clock and SPI
    start_tmc429_clock(CLOCK_GEN_PIN); 
    spi_init(spi0, 1000000);
    gpio_set_function(TMC429_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(TMC429_SDI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(TMC429_SDO_PIN, GPIO_FUNC_SPI);
    gpio_init(TMC429_CS_PIN);
    gpio_set_dir(TMC429_CS_PIN, GPIO_OUT);
    gpio_put(TMC429_CS_PIN, 1);

}

int main() {
    stdio_init_all();
    sleep_ms(2000); 
    hardware_init();
    
    TMC429 IMU_Stepper; 
    IMU_Stepper.setup(TMC429_CS_PIN, 16, spi0);

    // 3. Detection and Config
    while (!IMU_Stepper.communicating()) {
        printf("Detecting TMC429...\n");
        sleep_ms(1000);
    }

    IMU_Stepper.setLimitsInHz(MOTOR_INDEX, 200, 2000, 5000);
    IMU_Stepper.setRampMode(MOTOR_INDEX);
    IMU_Stepper.disableLeftSwitchStop(MOTOR_INDEX);
    IMU_Stepper.disableRightSwitchStop(MOTOR_INDEX);
    IMU_Stepper.setActualPosition(MOTOR_INDEX, 0);
    // 4. Main Movement Loop (Limited to 5 iterations)
    printf("Starting 5-cycle test...\n");

    for (int cycle = 1; cycle <= 5; cycle++) {
        printf("\n--- Cycle %d of 5 ---\n", cycle);

        // Forward to 5000
        IMU_Stepper.setTargetPosition(MOTOR_INDEX, 5000);
        while (!IMU_Stepper.atTargetPosition(MOTOR_INDEX)) {
            printf("Pos: %ld | Vel: %ld Hz\r", 
                   IMU_Stepper.getActualPosition(MOTOR_INDEX), 
                   IMU_Stepper.getActualVelocityInHz(MOTOR_INDEX));
            sleep_ms(100);
        }
        sleep_ms(1000);

        // Backward to -5000
        IMU_Stepper.setTargetPosition(MOTOR_INDEX, -5000);
        while (!IMU_Stepper.atTargetPosition(MOTOR_INDEX)) {
            printf("Pos: %ld | Vel: %ld Hz\r", 
                   IMU_Stepper.getActualPosition(MOTOR_INDEX), 
                   IMU_Stepper.getActualVelocityInHz(MOTOR_INDEX));
            sleep_ms(100);
        }
        sleep_ms(1000);
    }

    // 5. Shutdown Logic
    printf("\n\nTest complete. Shutting down...\n");
    
    // Stop the TMC429 from sending pulses
    IMU_Stepper.stop(MOTOR_INDEX); 
    
    // Disable the driver power (GPIO 7 HIGH)
    gpio_put(DRIVER_EN_PIN, 1); 
    
    printf("Motor de-energized. It is now safe to unplug the battery.\n");

    while(1) { sleep_ms(1000); } // Prevent main from exiting
    return 0;
}