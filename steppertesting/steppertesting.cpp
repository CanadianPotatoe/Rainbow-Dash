#include <stdio.h>
#include <cmath>
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
#define DRIVER1_EN_PIN 7 
#define DRIVER2_EN_PIN 8
#define START_BUTTON_PIN 11
#define MOTOR_INDEXA 0 
#define MOTOR_INDEXB 1      

const float STEPS_PER_CM = 100.32; 
const float TRACK_WIDTH_CM = 19.3; 

TMC429 Stepper;

void start_tmc429_clock(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice_num, 7);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 4);
    pwm_set_enabled(slice_num, true);
}

void hardware_init() {
    printf("[INIT] Hardware Initialization...\n");
    gpio_init(DRIVER1_EN_PIN);
    gpio_init(DRIVER2_EN_PIN);
    gpio_set_dir(DRIVER1_EN_PIN, GPIO_OUT);
    gpio_set_dir(DRIVER2_EN_PIN, GPIO_OUT);
    // Disable motors initially
    gpio_put(DRIVER1_EN_PIN, 1); 
    gpio_put(DRIVER2_EN_PIN, 1);
    gpio_init(START_BUTTON_PIN);
    gpio_set_dir(START_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(START_BUTTON_PIN);
    start_tmc429_clock(CLOCK_GEN_PIN); 
    spi_init(spi0, 1000000);
    gpio_set_function(TMC429_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(TMC429_SDI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(TMC429_SDO_PIN, GPIO_FUNC_SPI);
    gpio_init(TMC429_CS_PIN);
    gpio_set_dir(TMC429_CS_PIN, GPIO_OUT);
    gpio_put(TMC429_CS_PIN, 1);
    Stepper.setup(TMC429_CS_PIN, 16, spi0);
}

void forward(int distance_cm) { 
    long target_steps = (long)round(distance_cm * STEPS_PER_CM);
    printf("[RUN] Forward %d cm (%ld steps)\n", distance_cm, target_steps);

    // Reset positions to 0
    Stepper.setActualPosition(MOTOR_INDEXA, 0);
    Stepper.setActualPosition(MOTOR_INDEXB, 0);
    
    // Set targets
    Stepper.setTargetPosition(MOTOR_INDEXA, target_steps);
    Stepper.setTargetPosition(MOTOR_INDEXB, target_steps);

    // Wait for completion
    while (!Stepper.atTargetPosition(MOTOR_INDEXA) || !Stepper.atTargetPosition(MOTOR_INDEXB)) {
        sleep_ms(10);
    }
    printf("[OK] Forward complete\n");
    sleep_ms(200); 
}

void left() {
    float turn_dist = (TRACK_WIDTH_CM * 3.14159) / 4.0;
    long steps = (long)round(turn_dist * STEPS_PER_CM);

    // 3. Reset positions
    Stepper.setActualPosition(MOTOR_INDEXA, 0);
    Stepper.setActualPosition(MOTOR_INDEXB, 0);
    sleep_ms(50); // Give the TMC429 time to register the reset

    // 4. Set new targets
    Stepper.setTargetPosition(MOTOR_INDEXA, -steps);
    Stepper.setTargetPosition(MOTOR_INDEXB, steps);

    // 5. Use a safer polling loop
    while (true) {
        bool a_done = Stepper.atTargetPosition(MOTOR_INDEXA);
        bool b_done = Stepper.atTargetPosition(MOTOR_INDEXB);
        if (a_done && b_done) break;
        sleep_ms(10);
    }
}

void right() {
    float turn_dist = (TRACK_WIDTH_CM * 3.14159) / 4.0;
    long target_steps = (long)round(turn_dist * STEPS_PER_CM);
    printf("[RUN] Right Turn 90 deg (%ld steps)\n", target_steps);

    // Reset positions to 0
    Stepper.setActualPosition(MOTOR_INDEXA, 0);
    Stepper.setActualPosition(MOTOR_INDEXB, 0);
    
    // Set targets (Opposite directions for pivot)
    Stepper.setTargetPosition(MOTOR_INDEXA, target_steps);
    Stepper.setTargetPosition(MOTOR_INDEXB, -target_steps);

    while (!Stepper.atTargetPosition(MOTOR_INDEXA) || !Stepper.atTargetPosition(MOTOR_INDEXB)) {
        sleep_ms(10);
    }
    printf("[OK] Right turn complete\n");
    sleep_ms(200);
}

int main() {
    stdio_init_all();
    hardware_init();
    
    while (!Stepper.communicating()) { 
        printf("Waiting for TMC429...\n");
        sleep_ms(500); 
    }
    
    Stepper.setLimitsInHz(MOTOR_INDEXA, 200, 2000, 5000); 
    Stepper.setLimitsInHz(MOTOR_INDEXB, 200, 2000, 5000);
    Stepper.setRampMode(MOTOR_INDEXA);
    Stepper.setRampMode(MOTOR_INDEXB);

    printf("\nREADY: Press Button (GPIO 11) to Start Run\n");

    while (true) {
        // 1. Wait for the pin to go LOW (button pressed)
        if (gpio_get(START_BUTTON_PIN) == 0) {
            
            // 2. Debounce delay (20-50ms is standard)
            sleep_ms(50); 
            
            // 3. Check again: Is it STILL pressed?
            if (gpio_get(START_BUTTON_PIN) == 0) {
                // If yes, break out of the loop and start the robot
                break; 
            }
        }
        sleep_ms(10); 
    }

    printf("[START] Button confirmed. Enabling motors...\n");


    gpio_put(DRIVER1_EN_PIN, 0); // Enable Motor 1
    sleep_ms(250);               // Let the power stabilize

    gpio_put(DRIVER2_EN_PIN, 0); // Enable Motor 2
    sleep_ms(250);               // Let the power stabilize

    // --- EXECUTION ---
    left();

    printf("[FINISH] Sequence complete. Powering down.\n");
    gpio_put(DRIVER1_EN_PIN, 1); 
    gpio_put(DRIVER2_EN_PIN, 1);
    while(1) {
        tight_loop_contents(); // Keep the CPU alive but idle
    }
}
