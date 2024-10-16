/* Blink Example

This example code is in the Public Domain (or CC0 licensed, at your option.)

Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.
*/

#include "Arduino.h"
#include "Wire.h"
#include "SimpleFOC.h"
#include "Sparkfun_TMAG5273_Arduino_Library.h"

extern "C"
{
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"
}

extern "C"
{
void app_main(void);    
}


#define UH 16
#define UL 17
#define VH 18
#define VL 23
#define WH 19
#define WL 33

BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(UH, UL, VH, VL, WH, WL);

// Create a new sensor object
TMAG5273 sensor; 

// I2C default address
uint8_t i2cAddress = TMAG5273_I2C_ADDRESS_INITIAL;

// Set constants for setting up device
uint8_t conversionAverage = TMAG5273_X32_CONVERSION;
uint8_t magneticChannel = TMAG5273_XYX_ENABLE;
uint8_t angleCalculation = TMAG5273_XY_ANGLE_CALCULATION;

float current_angle = 0.0;
float target_speed = 5.0;

void TMAG5273_Setup(void)
{
    Serial.begin(115200);
    Serial.print("Initializing sensor....");
    Wire.begin();
    while (!sensor.begin(i2cAddress, Wire)) {
        Serial.print(".");
    }
    sensor.setConvAvg(conversionAverage);
    sensor.setMagneticChannel(magneticChannel);
    sensor.setAngleEn(angleCalculation);
    Serial.println("initialized");
}

void BLDC_Setup(void)
{
    Serial.print("Intiailizing motor....");
    driver.voltage_power_supply = 3.3;
    driver.pwm_frequency = 20000;
    driver.voltage_limit = 4.0;
    driver.init();
    motor.linkDriver(&driver);
    motor.voltage_limit = 4.0;
    motor.controller = MotionControlType::velocity_openloop;
    motor.init();
    motor.disable();
    vTaskDelay(1000/portTICK_PERIOD_MS);
    motor.enable();
    Serial.println("intiailized....");
}

void Read_TMAG5273(void *pvParameters)
{
    while (1) {
        if ((sensor.getMagneticChannel() != 0) && (sensor.getAngleEn() != 0)) {
            current_angle = sensor.getAngleResult();
            Serial.printf("Current Angle: %.2f°\n", current_angle);
        } else {
            Serial.println("Mag Channels disabled, stopping..");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void Control_Motor(void* pvParameters)
{
    while (1) {
        motor.move(target_speed);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    TMAG5273_Setup();
    BLDC_Setup();
    xTaskCreate(Read_TMAG5273,"Read TMAG5273",1024*2,NULL,2,NULL);
    xTaskCreate(Control_Motor,"Control Motor",1024*2,NULL,4,NULL);
}
