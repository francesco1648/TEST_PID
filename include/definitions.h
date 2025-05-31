#ifndef definitions_h
#define definitions_h

// Main loop timings
#define DT_BAT 1000   // 1000ms / 1000 = 1 Hz
#define DT_PID 10     // 1000ms / 100   = 100 Hz
#define DT_TEL 10     // 1000ms / 10   = 100 Hz
#define DT_ENC 20     // 1000ms / 20   = 50 Hz
#define DT_MOTOR_CURR 10 // 1000ms / 10 = 100 Hz
#define DT_MOTOR_TEMP 1000 // 1000ms / 1000 = 1 Hz

// Current and temperature thresholds configurations
#define MAX_CURR 5.0f // 5A
#define MAX_TEMP 50.0f // 50°C
#define MAX_CURR_READINGS 100 // 100 readings
#define MAX_TEMP_READINGS 5 // 5 readings

// PWM configuration
#define PWM_MAX_VALUE 1023
#define PWM_FREQUENCY 15000

// I²C configuration
#define I2C_SENS_SDA   18
#define I2C_SENS_SCL   19
#define I2C_ADC_SDA   2
#define I2C_ADC_SCL   3

#define ABSOLUTE_ENCODER_ADDRESS 0x40
//
// Timeouts
#define CAN_TIMEOUT 1000
// Battery configuration
#define BAT_LOW 11.1f
#define BAT_NOM 12.6f
#define BAT_PIN 28
#define BAT_R1 10000
#define BAT_R2 3300

// Motors pins
#define DRV_TR_LEFT_DIR  15
#define DRV_TR_LEFT_PWM 14

#define DRV_TR_RIGHT_DIR  9
#define DRV_TR_RIGHT_PWM  8


// Motor configuration
#define MAX_SPEED 65.f

// Encoder pins
#define ENC_TR_LEFT_A   12
#define ENC_TR_LEFT_B   13

#define ENC_TR_RIGHT_A  10
#define ENC_TR_RIGHT_B  11

// ADC configuration
#define ADC_BASE_CHANNEL_LEFT 0
#define ADC_BASE_CHANNEL_RIGHT 2
#define ADC_ADDR 0x49  // 0x49 to be specified because currently the addr pin is connected to 3.3 V


// Encoder conversion constant
// K = 100          *       10^6        *       60      / (     48   *             74,83                    *       2 )
//      centiRPM           microsecToSec        SecToMin       intPerRotation        transmissionRatio             transmissionRatio2
#define ENC_TR_CONVERSION (835226)

// Traction encoder filter samples
#define ENC_TR_SAMPLES 10

// Display
#define DISPLAY_ADDR 0x3c
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64

// Interface definitions
#define NMENUS 4
#define MENUTIMEOUT 18
#define BTNOK 16
#define BTNNAV 17
#define DEBOUNCE 300

// Versioning
#ifndef VERSION
  #define VERSION "testing"
#endif

// OTA configuration
#define OTA_PWD "ciaociao"
#define WIFI_SSID "iswifi"
#define WIFI_PWD "ciaociao"
#define WIFI_HOSTBASE "picow-"
#define CONF_PATH "/config.txt"

#endif
