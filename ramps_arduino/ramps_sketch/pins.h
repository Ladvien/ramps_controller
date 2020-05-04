// For RAMPS 1.4
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN           2

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

#define E0_STEP_PIN         26
#define E0_DIR_PIN          28
#define E0_ENABLE_PIN       24

#define E0_STEP_PIN         36
#define E0_DIR_PIN          34
#define E0_ENABLE_PIN       30

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13

#define FAN_PIN            9

#define PS_ON_PIN          12
#define KILL_PIN           -1

#define HEATER_0_PIN       10
#define HEATER_1_PIN       8
#define TEMP_0_PIN         13   // ANALOG NUMBERING
#define TEMP_1_PIN         14   // ANALOG NUMBERING

#define MOTOR_X         0x00
#define MOTOR_Y         0x01
#define MOTOR_Z         0x02
#define MOTOR_E0        0x03
#define MOTOR_E1        0x04

#define DRIVE_CMD       (char)0x01
#define HALT_CMD        (char)0x0F
#define DIR_CC          (char)0x00
#define DIR_CCW         (char)0x01

#define COMPLETED_CMD   (char)0x07
#define END_TX          (char)0x04
#define ACK             (char)0x06 // Acknowledge
#define NACK            (char)0x15 // Negative Acknowledge