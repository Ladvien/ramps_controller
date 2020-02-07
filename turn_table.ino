#include <avr/interrupt.h> 
#include <avr/io.h> 



// https://reprap.org/mediawiki/images/f/f6/RAMPS1.4schematic.png
// https://reprap.org/forum/read.php?219,168722

// TODO: Pulse width set by initialization.
// TODO: Setup all motors to be selected by master.
// TODO: Add a timer to shutdown motors after threshold.
//       And keep motor enabled until threshold has been met.
// TODO: Handle 0x0A values as part of packet (e.g., if MILLI_BETWEEN = 10).
// TODO: Add a "holding torque" feature; making it so motors never disable.

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

#define E_STEP_PIN         26
#define E_DIR_PIN          28
#define E_ENABLE_PIN       24

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

#define MOTOR_X         0x01
#define MOTOR_Y         0x02
#define MOTOR_Z         0x03
#define MOTOR_E1        0x04
#define MOTOR_E2        0x05

#define DRIVE_CMD       (char)0x01
#define HALT_CMD        (char)0x0F
#define DIR_CC          (char)0x00
#define DIR_CCW         (char)0x01

#define COMPLETED_CMD   (char)0x07
#define END_TX          (char)0x0A
#define ACK             (char)0x06 // Acknowledge
#define NACK            (char)0x15 // Negative Acknowledge


// Determine the pulse width of motor.
#define MOTOR_ANGLE           1.8
#define PULSE_WIDTH_MICROS    360 / MOTOR_ANGLE

#define RX_BUFFER_SIZE 16

/*
  MOTOR_NUM:
      X     = 0
      Y     = 1
      Z     = 2
      E1    = 3
      E2    = 4
      
  PACKET_TYPES
      0x01 = motor_write
      0x02 = motor_halt

  DIRECTION
      0x00 = CW
      0x01 = CCW

  MOTOR MOVE PROTOCOL:
                       0               1     2     3        4       5         6
  MOTOR_PACKET = PACKET_TYPE_CHAR MOTOR_NUM DIR STEPS_1 STEPS_2 MILLI_BETWEEN \n
  MOTOR_PACKET =    01                01    00    03     E8        05         0A
  MOTOR_PACKET =    0x 01010003E8050A

  HALT         = 0x0F
*/


/* Create a structure for the motors
 *  direction_pin = pin to control direction of stepper.
 *  step_pin      = pin to control the steps.
 *  enable_pin    = pin to enable motor.
 */
struct MOTOR {
  uint8_t direction_pin;
  uint8_t step_pin;
  uint8_t enable_pin;
  uint8_t pulse_width_micros;
};

struct BUFFER {
  uint8_t data[RX_BUFFER_SIZE];
  uint8_t bufferSize;
  uint8_t index;
  boolean packetComplete;
  uint8_t shutdownThreshold;
};

/* Initialize motors */
MOTOR motorX = {
      X_DIR_PIN,
      X_STEP_PIN,
      X_ENABLE_PIN,
      PULSE_WIDTH_MICROS
};

// Urgent shutdown.
volatile boolean halt = false;
volatile static bool triggered;

/* Initialize RX buffer */
BUFFER rxBuffer;;

/* Initialize program */
void setup()
{
  // Greetings.
  Serial.begin(115200);
  greetings();
  
  // Initialize the structures
  motorSetup(motorX);
  rxBuffer.bufferSize = RX_BUFFER_SIZE;

  // Disable holding torque.
  digitalWrite(motorX.enable_pin, HIGH);
}

/* Main */
void loop()
{
  // If packet is packetComplete
  if (rxBuffer.packetComplete) {
    
    uint8_t packet_type = rxBuffer.data[0];

    switch (packet_type) {
      case DRIVE_CMD:
        {
          // Unpack the command.
          uint8_t motorNumber =  rxBuffer.data[1];
          uint8_t direction =  rxBuffer.data[2];
          uint16_t steps = ((uint8_t)rxBuffer.data[3] << 8)  | (uint8_t)rxBuffer.data[4];
          uint8_t milliSecondsDelay = rxBuffer.data[5];

          // Let the master know command is in process.
          sendAck();
  
          // Start the motor
          writeMotor(motorX, direction, steps, milliSecondsDelay);
        }
        break;
      default:
        sendNack();
        break;
    }
    // Clear the buffer for the nexgt packet.
    resetBuffer(&rxBuffer);
  }
}


void greetings() {
  Serial.println("RAMPs 1.4 stepper driver.");
  Serial.println("  MOTOR_NUM:");
  Serial.println("      X     = 0");
  Serial.println("      Y     = 1");
  Serial.println("      Z     = 2");
  Serial.println("      E1    = 3");
  Serial.println("      E2    = 4");
  Serial.println("      ");
  Serial.println("  PACKET_TYPES");
  Serial.println("      0x01 = motor_write");
  Serial.println("");
  Serial.println("  DIRECTION");
  Serial.println("      0x00 = CW");
  Serial.println("      0x01 = CCW");
  Serial.println("");
  Serial.println("  MOTOR MOVE PROTOCOL:");
  Serial.println("                       0               1     2     3        4       5         6");
  Serial.println("  MOTOR_PACKET = PACKET_TYPE_CHAR MOTOR_NUM DIR STEPS_1 STEPS_2 MILLI_BETWEEN \\n");
  Serial.println("  MOTOR_PACKET =    01                01    00    03     E8        05         0A");
  Serial.println("  MOTOR_PACKET =    0x 01010003E8050A");
  Serial.println("");
  Serial.println("  HALT         = 0x0F");
}


/*  ############### MOTORS ###############
 * 
*/

/* Method for initalizing MOTOR */
void motorSetup(MOTOR motor) {

  // Setup motor pins
  pinMode(motor.direction_pin, OUTPUT);
  pinMode(motor.step_pin, OUTPUT);
  pinMode(motor.enable_pin, OUTPUT);

}

/* Write to MOTOR */
void writeMotor(MOTOR motor, int direction, uint16_t numberOfSteps, int milliBetweenSteps) {

    // Enable motor.
    digitalWrite(motor.enable_pin, LOW);

    // Check direction;
    switch (direction) {
      case DIR_CC:
        digitalWrite(motor.direction_pin, HIGH);
        break;
      case DIR_CCW:
        digitalWrite(motor.direction_pin, LOW);
        break;
      default:
        sendNack();
        return;
    }

    // Move the motor (but keep an eye for a halt command)
    for(int n = 0; n < numberOfSteps; n++) {
      // Interrupt motor
      if(checkForHalt()) {  
        sendAck();
        break; 
      }
      digitalWrite(motor.step_pin, HIGH);
      delayMicroseconds(motor.pulse_width_micros);
      digitalWrite(motor.step_pin, LOW);
      delay(milliBetweenSteps);
    }

    // Disable holding torque.
    digitalWrite(motor.enable_pin, HIGH);

    // Let the user know the move is done.
    sendCompletedAction();
}

// END MOTORS

/*  ############### COMMUNICATION ###############
 * 
*/
void serialEvent() {

  // Get all the data.
  while (Serial.available()) {

    // Read a byte
    uint8_t inByte = (uint8_t)Serial.read();

    // Store the byte in the buffer.
    rxBuffer.data[rxBuffer.index] = inByte;
    rxBuffer.index++;

    // If a complete packet character is found, mark the packet
    // as ready for execution.
    if ((char)inByte == '\n') {
      rxBuffer.packetComplete = true;
    }
    
  }
}

// Clear the buffer.
void resetBuffer(struct BUFFER *buffer) {
  memset(buffer->data, 0, sizeof(buffer->data));
  buffer->index = 0;
  buffer->packetComplete = false;
}

// Does not count termination char.
int packetLength(BUFFER buffer){
  for(int i = 0; i < buffer.bufferSize; i++) {
    if((char)buffer.data[i] == '\n'){ return i; }
  }
  return -1;
}

void sendAck() {
  Serial.write(ACK);
  Serial.write(END_TX);
}

void sendNack() {
  Serial.write(ACK);
  Serial.write(END_TX);
}

void sendCompletedAction() {
  Serial.write(COMPLETED_CMD);
  Serial.write(END_TX);
}

// Halt is handled outside normal communication protocol.
boolean checkForHalt() {
  if (Serial.available()){
    // Halt command has no termination character.
    if ((uint8_t)Serial.read() == HALT_CMD) {
      return true;
    }
  }
  return false;
}

// END COMMUNICATION
