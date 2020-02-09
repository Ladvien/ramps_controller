#include <avr/interrupt.h> 
#include <avr/io.h> 
#include "pins.h"


// https://reprap.org/mediawiki/images/f/f6/RAMPS1.4schematic.png
// https://reprap.org/forum/read.php?219,168722

// TODO: Remove unneeded "MOTOR_NUM", use packet index to determine motor.
// TODO: Pulse width set by initialization.
// TODO: Add a timer to shutdown motors after threshold.
//       And keep motor enabled until threshold has been met.
// TODO: Add a "holding torque" feature; making it so motors never disable.


// Determine the pulse width of motor.
#define MOTOR_ANGLE           1.8
#define PULSE_WIDTH_MICROS    360 / MOTOR_ANGLE
#define NUM_MOTORS            5
#define PACKET_LENGTH         6
#define RX_BUFFER_SIZE        NUM_MOTORS * PACKET_LENGTH
#define PACKAGE_SIZE          NUM_MOTORS * PACKET_LENGTH


/*
  MOTOR_NUM:
      X     = 0
      Y     = 1
      Z     = 2
      E0    = 3
      E1    = 4
      
  PACKET_TYPES
      0x01 = motor_write
      0x02 = motor_halt

  DIRECTION
      0x00 = CW
      0x01 = CCW

  MOTOR MOVE PROTOCOL:
                       0               1     2     3        4       5        
  MOTOR_PACKET = PACKET_TYPE_CHAR MOTOR_NUM DIR STEPS_1 STEPS_2 MICROSECONDS_BETWEEN
  MOTOR_PACKET =    01                01    00    03     E8        05        
  MOTOR_PACKET =    0x 01010003E8050A

  HALT         = 0x0F

  PACKAGE = PACKET1 PACKET2 PACKET3 PACKET4 PACKET5
  
  PACKAGE_EXAMPLE = 01 01 00 FF E8 01 01 02 00 FF E8 01 01 02 00 FF E8 01 01 02 00 FF E8 01 01 02 00 FF E8 01
*/

/*
                          0         1         2     0
  COMPLETED_PACKET = PACKET_TYPE SUCCESS MOTOR_NUM \n
                         0x01               
  PACKET_TYPES:
    MOTOR_FINISHED = 0x01

  SUCCESS_TYPES:
    SUCCESS = 0x06
    FAIL    = 0x15

  Types not motor related, MOTOR_NUM = 0.
/*


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

/* Create a structure for the motors' state.
 *  direction         = the direction the motor should travel.
 *  distance          = distance left to travel.
 *  micro_between     = the delay between on-off toggle.
 *  next_toggle_index = number of delays before toggling.
 */
struct MOTOR_STATE {
  uint8_t direction;
  uint8_t steps;
  uint8_t micro_between;
  uint8_t delay_cursor;
  bool enabled;
};

struct BUFFER {
  uint8_t data[RX_BUFFER_SIZE];
  uint8_t bufferSize;
  uint8_t index;
  bool packetComplete;
  uint8_t shutdownThreshold;
};

/* Initialize motors */
MOTOR motorX = {
      X_DIR_PIN,
      X_STEP_PIN,
      X_ENABLE_PIN,
      PULSE_WIDTH_MICROS
};

MOTOR motorY = {
      Y_DIR_PIN,
      Y_STEP_PIN,
      Y_ENABLE_PIN,
      PULSE_WIDTH_MICROS
};

MOTOR motorZ = {
      Z_DIR_PIN,
      Z_STEP_PIN,
      Z_ENABLE_PIN,
      PULSE_WIDTH_MICROS
};

MOTOR motorE0 = {
      E0_DIR_PIN,
      E0_STEP_PIN,
      E0_ENABLE_PIN,
      PULSE_WIDTH_MICROS
};

MOTOR motorE1 = {
      X_DIR_PIN,
      X_STEP_PIN,
      X_ENABLE_PIN,
      PULSE_WIDTH_MICROS
};

MOTOR_STATE motorXState = { DIR_CC, 0, 0, 0, false };
MOTOR_STATE motorYState = { DIR_CC, 0, 0, 0, false };
MOTOR_STATE motorZState = { DIR_CC, 0, 0, 0, false };
MOTOR_STATE motorE0State = { DIR_CC, 0, 0, 0, false };
MOTOR_STATE motorE1State  = { DIR_CC, 0, 0, 0, false };

// All motors.
int allMotors[] = { MOTOR_X, MOTOR_Y, MOTOR_Z, MOTOR_E0, MOTOR_E1 };

// Urgent shutdown.
volatile bool halt = false;
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
  motorSetup(motorY);
  motorSetup(motorZ);
  motorSetup(motorE0);
  motorSetup(motorE1);
  
  // Disable holding torque.
  digitalWrite(motorX.enable_pin, HIGH);
  digitalWrite(motorY.enable_pin, HIGH);
  digitalWrite(motorZ.enable_pin, HIGH);
  digitalWrite(motorE0.enable_pin, HIGH);
  digitalWrite(motorE1.enable_pin, HIGH);
  
  rxBuffer.bufferSize = RX_BUFFER_SIZE;
}

/* Main */
void loop()
{
  if (rxBuffer.packetComplete) {
    // If packet is packetComplete
    handleCompletePacket(rxBuffer);
    // Clear the buffer for the next packet.
    resetBuffer(&rxBuffer);
  }
  
  // Start the motor
  writeMotor();
}

/*  ############### PACKETS ###############
 * 
*/
void handleCompletePacket(BUFFER rxBuffer) {
    
    int packetProcessingIndex = 0;
    
    for (int i = 0; i < RX_BUFFER_SIZE; i+=PACKET_LENGTH)
    {
      uint8_t packet_type = rxBuffer.data[0];
      
      switch (packet_type) {
        case DRIVE_CMD:

            // Unpack the command.
            uint8_t motorNumber =  rxBuffer.data[i+1];
            uint8_t direction =  rxBuffer.data[i+2];
            uint16_t steps = ((uint8_t)rxBuffer.data[i+3] << 8)  | (uint8_t)rxBuffer.data[i+4];
            uint8_t microSecondsDelay = rxBuffer.data[i+5];
            
            // Should we move this motor.
            if (steps > 0) {
              // Set motor state.
              Serial.print("Motor number #");
              Serial.println(motorNumber);
              setMotorState(motorNumber, direction, steps, microSecondsDelay);
            }
            
            // Let the master know command is in process.
            sendAck();
          break;
        default:
          sendNack();
          break;
      }
    }
}


/*  ############### MOTORS ###############
 * 
*/

MOTOR getMotor(uint8_t motorNumber) {
  switch (motorNumber)
  {
    case MOTOR_X:
      return motorX;
      break;
    case MOTOR_Y:
      return motorY;
      break;
    case MOTOR_Z:
      return motorZ;
      break;
    case MOTOR_E0:
      return motorE0;
      break;
    case MOTOR_E1:
      return motorE0;
      break;
    default:
      break;
  }
}

MOTOR_STATE* getMotorState(uint8_t motorNumber) {
  switch (motorNumber)
  {
    case MOTOR_X:
      return &motorXState;
      break;
    case MOTOR_Y:
      return &motorYState;
      break;
    case MOTOR_Z:
      return &motorZState;
      break;
    case MOTOR_E0:
      return &motorE0State;
      break;
    case MOTOR_E1:
      return &motorE0State;
      break;
    default:
      break;
  }
}

void setMotorState(uint8_t motorNumber, uint8_t direction, uint16_t steps, uint8_t microSecondsDelay) {

    // Get reference to motor state.
    MOTOR_STATE* motorState = getMotorState(motorNumber);

    // Update with target states.
    motorState->direction = direction;
    motorState->steps = steps;
    motorState->micro_between = microSecondsDelay;
    motorState->delay_cursor = 0;
}

void resetMotorState(MOTOR_STATE* motorState){
  motorState->direction = DIR_CC;
  motorState->steps = 0;
  motorState->micro_between = 0;
  motorState->delay_cursor = 0;
  motorState->enabled = false;
}

/* Method for initalizing MOTOR */
void motorSetup(MOTOR motor) {

  // Setup motor pins
  pinMode(motor.direction_pin, OUTPUT);
  pinMode(motor.step_pin, OUTPUT);
  pinMode(motor.enable_pin, OUTPUT);
}

/* Write to MOTOR */
void writeMotor() {

    // Loop over all motors.
    for (int i = 0; i < int(sizeof(allMotors)/sizeof(int)); i++)
    {

      // Get motor and motorState for this motor.
      MOTOR motor = getMotor(allMotors[i]);
      MOTOR_STATE* motorState = getMotorState(allMotors[i]);

      // Check if motor needs to move.
      if (motorState->steps > 0) {
        
        // Enable motor.
        if (motorState->enabled == false) {
          enableMotor(motor, motorState);
        }

        // Set motor direction.
        setDirection(motor, motorState->direction);

        // If delay expired, write step.
        if (motorState->delay_cursor > motorState->micro_between) {
            // Reset motor's delay.
            motorState->delay_cursor = 0;
            writeMotor(motor);
            motorState->steps -= 1;
        }        
      }

      // Update delay cursor.
      motorState->delay_cursor += 1;

      // If steps are finished, disable motor and reset state.
      if (motorState->steps == 0 && motorState->enabled == true ) {
        Serial.println("Disabled motor");
        disableMotor(motor, motorState);
        resetMotorState(motorState);
      }
    }

    // Delay for all motors.
    delayMicroseconds(1);
}

void writeMotor(MOTOR motor) {
    digitalWrite(motor.step_pin, HIGH);
    delayMicroseconds(motor.pulse_width_micros);
    digitalWrite(motor.step_pin, LOW);
}

void enableMotor(MOTOR motor, MOTOR_STATE* motor_state) {
    // Enable motor.
    digitalWrite(motor.enable_pin, LOW);
    motor_state->enabled = true;
}

void disableMotor(MOTOR motor, MOTOR_STATE* motor_state) {
    // Disable holding torque.
    digitalWrite(motor.enable_pin, HIGH);
    motor_state->enabled = false;
}

void setDirection(MOTOR motor, uint8_t direction) {
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

    if (rxBuffer.index == PACKAGE_SIZE) {
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

void greetings() {
  Serial.println("RAMPs 1.4 stepper driver.");
  Serial.println("Welcome!");
}
// END COMMUNICATION

