#include <avr/interrupt.h> 
#include <avr/io.h> 
#include "pins.h"


// https://reprap.org/mediawiki/images/f/f6/RAMPS1.4schematic.png
// https://reprap.org/forum/read.php?219,168722
// https://google.github.io/styleguide/cppguide.html

// TODO: Modify motor steps to be in delay window.
// TODO: Remove unneeded "MOTOR_NUM", use packet index to determine motor.
// TODO: Pulse width set by initialization.
// TODO: Add a timer to shutdown motors after threshold.
//       And keep motor enabled until threshold has been met.
// TODO: Add a "holding torque" feature; making it so motors never disable.

// Output debug info?
#define DEBUG                 true

// Determine the pulse width of motor.
#define MOTOR_ANGLE           1.8
#define PULSE_WIDTH_MICROS    360 / MOTOR_ANGLE
#define NUM_MOTORS            5
#define PACKET_LENGTH         5
#define RX_BUFFER_SIZE        NUM_MOTORS * PACKET_LENGTH
#define PACKET_SIZE           5
#define STEP_WINDOW           10 // Number of microsecond variance allowed per step.
#define SENTINEL              0
#define MINIMUM_STEPPER_DELAY 2000

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
                       0        1        2        3                  4       
  MOTOR_PACKET = PACKET_TYPE   DIR   STEPS_1    STEPS_2    MICROSECONDS_BETWEEN
  MOTOR_PACKET =    01         00       03        E8                 05        
  MOTOR_PACKET =    0x 01010003E8050A

  HALT         = 0x0F

  PACKAGE = PACKET1 PACKET2 PACKET3 PACKET4 PACKET5
  
  PACKAGE_EXAMPLE = 01 00 03 E8 05       01 00 03 E8 05     01 00 03 E8 05     01 00 03 E8 05      01 00 03 E8 05
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
 *  step_delay     = the delay between on-off toggle.
 *  next_step_at       = when the motor should take its next step.
 */
struct MOTOR_STATE {
  uint8_t direction;
  uint16_t steps;
  unsigned long step_delay;
  unsigned long next_step_at;
  bool enabled;
};

struct BUFFER {
  uint8_t data[RX_BUFFER_SIZE];
  uint8_t buffer_size;
  uint8_t index;
  bool packet_complete;
  uint8_t shutdown_threshold;
};

/* Initialize motors */
MOTOR motor_x = {
      X_DIR_PIN,
      X_STEP_PIN,
      X_ENABLE_PIN,
      PULSE_WIDTH_MICROS
};

MOTOR motor_y = {
      Y_DIR_PIN,
      Y_STEP_PIN,
      Y_ENABLE_PIN,
      PULSE_WIDTH_MICROS
};

MOTOR motor_z = {
      Z_DIR_PIN,
      Z_STEP_PIN,
      Z_ENABLE_PIN,
      PULSE_WIDTH_MICROS
};

MOTOR motor_e0 = {
      E0_DIR_PIN,
      E0_STEP_PIN,
      E0_ENABLE_PIN,
      PULSE_WIDTH_MICROS
};

MOTOR motor_e1 = {
      X_DIR_PIN,
      X_STEP_PIN,
      X_ENABLE_PIN,
      PULSE_WIDTH_MICROS
};

MOTOR_STATE motor_x_state = { DIR_CC, 0, 0, SENTINEL, false };
MOTOR_STATE motor_y_state = { DIR_CC, 0, 0, SENTINEL, false };
MOTOR_STATE motor_z_state = { DIR_CC, 0, 0, SENTINEL, false };
MOTOR_STATE motor_e0_state = { DIR_CC, 0, 0, SENTINEL, false };
MOTOR_STATE motor_e1_state  = { DIR_CC, 0, 0, SENTINEL, false };

// All motors.
int all_motors[] = { MOTOR_X, MOTOR_Y, MOTOR_Z, MOTOR_E0, MOTOR_E1 };

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
  motorSetup(motor_x);
  motorSetup(motor_y);
  motorSetup(motor_z);
  motorSetup(motor_e0);
  motorSetup(motor_e1);
  
  // Disable holding torque.
  digitalWrite(motor_x.enable_pin, HIGH);
  digitalWrite(motor_y.enable_pin, HIGH);
  digitalWrite(motor_z.enable_pin, HIGH);
  digitalWrite(motor_e0.enable_pin, HIGH);
  digitalWrite(motor_e1.enable_pin, HIGH);
  
  rxBuffer.buffer_size = RX_BUFFER_SIZE;
}

/* Main */
void loop()
{
  if (rxBuffer.packet_complete) {
    // If packet is packet_complete
    handleCompletePacket(rxBuffer);
    // Clear the buffer for the next packet.
    resetBuffer(&rxBuffer);
  }
  
  // Start the motor
  pollMotor();
}

/*  ############### PACKETS ###############
 * 
*/
void handleCompletePacket(BUFFER rxBuffer) {
    
    uint8_t packet_type = rxBuffer.data[0];
      
    switch (packet_type) {
      case DRIVE_CMD:

          // Unpack the command.
          uint8_t motorNumber =  rxBuffer.data[1];
          uint8_t direction =  rxBuffer.data[2];
          uint16_t steps = ((uint8_t)rxBuffer.data[3] << 6)  | (uint8_t)rxBuffer.data[4];
          unsigned long microSecondsDelay = rxBuffer.data[5] * 1000; // Delay comes in as milliseconds.

          if (microSecondsDelay < MINIMUM_STEPPER_DELAY) { microSecondsDelay = MINIMUM_STEPPER_DELAY; }

          // Should we move this motor.
          if (steps > 0) {
            // Set motor state.
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


/*  ############### MOTORS ###############
 * 
*/

MOTOR getMotor(uint8_t motorNumber) {
  switch (motorNumber)
  {
    case MOTOR_X:
      return motor_x;
      break;
    case MOTOR_Y:
      return motor_y;
      break;
    case MOTOR_Z:
      return motor_z;
      break;
    case MOTOR_E0:
      return motor_e0;
      break;
    case MOTOR_E1:
      return motor_e0;
      break;
    default:
      break;
  }
}

MOTOR_STATE* getMotorState(uint8_t motorNumber) {
  switch (motorNumber)
  {
    case MOTOR_X:
      return &motor_x_state;
      break;
    case MOTOR_Y:
      return &motor_y_state;
      break;
    case MOTOR_Z:
      return &motor_z_state;
      break;
    case MOTOR_E0:
      return &motor_e0_state;
      break;
    case MOTOR_E1:
      return &motor_e1_state;
      break;
    default:
      Serial.println("No motor state found.");
      break;
  }
}

void setMotorState(uint8_t motorNumber, uint8_t direction, uint16_t steps, unsigned long microSecondsDelay) {

    // Get reference to motor state.
    MOTOR_STATE* motorState = getMotorState(motorNumber);

    if( DEBUG ) {
      Serial.print("Motor number #");
      Serial.println(motorNumber);
      Serial.print("direction:");
      Serial.println(direction);
      Serial.print("steps:");
      Serial.println(steps);
      Serial.print("microSecondsDelay:");
      Serial.println(microSecondsDelay);
    }

    // Update with target states.
    motorState->direction = direction;
    motorState->steps = steps;
    motorState->step_delay = microSecondsDelay;
    motorState->next_step_at = micros() + microSecondsDelay;
}

void resetMotorState(MOTOR_STATE* motorState){
  motorState->direction = DIR_CC;
  motorState->steps = 0;
  motorState->step_delay = 0;
  motorState->next_step_at = SENTINEL;
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
void pollMotor() {

    unsigned long current_micros = micros();

    // Loop over all motors.
    for (int i = 0; i < int(sizeof(all_motors)/sizeof(int)); i++)
    {

      // Get motor and motorState for this motor.
      MOTOR motor = getMotor(all_motors[i]);
      MOTOR_STATE* motorState = getMotorState(all_motors[i]);
      
      // Check if motor needs to move.
      if (motorState->steps > 0) {

        // Initial step timer.
        if (motorState->next_step_at == SENTINEL) {
          motorState->next_step_at = micros() + motorState->step_delay;
        }

        // Enable motor.
        if (motorState->enabled == false) {
          enableMotor(motor, motorState);
        }

        // Set motor direction.
        setDirection(motor, motorState->direction);

        unsigned long window = motorState->step_delay;  // we should be within this time frame

        if(current_micros - motorState->next_step_at < window) {         
            writeMotor(motor);
            motorState->steps -= 1;
            motorState->next_step_at += motorState->step_delay;
            // Serial.println(motorState->steps);
        }
      }

      // If steps are finished, disable motor and reset state.
      if (motorState->steps == 0 && motorState->enabled == true ) {
        Serial.println("Disabled motor");
        disableMotor(motor, motorState);
        resetMotorState(motorState);
      }
    }
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
uint8_t decodePacket(uint8_t value) {
  return (value >> 2) &~ 0xC0;
}

uint8_t encodePacket(uint8_t value) {
  return (value << 2) | 0x03;
}

void serialEvent() {

  // Get all the data.
  while (Serial.available()) {

    // Read a byte
    uint8_t inByte = (uint8_t)Serial.read();



    if (inByte == END_TX) {
      rxBuffer.packet_complete = true;
    } else {
      // Store the byte in the buffer.
      inByte = decodePacket(inByte);
      rxBuffer.data[rxBuffer.index] = inByte;
      rxBuffer.index++;
    }
  }
}

// Clear the buffer.
void resetBuffer(struct BUFFER *buffer) {
  memset(buffer->data, 0, sizeof(buffer->data));
  buffer->index = 0;
  buffer->packet_complete = false;
}

// Does not count termination char.
int packetLength(BUFFER buffer){
  for(int i = 0; i < buffer.buffer_size; i++) {
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
  Serial.println("Protocol reserves first two bits for flow control. ");
  Serial.println("Packet: MOTOR_PACKET = PACKET_TYPE_CHAR MOTOR_NUM DIR STEPS_1 STEPS_2 MILLI_BETWEEN 0x04");
  Serial.println("Encode: VALUE = (VALUE << 2) | 0x03");
  Serial.println("Decode: VALUE = (VALUE >> 2) &~ 0xC0");
  Serial.println("Pre-encoded value : 01 00 01 3F 3F 05 04");
  Serial.println("Post-encoded value: 07 03 07 FF FF 17 04");
}
// END COMMUNICATION

