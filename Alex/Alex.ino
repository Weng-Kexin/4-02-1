#include <serialize.h>
#include "packet.h"
#include "constants.h"

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  11  // Right reverse pin

/*
      Alex's State Variables
*/

// Store the ticks from Alex's left and
// right encoders.
static volatile unsigned long leftForwardTicks;
static volatile unsigned long rightForwardTicks;
static volatile unsigned long leftReverseTicks;
static volatile unsigned long rightReverseTicks;

// Left and right encoder ticks for turning
static volatile unsigned long leftForwardTicksTurns;
static volatile unsigned long rightForwardTicksTurns;
static volatile unsigned long leftReverseTicksTurns;
static volatile unsigned long rightReverseTicksTurns;

static volatile unsigned long diff;
// Store the revolutions on Alex's left
// and right wheels
volatile float leftRevs;
volatile float rightRevs;

// Forward and backward distance traveled
volatile float forwardDist;
volatile float reverseDist;
volatile float degree;

TResult readPacket(TPacket *packet)
{
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);

}

void sendStatus()
{
  //sends tick status to pi
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;

  sendResponse(&statusPacket);
}

void sendColour(int colour)
{
  //sends colour "status" when findColour is called on the pi
  
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_COLOUR;
  statusPacket.params[0] = colour;
  statusPacket.params[1] = 0;
  statusPacket.params[2] = 0;
  statusPacket.params[3] = 0;
  statusPacket.params[4] = 0;
  statusPacket.params[5] = 0;
  statusPacket.params[6] = 0;
  statusPacket.params[7] = 0;
  statusPacket.params[8] = 0;
  statusPacket.params[9] = 0;

  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);

}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.

  DDRD  &= 0b11110011;
  PORTD |= 0b00001100;
}
#define COUNTS_PER_DEGREE_LEFT   0.80
#define COUNTS_PER_DEGREE_RIGHT   0.80
#define COUNTS_PER_REV      182
#define WHEEL_CIRC          20.41

// Functions to be called by INT0 and INT1 ISRs.

static volatile float ultraTime = 1000;
static volatile float distance;
static unsigned long currTime; static unsigned long lastTime = 0;
void leftISR()
{
  switch (dir)
  {
    case STOP:
      break;

    case FORWARD:
      leftForwardTicks++;
      forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
      diff = leftForwardTicks - rightForwardTicks;
      break;


    case BACKWARD:
      leftReverseTicks++;
      reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
      diff = leftReverseTicks - rightReverseTicks;
      break;

    case LEFT:
      leftReverseTicksTurns++;
      degree = (unsigned long) ((float) leftReverseTicksTurns / COUNTS_PER_DEGREE_LEFT);
      diff = leftReverseTicksTurns - rightForwardTicksTurns;
      break;

    case RIGHT:
      leftForwardTicksTurns++;
      degree = (unsigned long) ((float) leftForwardTicksTurns / COUNTS_PER_DEGREE_RIGHT);
      diff = leftForwardTicksTurns - rightReverseTicksTurns;
      break;
  }
//    Serial.print("LEFT FORW TICKS TURNS: ");
//    Serial.println(leftForwardTicksTurns);
//    Serial.print("LEFT REVRS TICKS TURNS: ");
//    Serial.println(leftReverseTicksTurns);
//    Serial.print("DEGREE: ");
//    Serial.println(degree);
//    Serial.print("LEFT Forward Dist: ");
//    Serial.println(forwardDist );
//    Serial.print("LEFT Reverse Dist: ");
//    Serial.println(reverseDist );
}

void rightISR()
{
  switch (dir)
  {
    case STOP:
      break;

    case FORWARD:
      rightForwardTicks++;
      break;

    case BACKWARD:
      rightReverseTicks++;
      break;

    case LEFT:
      rightForwardTicksTurns++;
      break;

    case RIGHT:
      rightReverseTicksTurns++;
      break;
  }
  //  rightRevs = rightTicks / COUNTS_PER_REV;
  //  forwardDist = rightRevs * WHEEL_CIRC;
  //  Serial.print("RIGHT FORW TICKS: ");
  //  Serial.println(rightForwardTicks);
  //  Serial.print("RIGHT REVRS TICKS: ");
  //  Serial.println(rightReverseTicks);
  //  Serial.print("RIGHT Dist: ");
  //  Serial.println(forwardDist );
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.

  EICRA = 0b00001010;
  EIMSK = 0b00000011;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect)
{
  leftISR();
}

ISR(INT1_vect)
{
  rightISR();
}


// Implement INT0 and INT1 ISRs above.

/*
   Setup and start codes for serial communications

*/
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(57600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.

}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count = 0;

  while (Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

/*
   Alex's motor drivers.

*/

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:
        A1IN - Pin 5, PD5, OC0B
        A2IN - Pin 6, PD6, OC0A
        B1IN - Pin 10, PB2, OC1B
        B2In - pIN 11, PB3, OC2A
  */
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{

}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if (speed < 0.0)
    speed = 0;

  if (speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 250.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  int val = pwmVal(speed);
  int diff_left, diff_right;
  dir = FORWARD;
  
  ultrasonicDist();
  ultraTime = (ultraTime < 10) ? 900 : ultraTime;
  while(forwardDist < dist && (ultraTime > 700)){
    diff_left = (diff > 0)? diff : 0;
    diff_right = (diff > 0)? 0 : diff;
    analogWrite(LF, val - diff_left);
    analogWrite(RF, val - diff_right);
    analogWrite(LR, 0);
    analogWrite(RR, 0);
    ultrasonicDist();
    ultraTime = (ultraTime < 10) ? 900 : ultraTime;
  }
  stop_motor();

}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{

  int val = pwmVal(speed);
  int diff_left, diff_right;
  dir = BACKWARD;

  while (reverseDist < dist)
  {
    diff_left = (diff > 0)? diff : 0;
    diff_right = (diff > 0)? 0 : diff;
    analogWrite(LF, 0);
    analogWrite(RF, 0);
    analogWrite(LR, val - diff_left);
    analogWrite(RR, val - diff_right);
  }
  stop_motor();
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  int val = pwmVal(speed);
  int diff_left, diff_right;  
  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  dir  = LEFT;
  while (degree < ang)
  {
    diff_left = (diff > 0)? diff : 0;
    diff_right = (diff > 0)? 0 : diff;
    analogWrite(LF, 0);
    analogWrite(RF, val - diff_right);
    analogWrite(LR, val - diff_left);
    analogWrite(RR, 0);    
  }
  stop_motor();
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  int val = pwmVal(speed);
  int diff_left, diff_right;
  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  dir = RIGHT;
  while (degree < ang)
  {
    diff_left = (diff > 0)? diff : 0;
    diff_right = (diff > 0)? 0 : diff;
    analogWrite(LF, val - diff_left);
    analogWrite(RF, 0);
    analogWrite(LR, 0);
    analogWrite(RR, val - diff_right);    
  }
  stop_motor();
}

// Stop Alex. To replace with bare-metal code later.
void stop_motor()
{
  dir = STOP;
  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
}

/*
   Alex's setup and run codes

*/

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;

  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;
  forwardDist = 0;
  reverseDist = 0;
  degree = 0;
}

// Clears one particular counter
void clearOneCounter(int which)
{
  //  switch(which)
  //  {
  //    case 0:
  clearCounters();
  //      break;

  //    case 1:
  //      leftTicks=0;
  //      break;
  //
  //    case 2:
  //      rightTicks=0;
  //      break;
  //
  //    case 3:
  //      leftRevs=0;
  //      break;
  //
  //    case 4:
  //      rightRevs=0;
  //      break;
  //
  //    case 5:
  //      forwardDist=0;
  //      break;
  //
  //    case 6:
  //      reverseDist=0;
  //      break;
  //  }
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch (command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_STOP:
      sendOK();
      stop_motor();
      break;
    case COMMAND_GET_STATS:
      sendOK();
      sendStatus();
      break;
    case COMMAND_CLEAR_STATS:
      sendOK();
      clearOneCounter(command->params[0]);
      break;
    case COMMAND_FIND_COLOUR:
      sendOK();
      findColour();
      break;


    /*
       Implement code for other commands here.

    */

    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit = 0;

  while (!exit)
  {
    TPacket hello;
    TResult result;

    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {


        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}


void setup() {
  // put your setup code here, to run once:

  cli();
  //ultrasonic
  ultrasonicsetup();
  
  //TCS3200
  coloursetup();


  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}


#define THRESHOLD 5
void loop() {


  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK)
    handlePacket(&recvPacket);
  else if (result == PACKET_BAD)
  {
    sendBadPacket();
  }
  else if (result == PACKET_CHECKSUM_BAD)
  {
    sendBadChecksum();
  }


}
