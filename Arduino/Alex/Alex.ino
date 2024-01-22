#include <serialize.h>
#include <math.h>
#include "packet.h"
#include "constants.h"
#include <stdarg.h>



float alexDiagonal = 0.0;
float alexCirc = 0.0;

typedef enum
{
  STOP=0,
  FORWARD=1,
  BACKWARD=2,
  LEFT=3,
  RIGHT=4
} TDirection;

volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      76

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.4

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  2   // PORTBPIN2 Pin 10
#define LR                  3   // PORTBPIN3 Pin 11
#define RF                  6  // PORTDPIN6 Pin 6
#define RR                  5  // PORTDPIN5 Pin 5

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

//variables to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

//variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

//Variables for Identifying object
#define S0 7 //PORTDPIN7 Pin 7
#define S1 0 //PORTBPIN0 8
#define S2 4 //PORTBPIN4 12
#define S3 5 //PORTBPIN5 13
#define sensorOut 1 //PORTBPIN1 Pin 9

#define ALEX_LENGTH 13
#define ALEX_BREADTH 10

//variables for ultrasonic
#define TRIGGER 5 //PORTCPIN5 A5 
#define ECHO 4 //PORTCPIN4 A4
unsigned long ultrasonic_s = 0;
unsigned long duration_us = 0;
unsigned long distance_us = 0;
/*
 * 
 * Alex Communication Routines.
 * 
 */
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
}


void setupUltra()
{
  DDRC |= (1 << TRIGGER);
  DDRC &= ~(1 << ECHO); 
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.

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

void identifyObject()
{
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_IDENTIFY;
  statusPacket.params[0] = redGreen();
  sendResponse(&statusPacket);
  //ultrasonic distance
  PORTC |= 1 << TRIGGER;
  ultrasonic_s = micros();
  delayMicroseconds(10);
  PORTC &= 0b11011111;
  duration_us = pulseIn(A4, HIGH);
  distance_us = 0.017 * duration_us - 3;
  TPacket directionPacket;
  directionPacket.packetType = PACKET_TYPE_RESPONSE;
  directionPacket.command = RESP_DISTANCE;
  directionPacket.params[0] = distance_us;
  sendResponse(&directionPacket);
}

void setupIdentify()
{
  DDRD |= (1 << S0); 
  DDRB |= (1 << S1) | (1 << S2) | (1 << S3);

  //Set sensorOut pin to input mode
  DDRB &= ~(1 << sensorOut);

  //Setting freq scaling to 20%
  PORTD |=  (1 << S0);
  PORTB &= ~(1 << S1);
}

long colourArray[3] = {0,0,0}; // floats to hold colour array
//to be callibrated
long redArray[3] = {223,555,380};
long greenArray[3] = {552,412,368};
long gArray[3] = {420,309,290};
long rArray[3] = {244, 450, 317};
long blueArray[3] = {412,276,179};
long orangeArray[3] = {190,430,365};
long purpleArray[3] = {365,403,237};
long blackArray[3] = {794,707,485};
long whiteArray[3] = {152,134,95};

long red2Array[3] = {148,397,273};
long green2Array[3] = {396,261,243};
long g2Array[3] = {288,195,199};
long r2Array[3] = {181, 383, 273};
long blue2Array[3] = {247,132,78};
long orange2Array[3] = {98,246,220};
long purple2Array[3] = {229,229,122};
long black2Array[3] = {685,607,423};
long white2Array[3] = {100,90,63};

long red3Array[3] = {275,624,431};
long green3Array[3] = {630,497,427};
long g3Array[3] = {468,355, 327};
long r3Array[3] = {305,511,358};
long blue3Array[3] = {459,323,213};
long orange3Array[3] = {210,475,397};
long purple3Array[3] = {393,439,270};
long black3Array[3] = {862,761,527};
long white3Array[3] = {197,175,124};

long *refColours[21] = {redArray, red2Array, red3Array, greenArray, green2Array, 
                        green3Array, whiteArray, blackArray, blueArray, orangeArray, 
                        purpleArray, white2Array, black2Array, blue2Array, orange2Array,
                        purple2Array, black3Array, white3Array, blue3Array, orange3Array, purple3Array};
short determine_colour(long colourArray[]) {
  double min_dist = 100000;
  short min_i = -1;
  
  // finds minimum Euclidean distance between measured color and reference colors
  for (short i = 0; i < 15; i += 1) {
    double dist = sqrtf((double)(refColours[i][0] - colourArray[0]) * (refColours[i][0] - colourArray[0])
      + (double)(refColours[i][1] - colourArray[1]) * (refColours[i][1] - colourArray[1])
      + (double)(refColours[i][2] - colourArray[2]) * (refColours[i][2] - colourArray[2]));

    if (dist < min_dist) {
      min_dist = dist;
      if (i < 3)
      {
        min_i = 0;
      }
      else if (i < 6)
      {
        min_i = 1;
      }
      else{
        min_i = 2;
      }
    }
   }
return min_i;
}

// return 0 if Red, return 1 if Green, return 2 if neither detected
unsigned long redGreen()
{
  //setting red filtered photodiodes to be read
  PORTB &= ~((1 << S2) | (1 << S3));
  colourArray[0] = pulseIn(9, LOW);
  delay(100);

  //setting green filtered photodiodes to be read
  PORTB |= (1 << S2) | (1 << S3);
  colourArray[1] = pulseIn(9, LOW);
  delay(100);

  //setting blue filtered photodiodes to be read
  PORTB &= ~(1 << S2);
  PORTB |= (1 << S3);
  colourArray[2] = pulseIn(9, LOW);
  delay(100);
  return determine_colour(colourArray);
}


void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
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
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
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


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  //if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  //}
  if (dir == BACKWARD) {
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  if (dir == LEFT)
    leftReverseTicksTurns++;
  if (dir == RIGHT)
    leftForwardTicksTurns++;
}

void rightISR()
{
  //if (dir == FORWARD)
    rightForwardTicks++;
  if (dir == BACKWARD)
    rightReverseTicks++;
  if (dir == LEFT)
    rightForwardTicksTurns++;
  if (dir == RIGHT)
    rightReverseTicksTurns++;
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EIMSK |= 0b11;
  EICRA |= 0b00001010;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
ISR(INT0_vect) {
  leftISR();
}

ISR(INT1_vect) {
  rightISR();
}

// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  //Setting framework to 8N1
  UCSR0C = 0b00000110;
  //Setting baudrate to 9600 
  UBRR0L = 103;
  UBRR0H = 0;
  UCSR0A = 0;
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  //Polling mode and Enabling of UART receiver and transmitter
  UCSR0B = 0b00011000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.
int readSerial(char *buffer)
{
  int count=0;
  
  while ((UCSR0A & 0b10000000))
    buffer[count++] = UDR0;
  
  return count;
}

// Write to serial port
void writeSerial(const char *buffer, int len)
{
  for(int i = 0; i < len; i+=1) {
    while((UCSR0A & 0b00100000) == 0);
    UDR0 = buffer[i];
  }
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B RR
   *    A2IN - Pin 6, PD6, OC0A RF
   *    B1IN - Pin 10, PB2, OC1B LF
   *    B2In - pIN 11, PB3, OC2A LR
   */
  // Set up output pins
  DDRB |= ((1 << LF) | (1 << LR));
  DDRD |= ((1 << RF) | (1 << RR));
  //Set up PWM for Pin 5 and 6 (Clear on compare match up-counting)
  TCNT0 = 0;
  OCR0A = 0;
  OCR0B = 0;
  TCCR0A = 0b10100001;
  TIMSK0 |= 0b110;
  //Setup PWM for Pin 10
  TCNT1 = 0;
  OCR1BH = 0;
  OCR1BL = 0;
  TCCR1A = 0b00100001;
  TIMSK1 = 0b100; 
  //Setup PWM for Pin 11 (Clear on compare match up-counting)
  TCNT2 = 0;
  OCR2A = 0;
  TCCR2A = 0b10000001;
  TIMSK2 = 0b010;
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  //Start PWM for Pin 5 and 6
  TCCR0B = 0b00000011;
  //Start PWM for Pin 10
  TCCR1B = 0b00000011;
  //Start PWM for Pin 11
  TCCR2B = 0b00000011;
  sei();
}

ISR(TIMER0_COMPA_vect) {
}

ISR(TIMER0_COMPB_vect) {
}

ISR(TIMER1_COMPB_vect) {
}

ISR(TIMER2_COMPA_vect) {
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

int pwmVal_1(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 65535);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  if (dist == 0) 
    deltaDist = 999999;
  else
    deltaDist = dist;
    
  newDist = forwardDist + deltaDist;
  dir = FORWARD;
  int val = pwmVal(speed);
  int val_1 = pwmVal_1(speed);
  // For now we will ignore dist and move
  // forward indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  OCR0A = val;
  OCR0B = 0;
  OCR1BH = val_1 >> 8;
  OCR1BL = val_1;
  OCR2A = 0;
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  if (dist == 0) 
    deltaDist = 999999;
  else 
    deltaDist = dist;

  newDist = reverseDist + deltaDist;
  dir = BACKWARD;
  
  int val = pwmVal(speed);
  
  // For now we will ignore dist and 
  // reverse indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  OCR0A = 0;
  OCR0B = val;
  OCR1BH = 0;
  OCR1BL = 0;
  OCR2A = val;
}

unsigned long computeDeltaTicks(float ang){
  unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  dir = LEFT;
  
  int val = pwmVal(speed);
  int val_1 = pwmVal_1(speed);
  if (ang == 0) 
    deltaTicks = 999999;
  else {
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = leftReverseTicksTurns + deltaTicks;
  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  OCR0A = 0;
  OCR0B = val;
  OCR1BH = val_1 >> 8;
  OCR1BL = val_1;
  OCR2A = 0;
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;
  
  int val = pwmVal(speed);

  if (ang == 0) 
    deltaTicks = 999999;
  else {
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = leftForwardTicksTurns + deltaTicks;
  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  OCR0A = val;
  OCR0B = 0;
  OCR1BH = 0;
  OCR1BL = 0;
  OCR2A = val;
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{ 
  OCR0A = 0;
  OCR0B = 0;
  OCR1BH = 0;
  OCR1BL = 0;
  OCR2A = 0;
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  leftForwardTicksTurns=0; 
  rightForwardTicksTurns=0;
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
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
        stop();
        break; 
    case COMMAND_GET_STATS:
        sendOK();
        sendStatus();
        break;
    case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter(command->params[0]);
        break;
    case COMMAND_IDENTIFY_OBJECT:
        sendOK();
        identifyObject();
        break;
    
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:
  alexDiagonal = sqrt((ALEX_LENGTH*ALEX_LENGTH) + (ALEX_BREADTH*ALEX_BREADTH));
  alexCirc = PI*alexDiagonal;
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  setupIdentify();
  setupUltra();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
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

void loop() {
 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
  else if(result == PACKET_CHECKSUM_BAD)
    {
      sendBadChecksum();
    } 
    
  if (deltaDist > 0) {
    if (dir == FORWARD) {
      if (forwardDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == BACKWARD) {
      if (reverseDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }
  
  if (deltaTicks > 0) {
    if (dir == LEFT) {
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == RIGHT) {
      if (leftForwardTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == STOP) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
}
