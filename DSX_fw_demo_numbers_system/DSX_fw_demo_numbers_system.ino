/*
   Jay Pacamarra
   Created: 3/10/2021 1:00PM

   Just a test program for inputting number commands and parsing them into the correct sections

   Example: XXYYSZZZZW

   XX   = ID    (0-99)
   YY   = loc   (0-99)
   S    = sign  (0,1)
   ZZZZ = val   (0-9999)
   W    = ret   (0-9)

*/

// defines to make extracting information from the Buffer more readable
// These defines are indexes for reading the Buffer
// For reference: BUFFER[numChar] = {0,1,2,3,4,5,6,7,8,9}   <- These numbers are indexes


#include <Servo.h>


#define ID_START    0
#define ID_END      1
#define loc_START   2
#define loc_END     3
#define sign_START  4
#define sign_END    4
#define val_START   5
#define val_END     8
#define ret_START   9
#define ret_END     9

#define ID_Length   2
#define loc_Length  2
#define sign_Length 1
#define val_Length  4
#define ret_Length  1

#define ret_cmd_complete_ping 9

#define cmd_digitalWrite      10
#define cmd_digitalRead       11
#define cmd_getPinMode        12
#define cmd_analogRead        13
#define cmd_pwmWrite          14
#define cmd_setPWMFreq        15
#define cmd_servo             16
#define cmd_getEncoderSpeed   17
#define cmd_getEncoderDir     18
#define cmd_getSerialInfo     19
#define cmd_getSysStatus      20

// ****** Variables ******
const int numChar = 10;             // number of characters we want.
char Buffer[numChar];               // Buffer to hold characters or number commands.
char endMarker = '\n';              // Our personal endMarker or terminating character. We can change this.
bool newDataIsAvailable = false;    // bool variable so we know if new data is available to start executing commands.
bool readyToExecuteCmd = false;     // bool variable so we know if we can start executing a command
int pin_type;                       // holds the pin type such as digital input, digital output, etc.

typedef enum
{
  DIn,    // digital input
  DOut,   // digital output
  AIn,    // analog input
  AOut    // pwm analog output
} pinType;

unsigned char encoderChA = 2;       // encoder pin channel A
unsigned char encoderChB = 4;       // encoder pin channel B
unsigned char ardDioInPins[] = {encoderChA, encoderChB, 7, 10}; // available arduino INPUT digital pins
unsigned char ardDioOutPins[] = {8, 9, 12, 13};                 // available arduino OUTPUT digital pins
unsigned char ardPwmPins[] = {3, 5, 11};                        // available arduino pwm pins
// PWM pins 3 and 11 can configure their frequency
unsigned char ardAnalogPins[] = {20, 21, 22, 23, 24, 25};             // A0,A1,A2,A3,A4,A5
unsigned char ardServoPin = 6;                                  // available arduino servo pin

// Create servo object to use servo
Servo myservo;

int ID = -1;      // ID number for command, see lookup table
int loc = -1;     // location (pin) for command, see lookup table
int sign = -1;    // sign of val, 1=positive and 0=negative
int val = -1;     // the value for the command, see lookup table
int ret = -1;     // return type, see lookup table

void setup()
{
  Serial.begin(9600);   // Set up serial communication

  for (unsigned int i = 0; i < sizeof(ardDioInPins); ++i) // Setup digital input pins
    pinMode(ardDioInPins[i], INPUT);
  for (unsigned int i = 0; i < sizeof(ardDioOutPins); ++i) // Setup digital output pins
    pinMode(ardDioOutPins[i], OUTPUT);
  myservo.attach(ardServoPin);                // Setup servo pin

  // Debugging

}

void loop()
{
  serialReceive();      // fill in the buffer which holds the input characters if serial data is detected
  separateCommand();    // Separate buffer into the ID,loc,sign,val,ret. Each of these are integer numbers.
  executeCommand();     // execute command based on the ID,loc,sign,val,ret
}

void serialReceive()
{
  static int index = 0; // index variable for filling in the buffer
  char rChar;           // received character, from the output of Serial.read() function

  while (Serial.available() > 0)
  {
    rChar = Serial.read();
    if (rChar != endMarker && index <= numChar - 1)
    {
      Buffer[index] = rChar;
      index++;
    }
    else
    {
      Buffer[index] = '\0'; // terminate the buffer
      index = 0;            // reset the counter

      //Check if Buffer is the correct length
      if (strlen(Buffer) != numChar)
      {
        clear_buffer();
      }
      else
      {
        newDataIsAvailable = true;
      }
    }
  }
}

void clear_buffer()
{
  strncpy(Buffer, "\0", sizeof(Buffer));
}

void separateCommand()
{
  // These arrays will temporarily hold the numberse for the atoi function
  char tmpID[ID_Length + 1];      // need an extra space for a null character
  char tmploc[loc_Length + 1];    // need an extra space for a null character
  char tmpsign[sign_Length + 1];  // need an extra space for a null character
  char tmpval[val_Length + 1];    // need an extra space for a null character
  char tmpret[ret_Length + 1];    // need an extra space for a null character

  unsigned int i;

  if (newDataIsAvailable)
  {
    /* ***** GET ID FROM BUFFER ***** */
    for (i = 0 ; i < ID_Length ; ++i)
    {
      tmpID[i] = Buffer[i];
    }
    tmpID[ID_Length] = '\0'; // terminate with a null character cuz c


    /* ***** GET location FROM BUFFER ***** */
    for (i = 0 ; i < loc_Length ; ++i)
    {
      tmploc[i] = Buffer[i + loc_START];
    }
    tmploc[loc_Length] = '\0';


    /* ***** GET sign FROM BUFFER ***** */
    tmpsign[0] = Buffer[sign_START];
    tmpsign[sign_Length] = '\0';


    /* ***** GET value FROM BUFFER ***** */
    for (i = 0 ; i < val_Length ; ++i)
    {
      tmpval[i] = Buffer[i + val_START];
    }
    tmpval[val_Length] = '\0';


    /* ***** GET return type FROM BUFFER ***** */
    tmpret[0] = Buffer[ret_START];
    tmpret[ret_Length] = '\0';


    // convert the temporary arrays to actual integer numbers
    ID = atoi(tmpID);
    loc = atoi(tmploc);
    sign = atoi(tmpsign);
    val = atoi(tmpval);
    ret = atoi(tmpret);

    /* DEBUGGING */
    //    Serial.println(ID);
    //    Serial.println(loc);
    //    Serial.println(sign);
    //    Serial.println(val);
    //    Serial.println(ret);

    newDataIsAvailable = false;

    readyToExecuteCmd = true;

    clear_buffer();
  }
}

void executeCommand()
{
  // update value to be negative if sign is 0
  if (sign == 0) val *= -1;

  // Identify which command we want to execute
  // If command not identified, it is ignored with the default case
  // Execute command if pin is correct and if we're ready to execute
  if (readyToExecuteCmd)
  {
    switch (ID)
    {
      case cmd_digitalWrite:
        pin_type = DOut;
        if (is_pin_valid(pin_type, loc))
          exec_digitalWrite(loc, val);
        break;

      case cmd_digitalRead:
        pin_type = DIn;
        if (is_pin_valid(pin_type, loc))
          exec_digitalRead(loc);
        break;

      case cmd_getPinMode:
        getDioMode(loc);
        break;

      case cmd_analogRead:
        pin_type = AIn;
        if (is_pin_valid(pin_type, loc))
          exec_analogRead(loc);
        break;

      case cmd_pwmWrite:
        pin_type = AOut;
        if (is_pin_valid(pin_type, loc))
          exec_pwm(loc, val);
        break;

      default:
        break;
    }
  }

  if (ret == ret_cmd_complete_ping)
  {
    // RETURN ping 
    returnCmdFinishedPing();
  }

  readyToExecuteCmd = false;  // After executing a command, we have to wait until we're ready to execute again
}

bool is_pin_valid(int pinType, int pin)
{
  bool valid_pin = false;
  unsigned int i;

  // check if pin matches one of the digital input pins
  if (pinType == DIn)
  {
    for (i = 0 ; i < sizeof(ardDioInPins) ; ++i)
      if (pin == ardDioInPins[i]) valid_pin = true;
  }

  // check if pin matches one of these digital output pins
  else if (pinType == DOut)
  {
    for (i = 0 ; i < sizeof(ardDioOutPins) ; ++i)
      if (pin == ardDioOutPins[i]) valid_pin = true;
  }

  // check if pin matches one of these pwm output pins
  else if (pinType == AOut)
  {
    for (i = 0 ; i < sizeof(ardPwmPins) ; ++i)
      if (pin == ardPwmPins[i]) valid_pin = true;
  }

  // check if pin matches one of these analog input pins
  else if (pinType == AIn)
  {
    for (i = 0 ; i < sizeof(ardAnalogPins) ; ++i)
      if (pin == ardAnalogPins[i]) valid_pin = true;
  }

  // return whether or not pin is valid (true or false)
  return valid_pin;
}

void exec_digitalWrite(int pin, int value) {
  if (value < 0 || value > 1) value = 0;
  digitalWrite(pin, value);
}

void exec_digitalRead(int pin) {
  char digitalvalHIGH[] = "0001";
  char digitalvalLOW[] = "0000";
  if (digitalRead(pin))
    returnVal(digitalvalHIGH);
  else
    returnVal(digitalvalLOW);
}

void getDioMode(int pin) {

  char pINPUT[] = "0000";
  char pOUTPUT[] = "0001";
  char pANALOGINPUT[] = "0002";
  char pPWMOUTPUT[] = "0003";
  char pSERVOOUTPUT[] = "0004";
  char pInvalid[] = "0005";

  if (is_pin_valid(DIn, pin)) returnVal(pINPUT);             // pin is digital input
  else if (is_pin_valid(DOut, pin)) returnVal(pOUTPUT);       // pin is digital output
  else if (is_pin_valid(AIn, pin)) returnVal(pANALOGINPUT);  // pin is Analog input
  else if (is_pin_valid(AOut, pin)) returnVal(pPWMOUTPUT);   // pin is PWM output
  else if (pin == ardServoPin) returnVal(pSERVOOUTPUT);     // pin is servo PWM output
  else returnVal(pInvalid); // pin is invalid
}

void exec_analogRead(int pin) {
  switch (pin)
  {
    case 20:
      Serial.println(analogRead(A0));
      break;
    case 21:
      Serial.println(analogRead(A1));
      break;
    case 22:
      Serial.println(analogRead(A2));
      break;
    case 23:
      Serial.println(analogRead(A3));
      break;
    case 24:
      Serial.println(analogRead(A4));
      break;
    case 25:
      Serial.println(analogRead(A5));
      break;
    default:
      break;
  }
}

void exec_pwm(int pin, int value) {
  if (value < 0) value = 0;
  if (value > 100) value = 100;
  value = map(value, 0, 100, 0, 255);
  analogWrite(pin, value);
}

void returnCmdFinishedPing()
{
  // delay a bit so simulink has a chance to catch the ping
  delay(1);

  char ping[] = "8888";
  returnVal(ping);

  // reset ret so it doesn't spam the ping after being sent once
  ret = -1;
}

void returnVal(char val[])
{
  Serial.print(ID);

  // Check if loc starts with 0, ex. 08
  // if it is then add a zero
  if (loc < 10)
  {
    Serial.print(0);
    Serial.print(loc);
  }
  else
  {
    Serial.print(loc);
  }
  Serial.print(sign);
  Serial.print(val);
  Serial.println(ret);
}
