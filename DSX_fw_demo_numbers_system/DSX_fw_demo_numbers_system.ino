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
#include <SPI.h>

#define ENC_COUNT_REV 48

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
#define cmd_getEncoderInfo    17
#define cmd_getSerialInfo     18
#define cmd_getSysStatus      19
#define cmd_dacWrite          20

/**** Macros for changing the PWM Frequency ****/
#define CLEAR_LAST3_LSB   B11111000
#define FREQ_32KHZ        B00000001
#define FREQ_4KHZ         B00000010
#define FREQ_980HZ        B00000011
#define FREQ_490HZ        B00000100 // default
#define FREQ_245HZ        B00000101
#define FREQ_122HZ        B00000110
#define FREQ_30HZ         B00000111

// ****** Variables ******
const int numChar = 10;             // number of characters we want.
char Buffer[numChar];               // Buffer to hold characters or number commands.
char endMarker = '\n';              // Our personal endMarker or terminating character. We can change this.
bool newDataIsAvailable = false;    // bool variable so we know if new data is available to start executing commands.
bool readyToExecuteCmd = false;     // bool variable so we know if we can start executing a command
int pin_type;                       // holds the pin type such as digital input, digital output, etc.
int SERIAL_CONFIG = SERIAL_8N1;     // holds data,parity, and stop bit config

volatile long encoderValue = 0;
int interval = 1000;      // One-second interval for measurements
long previousMillis = 0;  // Counter for milliseconds during interval
long currentMillis = 0;   // Counter for milliseconds during interval
int rpm = 0;              // will hold speed from encoder in rpm
bool CW = 1;
bool CCW = 0;
bool direction;   // to store direction 1: clockwise, 0: counter clockwise

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
unsigned char slaveSelectPin = 8;
unsigned char ardDioOutPins[] = {9, slaveSelectPin, 12};    // available arduino OUTPUT digital pins
unsigned char ardPwmPins[] = {3, 5};                        // available arduino pwm pins
// PWM pins 3 and 11 can configure their frequency
unsigned char ardAnalogPins[] = {20, 21, 22, 23, 24, 25};       // A0,A1,A2,A3,A4,A5
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
  Serial.begin(9600, SERIAL_CONFIG );   // Set up serial communication
  initPins();
  initEncoder();
  initSpi();


  // Debugging

}

void loop()
{
  serialReceive();          // fill in the buffer which holds the input characters if serial data is detected
  separateCommand();        // Separate buffer into the ID,loc,sign,val,ret. Each of these are integer numbers.
  executeCommand();         // execute command based on the ID,loc,sign,val,ret
  updateEncoderReadings();  // Update encoder readings
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

      case cmd_setPWMFreq:
        exec_setPWMFreq(loc, val);
        break;

      case cmd_servo:
        exec_servoWrite(loc, val);
        break;

      case cmd_getEncoderInfo:
        pin_type = DIn;
        if (is_pin_valid(pin_type, loc))
          readEncoder();
        break;

      case cmd_getSerialInfo:
        getSerial();
        break;

      case cmd_dacWrite:
        dacWrite(val);
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
  ret = 1;
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
  char sckPin[] = "0005";
  char mosiPin[] = "0006";
  char pInvalid[] = "0007";

  // set  return type 
  ret = 2;

  if (is_pin_valid(DIn, pin)) returnVal(pINPUT);             // pin is digital input
  else if (is_pin_valid(DOut, pin)) returnVal(pOUTPUT);       // pin is digital output
  else if (is_pin_valid(AIn, pin)) returnVal(pANALOGINPUT);  // pin is Analog input
  else if (is_pin_valid(AOut, pin)) returnVal(pPWMOUTPUT);   // pin is PWM output
  else if (pin == ardServoPin) returnVal(pSERVOOUTPUT);     // pin is servo PWM output
  else if (pin == 13) returnVal(sckPin);                    // pin is serial clock output
  else if (pin == 11) returnVal(mosiPin);                   // pin is MOSI output
  else returnVal(pInvalid); // pin is invalid
}

void exec_analogRead(int pin) {
  unsigned int analogReading = 0;

  // Return type
  ret = 3;

  switch (pin)
  {
    case 20:
      analogReading = analogRead(A0);
      break;
    case 21:
      analogReading = analogRead(A1);
      break;
    case 22:
      analogReading = analogRead(A2);
      break;
    case 23:
      analogReading = analogRead(A3);
      break;
    case 24:
      analogReading = analogRead(A4);
      break;
    case 25:
      analogReading = analogRead(A5);
      break;
    default:
      break;
  }

  // send back results to simulink
  returnValnum(analogReading);
}

void exec_pwm(int pin, int value) {
  if (value < 0) value = 0;
  if (value > 100) value = 100;
  value = map(value, 0, 100, 0, 255);
  analogWrite(pin, value);
}

void exec_setPWMFreq(int pin, int value) {
  if (pin == 3) {
    // set pin 3 and pin 11 pwm to the desired frequency
    switch (value) {
      case 30:
        TCCR2B = (TCCR2B & CLEAR_LAST3_LSB) | FREQ_30HZ;
        break;
      case 122:
        TCCR2B = (TCCR2B & CLEAR_LAST3_LSB) | FREQ_122HZ;
        break;
      case 245:
        TCCR2B = (TCCR2B & CLEAR_LAST3_LSB) | FREQ_245HZ;
        break;
      case 490:
        TCCR2B = (TCCR2B & CLEAR_LAST3_LSB) | FREQ_490HZ;
        break;
      case 980:
        TCCR2B = (TCCR2B & CLEAR_LAST3_LSB) | FREQ_980HZ;
        break;
      case 4000:
        TCCR2B = (TCCR2B & CLEAR_LAST3_LSB) | FREQ_4KHZ;
        break;
      case 3200:
        TCCR2B = (TCCR2B & CLEAR_LAST3_LSB) | FREQ_32KHZ;
        break;
      default:
        break;
    }
  }
}

void exec_servoWrite(int pin, int value)
{
  if (sign == 1)
  {
    if (value < 0) value = 0;
    else if (value > 180) value = 180;
    if (pin == ardServoPin)
      myservo.write(value);
  }
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

void returnValnum(int val)
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

  // Add extra zeros if val is less than 1000
  if (val < 1000 && val >= 100) Serial.print("0");
  else if (val < 100 && val >= 10) Serial.print("00");
  else if (val < 10) Serial.print("000");

  // Saturate val to 9999 or 0
  if (val > 9999) Serial.print("9999");
  else if (val < 0) Serial.print("0000");
  else Serial.print(val);

  Serial.println(ret);
}

void readEncoder()
{
  sign = direction;   // sign to represent CW or CCW
  returnValnum(rpm);
}

void initEncoder() {
  // initialize interrupt pins 2 for reading encoder
  pinMode(encoderChA, INPUT_PULLUP);
  pinMode(encoderChB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderChA), updateEncoder, RISING);

  // Setup initial values for timer
  previousMillis = millis();
}

void updateEncoderReadings()
{
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    // Calculate RPM
    rpm = (float)(encoderValue * 60 / ENC_COUNT_REV);

    encoderValue = 0;
  }
}

void updateEncoder()
{
  // Increment value for each pulse from encoder
  encoderValue++;

  // check if going CW or CCW
  if (digitalRead(encoderChB))
    direction = CW;
  else
    direction = CCW;
}

void getSerial()
{
  ret = 5;  // Serial Info

  switch (SERIAL_CONFIG)
  {
    case SERIAL_5N1:
      returnVal("0501");
      break;
    case SERIAL_6N1:
      returnVal("0601");
      break;
    case SERIAL_7N1:
      returnVal("0701");
      break;
    case SERIAL_8N1:
      returnVal("0801");
      break;
    case SERIAL_5N2:
      returnVal("0502");
      break;
    case SERIAL_6N2:
      returnVal("0602");
      break;
    case SERIAL_7N2:
      returnVal("0702");
      break;
    case SERIAL_8N2:
      returnVal("0802");
      break;
    case SERIAL_5E1:
      returnVal("0511");
      break;
    case SERIAL_6E1:
      returnVal("0611");
      break;
    case SERIAL_7E1:
      returnVal("0711");
      break;
    case SERIAL_8E1:
      returnVal("0811");
      break;
    case SERIAL_5E2:
      returnVal("0512");
      break;
    case SERIAL_6E2:
      returnVal("0612");
      break;
    case SERIAL_7E2:
      returnVal("0712");
      break;
    case SERIAL_8E2:
      returnVal("0812");
      break;
    case SERIAL_5O1:
      returnVal("0521");
      break;
    case SERIAL_6O1:
      returnVal("0621");
      break;
    case SERIAL_7O1:
      returnVal("0721");
      break;
    case SERIAL_8O1:
      returnVal("0821");
      break;
    case SERIAL_5O2:
      returnVal("0522");
      break;
    case SERIAL_6O2:
      returnVal("0622");
      break;
    case SERIAL_7O2:
      returnVal("0722");
      break;
    case SERIAL_8O2:
      returnVal("0822");
      break;
    default:
      break;
  }

}

void initPins()
{
  for (unsigned int i = 0; i < sizeof(ardDioInPins); ++i) // Setup digital input pins
    pinMode(ardDioInPins[i], INPUT);
  for (unsigned int i = 0; i < sizeof(ardDioOutPins); ++i) // Setup digital output pins
    pinMode(ardDioOutPins[i], OUTPUT);
  myservo.attach(ardServoPin);                // Setup servo pin
}

void initSpi()
{
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  SPI.begin();
}

void dacWrite(unsigned long val)
{
  // If we get a value past the maximum value allowed
  if(val>1023)val=1023;
  
  // Map 0 - 4095 to 000 to FFF
  val = map(val, 0, 1023, 0x0000, 0x03FF);

  // Shift two bits left cuz we using a 10 bit DAC
  // Refer to write register in the datasheet
  val = (val << 2);

  unsigned long base = 0x1000;

  if (sign == 1)
  {
    val = base | val;

    digitalWrite(slaveSelectPin, LOW);
    SPI.transfer16(val);
    digitalWrite(slaveSelectPin, HIGH);
  }
}
