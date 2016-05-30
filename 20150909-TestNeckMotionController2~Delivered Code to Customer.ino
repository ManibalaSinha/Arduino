/**************************************************************************************************\

  Description:  Implementation of Neck Motion Controller (Disinfection Robot)
                ____________________________________________________________________________________

  Designed by:  Exceed Solutions Engineering Company
 Copyright by:  Exceed Solutions Engineering Company
                info@exceedsolutions.ca
                ____________________________________________________________________________________

  yyyymmdd-MA:  Represents Mike Akbari
  yyyymmdd-MM:  Represents Mohammad Mirzaalivandi
                ____________________________________________________________________________________
__________________________________________________________________________________________
  20150626-MM:  File created.
  20150816-MM:  Potentiometer functionality added.

\**************************************************************************************************/
//_________________________________________________________________________________________ Headers
#include <PinChangeInt.h>
//#include <PinChangeIntConfig.h>
#include <EEPROM.h>
//________________________________________________________ Pre-processor User Configurable Constants
//_________________________________________________________General Constants
#define MOT_DIRECTION_CW    1
#define MOT_DIRECTION_CCW   0
#define MOT_DIRECTION_STOP  2

//#define DEBUG_PRINT
#define DEMO_BOARD                    //Changes PWM signals for demo board connectivity. comment it for UNO board

#define DEMO_SWEEP
#define HOST_MESSAGE_DURATION_TIME    1500

// Pin Configuration
#ifdef DEMO_BOARD
#define MOT1_PIN_PWM_A  10
#define MOT1_PIN_PWM_B  9
#define MOT2_PIN_PWM_A  11
#define MOT2_PIN_PWM_B  3
#else //ARDUINO_UNO
#define MOT1_PIN_PWM_A  11
#define MOT1_PIN_PWM_B  3
#define MOT2_PIN_PWM_A  10
#define MOT2_PIN_PWM_B  9
#endif

#define MOT1_PIN_HALL_CH_A  2  //Before Change:#define MOT1_PIN_HALL_CH_A  6
//#define MOT1_PIN_HALL_CH_A  6
#define MOT1_PIN_HALL_CH_B  7
#define MOT2_PIN_HALL_CH_A  8
#define MOT2_PIN_HALL_CH_B  12

#define MOT1_PIN_TOP_SW       A3
#define MOT1_PIN_DOWN_SW      A2
#define MOT2_PIN_RIGHT_SW     A1  //right
#ifdef DEMO_BOARD
#define MOT2_PIN_MIDDLE_SW    5  //middle
#else
#define MOT2_PIN_MIDDLE_SW    13  //middle
#endif
#define MOT2_PIN_LEFT_SW      A0  //left
#define MOT2_POT_PIN_SWING_CNT       A5
#define MOT2_POT_PIN_ELEVATOR_POS    A4
#define MOT2_POT_PIN_ROTATION_SPEED  A6
// End of Pin Configuration
//_________________________________________________________Motor1 Related Constants
#define MOT1_EEPROM_POSITION_ADDR     1
#define MOT1_POT_ELEVATOR_POS_STEP    100
#define MOT1_POT_ELEVATOR_POS    1

#define MOT1_CW_HALL_RUN_DESIRED_PERIOD       5000 // MicroSeconds //50    //Milliseconds
#define MOT1_CW_HALL_START_DESIRED_PERIOD     5000 // Microseconds //18   //Milliseconds
#define MOT1_CW_HALL_PERIOD_MARGIN            50  // Microsrconds //5     //Milliseconds
#define MOT1_CW_PWM_UP_TUNE_STEP      15//10
#define MOT1_CW_PWM_DOWN_TUNE_STEP    1
#define MOT1_CW_PWM_STOP_STEP         10
#define MOT1_CW_PWM_DOWN_MIN          1
#define MOT1_CW_PWM_START_BASE_LINE   255 //255 //230    //PWM Power
#define MOT1_CW_PWM_RUN_VAL           255//250//230   //PWM Power
#define MOT1_CW_POSITION_UP           1750 //1650 //1500 //36//44 //39 //37
#define MOT1_CW_POSITION_UP_MIN       1000 //35
#define MOT1_CW_POSITION_UP_MAX       1800 //45
#define MOT1_CW_POSITION_UP_MARGIN    0
#define MOT1_CW_STARTUP_TIME          200//2000
#define MOT1_CW_TIPTOE_TIME           4000        //Milliseconds
#define MOT1_CW_SPEED_CONTROL_POS_OFFSET 0
#define MOT1_CW_SPEED_CONTROL_START_POS  (MOT1_CW_POSITION_UP - MOT1_CW_SPEED_CONTROL_POS_OFFSET)//38
#define MOT1_CW_STOP_STEP_DELAY    1       //Milliseconds

#define MOT1_CCW_HALL_RUN_DESIRED_PERIOD      5000 //Microseconds //50    //Milliseconds
#define MOT1_CCW_HALL_START_DESIRED_PERIOD    2000 //Microseconds //200   //Milliseconds
#define MOT1_CCW_HALL_PERIOD_MARGIN           50 //Microseconds//5     //Milliseconds
#define MOT1_CCW_PWM_START_STEP       5
#define MOT1_CCW_PWM_UP_TUNE_STEP     10
#define MOT1_CCW_PWM_DOWN_TUNE_STEP   2 //3
#define MOT1_CCW_PWM_STOP_STEP        10
#define MOT1_CCW_PWM_DOWN_MIN         1 //10 //1 //20
#define MOT1_CCW_PWM_START_BASE_LINE  255 //100 //90 //80      //PWM Power
#define MOT1_CCW_PWM_RUN_VAL    255//180   //PWM Power
#define MOT1_CCW_POSITION_DOWN        50 //5 //3  //Changed for Harmonic drive motor
#define MOT1_CCW_TIPTOE_TIME          4000        //Milliseconds
#define MOT1_CCW_STARTUP_TIME         150 //1500
#define MOT1_CCW_SLOW_DOWN_PWM        100
#define MOT1_CCW_STOP_STEP_DELAY   1       //Milliseconds

//_________________________________________________________Motor2 Related Constants
//#define MOT2_LEFT_MOVE_DURATION 5000//3000     //Milliseconds
//#define MOT2_RIGHT_MOVE_DURATION 5000//3000     //Milliseconds
#define MOT2_MIDDLE_POS_ADJAST_TIME 20
#define MOT2_SWING_PAUSE_TIME     300
#define MOT2_SWING_ADJUST_TIME    330 //270 //320
#define MOT2_SWING_MIN_PWM        20 //40//25
#define MOT2_SWING_MIDDLE_ADJUST_PWM     25
//#define MOT2_ROTATION_SPEED_ADC_STOP_VAL  20
#define MOT2_ROTATION_SPEED_ADC_MIN_VAL   600
#define MOT2_ROTATION_ANGLE_ADC_MIN_VAL   200
#define MOT2_ROTATION_ANGLE_MIN_CNT       60
//#define MOT2_ROTATION_ANGLE_ADC_MIN_VAL   200
//#define MOT2_ANGLE_MIN_CNT                60
#define MOT2_PWM_START_STEP 1
#define MOT2_PWM_TUNE_STEP  5
#define MOT2_PWM_STOP_STEP  5
#define MOT2_TIPTOE_TIME       15000        //Milliseconds
#define MOT2_POT_SWING_CNT       0
#define MOT2_POT_ROTATION_ANGLE  1
#define MOT2_POT_ROTATION_SPEED  2
#define MOT2_POT_ADC_READ_CNT        4
#define MOT2_POT_SWING_CNT_STEP       55
#define MOT2_POT_ROTATION_ANGLE_STEP  200
#define MOT2_POT_ROTATION_SPEED_STEP  25
#define MOT2_ROTATION_HALL_MUL        60

//__________________________________________________________________________ Pre-processor Constants
#define DEBUG_CODE        1     // Adds debug prints to firmware
// Please change it to zero for release version
#if     DEBUG_CODE
#define GET_MODE          '1'   // MPU Asks NMC module mode (CAL/RUN)
#define DEMO_START        '2'
#define DEMO_END          '3'
#define MOT1_MOV_CW       '4'   // MPU commands NMC to move motor1 clock wise
#define MOT1_MOV_CCW      '5'   // MPU commands NMC to move motor1 counter clock wise 
#define MOT2_MOV_CW       '6'   // MPU commands NMC to move motor2 clock wise
#define MOT2_MOV_CCW      '7'   // MPU commands NMC to move motor2 counter clock wise
#define MOT1_STOP         '8'   // MPU commands NMC to stop motor1
#define MOT2_STOP         '9'   // MPU commands NMC to stop motor2

#define MODE_CAL          '1'   // Answers GET_MODE & notifies CPU about NMC state (CAL) 
#define MODE_RUN          '2'   // Answers GET_MODE & notifies CPU about NMC state (RUN) 
#define MODE_IDL          '3'   // Answers GET_MODE & notifies CPU about NMC state (IDL) 
#define MOT1_CW_STARTED   '4'   // Declares MOT1_MOV_CW command is done  0x04
#define MOT1_CCW_STARTED  '5'   // Declares MOT1_MOV_CCW command is done 
#define MOT2_CW_STARTED   '6'   // Declares MOT2_MOV_CW command is done  
#define MOT2_CCW_STARTED  '7'   // Declares MOT2_MOV_CCW command is done 
#define MOT1_STOPPED      '8'   // Declares MOT1_STOP command is done  
#define MOT2_STOPPED      '9'   // Declares MOT2_STOP command is done  

#else                           // #if     DEBUG_CODE

#define GET_MODE          0x01  // MPU Asks NMC module mode (CAL/RUN) 
#define MOT1_MOV_CW       0x04  // MPU commands NMC to move motor1 clock wise
#define MOT1_MOV_CCW      0x05  // MPU commands NMC to move motor1 counter clock wise 
#define MOT2_MOV_CW       0x06  // MPU commands NMC to move motor2 clock wise
#define MOT2_MOV_CCW      0x07  // MPU commands NMC to move motor2 counter clock wise
#define MOT1_STOP         0x08  // MPU commands NMC to stop motor1
#define MOT2_STOP         0x09  // MPU commands NMC to stop motor2

#define MODE_CAL          0x01  // Answers GET_MODE & notifies CPU about NMC state (CAL) 
#define MODE_RUN          0x02  // Answers GET_MODE & notifies CPU about NMC state (RUN) 
#define MODE_IDL          0x03  // Answers GET_MODE & notifies CPU about NMC state (IDL) 
#define MOT1_CW_STARTED   0x04  // Declares MOT1_MOV_CW command is done  0x04
#define MOT1_CCW_STARTED  0x05  // Declares MOT1_MOV_CCW command is done 
#define MOT2_CW_STARTED   0x06  // Declares MOT2_MOV_CW command is done  
#define MOT2_CCW_STARTED  0x07  // Declares MOT2_MOV_CCW command is done 
#define MOT1_STOPPED      0x08  // Declares MOT1_STOP command is done  
#define MOT2_STOPPED      0x09  // Declares MOT2_STOP command is done 

#endif                          // #if     DEBUG_CODE


//__________________________________________________________________ Initialize the library instance

//_________________________________________________________________________________ Global Variables
int systemMode;

//_________________________________________________________Motor1 Related Variables
volatile int mot1PWMDutyCycle = 0;
int mot1Direction = MOT_DIRECTION_STOP;          // 1: CW , 0:CCW
int mot1StartCWFinishedFlag = false;
int mot1StartCCWFinishedFlag = false;
unsigned long mot1StartMilliTime = micros();//millis();
//int neckPositionCompleteIsOK = false;
int mot1NeckPositionUpIsOK = false;
int mot1NeckPositionDownIsOK = false;
//int neckPositionRightIsOK = false;
//int neckPositionMiddleIsOK = false;
//int neckPositionLeftIsOK = false;
//int OldNeckPositionCompleteIsOK = false;
//int OldNeckPositionUpIsOK = false;
//int OldNeckPositionDownIsOK = false;
//int OldNeckPositionRightIsOK = false;

//int OldNeckPositionMiddleIsOK = false;
//int OldNeckPositionLeftIsOK = false;
volatile int mot1LastPosition = 0;
int mot1OldPosition = 0;
volatile unsigned long mot1HallPulsePeriod = 1000000;//1000;
volatile unsigned long mot1OldHallMilliTime = micros();//millis();
//volatile unsigned int interruptMot1HallChACnt = 0;
//volatile unsigned int interruptMot1HallChBCnt = 0;
//volatile unsigned int oldInterruptMot1HallChACnt = 0;
//volatile unsigned int oldInterruptMot1HallChBCnt = 0;
//volatile unsigned int interruptMot1UpSwitchCnt = 0;
volatile unsigned int interruptMot1DownSwitchCnt = 0;
//volatile unsigned int oldInterruptMot1UpSwitchCnt = 0;
volatile unsigned int oldInterruptMot1DownSwitchCnt = 0;
volatile unsigned int mot1PotElevatorPosVal = 0;
unsigned int mot1ElevatorPosADCVal = 0;
//_________________________________________________________Motor2 Related Variables
volatile int mot2PWMDutyCycle = 0;
int mot2Direction = MOT_DIRECTION_STOP;          // 1: CW , 0:CCW
int mot2NeckSwingDone = false;

int mot2NeckPositionRightIsOK = false;
int mot2NeckPositionLeftIsOK = false;
volatile unsigned int mot2PotSwingCntVal = 0;
volatile unsigned int mot2PotRotationAngleVal = 0;
volatile unsigned int mot2PotRotationSpeedVal = 0;

volatile unsigned int mot2SwingCounter = 0;

unsigned int mot2SwingCntADCVal = 0;
unsigned int mot2RotationAngleADCVal = 0;
unsigned int mot2RotationSpeedADCVal = 0;
unsigned long mot2StartMilliTime = millis();
//unsigned long tempMillis = millis();


volatile int mot2LastPosition = 0;
int mot2OldPosition = 0;

volatile unsigned long mot2HallPulsePeriod = 1000;
volatile unsigned long mot2OldHallMilliTime = millis();


//volatile unsigned int interruptMot2HallChACnt = 0;
//volatile unsigned int interruptMot2HallChBCnt = 0;


//volatile unsigned int oldInterruptMot2HallChACnt = 0;
//volatile unsigned int oldInterruptMot2HallChBCnt = 0;

volatile unsigned int interruptMot2RightSwitchCnt = 0;
volatile unsigned int interruptMot2MiddleSwitchCnt = 0;
volatile unsigned int interruptMot2LeftSwitchCnt = 0;

volatile unsigned int oldInterruptMot2RightSwitchCnt = 0;
volatile unsigned int oldInterruptMot2MiddleSwitchCnt = 0;
volatile unsigned int oldInterruptMot2LeftSwitchCnt = 0;
//____________________________________________________________________________ Function Declarations

//_________________________________________________________Motor1 Related Functions
void interruptMot1HallChAFunc();
void interruptMot1HallChBFunc();
//void interruptMot1UpSwitchFunc();
void interruptMot1DownSwitchFunc();
void mot1StartCW();
void mot1StartCCW();
void mot1CWStop();
void mot1CCWStop();
void mot1MoveDurationProtect();
void mot1SpeedControl();
void mot1SetElevatorPos();

//_________________________________________________________Motor2 Related Functions
void interruptMot2HallChAFunc();
void interruptMot2HallChBFunc();
void interruptMot2RightSwitchFunc();
void interruptMot2MidSwitchFunc();
void interruptMot2LeftSwitchFunc();
void mot2MovCW();
void mot2MovCCW();
void mot2CWStop();
void mot2CCWStop();
void mot2MoveDurationProtect();
void  mot2SetSwingCnt();
void  mot2SetRotationSpeed();
void  mot2SetRotationAngle();


//_________________________________________________________General Functions
void setPWMFreq(int pwmPin, int divider);
unsigned int readPotentiometerVal(int desiredPotentiometer);


//___________________________________________________________________________________ Setup Function
void setup()
{
  //MOT1_LAST_POS_WRITE(5);
  //  systemMode = MODE_IDL;
  systemMode = MODE_RUN;
  Serial.begin(9600);				// initialize serial communications
#ifdef DEBUG_PRINT
#ifdef DEBUG_CODE
  Serial.println("*******************************");
  Serial.println("*Please Enter Command         *");
  Serial.println("*Please Start with GET_MODE   *");
  Serial.println("*Command       Val            *");
  Serial.println("*GET_MODE      1              *");
  Serial.println("*MOT1_MOV_CW   4              *");
  Serial.println("*MOT1_MOV_CCW  5              *");
  Serial.println("*MOT2_MOV_CW   6              *");
  Serial.println("*MOT2_MOV_CCW  7              *");
  Serial.println("*MOT1_STOP     8              *");
  Serial.println("*MOT2_STOP     9              *");
  Serial.println("*******************************");
#endif
#endif //#ifdef DEBUG_PRINT

  pinMode(MOT1_PIN_PWM_A , OUTPUT);
  pinMode(MOT1_PIN_PWM_B , OUTPUT);
  pinMode(MOT2_PIN_PWM_A , OUTPUT);
  pinMode(MOT2_PIN_PWM_B , OUTPUT);

  pinMode(MOT1_PIN_HALL_CH_A , INPUT_PULLUP);
  attachPinChangeInterrupt(MOT1_PIN_HALL_CH_A , interruptMot1HallChAFunc , FALLING);

  //  pinMode(MOT1_PIN_HALL_CH_A , INPUT_PULLUP);
  //  attachInterrupt(MOT1_PIN_HALL_CH_A , interruptMot1HallChAFunc , FALLING);

  pinMode(MOT1_PIN_HALL_CH_B , INPUT_PULLUP);
  // attachPinChangeInterrupt(MOT1_PIN_HALL_CH_B , interruptMot1HallChBFunc , FALLING);


  pinMode(MOT2_PIN_HALL_CH_A , INPUT_PULLUP);
  attachPinChangeInterrupt(MOT2_PIN_HALL_CH_A , interruptMot2HallChAFunc , FALLING);

  pinMode(MOT2_PIN_HALL_CH_B , INPUT_PULLUP);
  //attachPinChangeInterrupt(MOT2_PIN_HALL_CH_B , interruptMot2HallChBFunc , FALLING);

  //pinMode(MOT1_PIN_TOP_SW , INPUT_PULLUP);
  //attachPinChangeInterrupt(MOT1_PIN_TOP_SW , interruptMot1UpSwitchFunc , FALLING);

  pinMode(MOT1_PIN_DOWN_SW , INPUT_PULLUP);
  attachPinChangeInterrupt(MOT1_PIN_DOWN_SW , interruptMot1DownSwitchFunc , FALLING);

  pinMode(MOT2_PIN_RIGHT_SW , INPUT_PULLUP);
  attachPinChangeInterrupt(MOT2_PIN_RIGHT_SW , interruptMot2RightSwitchFunc , FALLING);

  pinMode(MOT2_PIN_MIDDLE_SW , INPUT_PULLUP);
  attachPinChangeInterrupt(MOT2_PIN_MIDDLE_SW , interruptMot2MiddleSwitchFunc , FALLING);

  pinMode(MOT2_PIN_LEFT_SW , INPUT_PULLUP);
  attachPinChangeInterrupt(MOT2_PIN_LEFT_SW , interruptMot2LeftSwitchFunc , FALLING);

  analogReference(DEFAULT);

  analogWrite(MOT1_PIN_PWM_A, 0);
  analogWrite(MOT1_PIN_PWM_B, 0);
  analogWrite(MOT2_PIN_PWM_A, 0);
  analogWrite(MOT2_PIN_PWM_B, 0);
  /**************************************************************************************************\
  Description:  Sets Pulse Width Modulation (PWM) frequency based on selected pins
  The divider parameter represents the prescaler divider (1, 8, 64, 256, or 1024)
  \**************************************************************************************************/
  //setPWMFreq(MOT1_PIN_PWM_A, 8);   // change Timer2 divider to 8 gives 3.9kHz PWM freq
  setPWMFreq(MOT1_PIN_PWM_A, 256);
  //setPWMFreq(MOT1_PIN_PWM_A, 1024);

  //setPWMFreq(MOT2_PIN_PWM_B, 8);   // change Timer1 divider to 8 gives 3.9kHz PWM freq
  setPWMFreq(MOT2_PIN_PWM_B, 256);
  //setPWMFreq(MOT2_PIN_PWM_B, 1024);

  while (!Serial)
  {
    ;       // wait for serial port to connect
  }
  mot1LastPosition = EEPROM.read(MOT1_EEPROM_POSITION_ADDR);

#ifdef DEBUG_PRINT
  Serial.print("Last Position from EEPROM is:");
  Serial.println(mot1LastPosition);

  Serial.print("Initiate Swing ADC Val: ");
  Serial.println(readPotentiometerVal(MOT2_POT_SWING_CNT));

  //  Serial.print("Initiate Elevator Position ADC Val: ");
  //  Serial.println(readPotentiometerVal(MOT1_POT_ELEVATOR_POS));

  Serial.print("Initiate Rotation Angle ADC Val: ");
  Serial.println(readPotentiometerVal(MOT2_POT_ROTATION_ANGLE));

  Serial.print("Initiate Rotation Speed ADC Val: ");
  Serial.println(readPotentiometerVal(MOT2_POT_ROTATION_SPEED));

#endif //DEBUG_PRINT

  mot2SetSwingCnt();
  mot1SetElevatorPos();
  mot2SetRotationSpeed();
  mot2SetRotationAngle();

#ifdef DEBUG_PRINT
  Serial.print("Initiate Swing Count: ");
  Serial.println(mot2PotSwingCntVal);

  Serial.print("Initiate Elevator Motor Position: ");
  Serial.println(mot1PotElevatorPosVal);

  Serial.print("Initiate Rotation Speed (PWM): ");
  Serial.println(mot2PotRotationSpeedVal);

  Serial.print("Initiate Rotation Angle (Hall Pulse): ");
  Serial.println(mot2PotRotationAngleVal);
#endif //DEBUG_PRINT
}
//____________________________________________________________________________________ Loop Function
void loop()
{
  while (!Serial)
  {
    ;       // wait for serial port to connect
  }
  mot2SetSwingCnt();
  mot1SetElevatorPos();
  mot2SetRotationSpeed();
  mot2SetRotationAngle();
  mot1MoveDurationProtect();
  mot2MoveDurationProtect();

  if ((mot1StartCWFinishedFlag == true))
  {
    if (mot1NeckPositionUpIsOK == false)
      Serial.println("1111111111111111");
    mot1SpeedControl();    //Commented for move down problem
  }

#ifdef DEMO_SWEEP
  if ((mot2NeckSwingDone == false) && (mot1NeckPositionUpIsOK == true))
  {
    unsigned long tempTime = millis();
    while (millis() - tempTime <= (HOST_MESSAGE_DURATION_TIME))
    {
      //if ( mot2PotSwingCntVal != 0)
      //{
      Serial.write(2);
      delay (2);
      //}
    }
  }
  if ((mot2NeckSwingDone == false) && (mot1NeckPositionUpIsOK == true) && (mot2PotSwingCntVal != 0))
  {
    unsigned long tempTime = millis();
    while (millis() - tempTime <= (3 * MOT2_SWING_PAUSE_TIME))
    {
      //if ( mot2PotSwingCntVal != 0)
      //{
      Serial.write(2);
      delay (2);
      //}
    }
    delay(2 * MOT2_SWING_PAUSE_TIME);
    detachPinChangeInterrupt(MOT2_PIN_MIDDLE_SW);
    for (int i = 0 ; i < mot2PotSwingCntVal ; i++)
      //while (mot2SwingCounter <= mot2PotSwingCntVal)
    {
      mot2SetSwingCnt();
      mot2SetRotationSpeed();
      mot2SetRotationAngle();
#ifdef DEBUG_PRINT
      Serial.print("Swing Count:");
      Serial.println(mot2PotSwingCntVal);

      Serial.print("Speed:");
      Serial.println(mot2PotRotationSpeedVal);

      Serial.print("Angle:");
      Serial.println(mot2PotRotationAngleVal);

      //Serial.print("1->");
      //Serial.println(mot2SwingCounter);
#endif //DEBUG_PRINT
      mot2MovCCW();
      while (mot2Direction == MOT_DIRECTION_CCW && (mot2LastPosition <= (int)(0 - mot2PotRotationAngleVal))); //mot2LastPosition must decrement here (has negative value)
      while (mot2LastPosition >= 0);
      while (((abs (mot2LastPosition)) < mot2PotRotationAngleVal));
      mot2CCWStop();
      delay(MOT2_SWING_PAUSE_TIME);
      mot2MovCW();
      while (mot2Direction == MOT_DIRECTION_CW && (mot2LastPosition >= mot2PotRotationAngleVal));
      while (mot2LastPosition <= 0);
      while (((abs (mot2LastPosition)) < mot2PotRotationAngleVal));
      mot2CWStop();
      delay(MOT2_SWING_PAUSE_TIME);
      //Serial.println(mot2PWMDutyCycle);
    }

    attachPinChangeInterrupt(MOT2_PIN_MIDDLE_SW , interruptMot2MiddleSwitchFunc , FALLING);
    mot2NeckSwingDone = true;
    delay(MOT2_SWING_PAUSE_TIME);
    mot2MovCCW();
    while ((digitalRead(MOT2_PIN_MIDDLE_SW) != 0))
    {
      delay(5);
    }
    //mot2CCWStop();
    analogWrite(MOT2_PIN_PWM_A, 0);
    analogWrite(MOT2_PIN_PWM_B, 0);
    delay(5 * MOT2_SWING_PAUSE_TIME);
    //---------------------------------------------------------------------------
    mot2Direction = MOT_DIRECTION_CCW;
    analogWrite(MOT2_PIN_PWM_A, 0);
    analogWrite(MOT2_PIN_PWM_B, MOT2_SWING_MIDDLE_ADJUST_PWM);
    delay(MOT2_SWING_ADJUST_TIME);
    analogWrite(MOT2_PIN_PWM_A, 0);
    analogWrite(MOT2_PIN_PWM_B, 0);
    mot2CCWStop();
    //---------------------------------------------------------------------------
    delay(3 * MOT2_SWING_PAUSE_TIME);
    //}
    if (digitalRead(MOT2_PIN_MIDDLE_SW) == 0)
    {
      mot1HallPulsePeriod = 1000000;//1000;
      mot1StartMilliTime = millis();
      mot1OldPosition = mot1LastPosition;
      mot1StartCCW();
      //mot2NeckSwingDone = true;
    }
  }
#endif
  if ((mot1StartCCWFinishedFlag == true))
  {
    Serial.println("3333333333333333");
    //mot1SpeedControl(); // Commented for move down problem
  }
  if ((mot1LastPosition >= MOT1_CW_POSITION_UP) && (mot1Direction == MOT_DIRECTION_CW))
  {
    mot1PWMDutyCycle = 0;
    mot1CWStop();
    mot1NeckPositionDownIsOK = false;
    mot1NeckPositionUpIsOK = true;
    //Serial.println("UP");
  }
  if ((mot1LastPosition <= MOT1_CCW_POSITION_DOWN) && (mot1Direction == MOT_DIRECTION_CCW))
  {
    do
    {
      analogWrite(MOT1_PIN_PWM_A, MOT1_CCW_SLOW_DOWN_PWM);
      mot1MoveDurationProtect();
    } while (((digitalRead(MOT1_PIN_DOWN_SW)) != LOW) && mot1NeckPositionDownIsOK == false);
    //mot1PWMDutyCycle = 0;
    mot1CCWStop();
    mot1NeckPositionDownIsOK = true;
    mot1NeckPositionUpIsOK = false;
  }
  if (Serial.available() > 0)
  {
    char command = Serial.read();
    switch (command)
    {
      case GET_MODE:
        if (systemMode == MODE_IDL) systemMode = MODE_RUN;
        //Serial.println(GET_MODE);
        break;
      case DEMO_START:
        //      1 - Check if the motor is up enough or not (check current position)
        //      2 - If the answer is NO then call  motor1 Start Clockwise function to move it to predefined position
        //      3 - Delay for suitable time
        //      4 - Start move left for suitable predefined time
        //      5 - Wait in loop for next command

        if (mot1LastPosition < (MOT1_CW_POSITION_UP - MOT1_CW_POSITION_UP_MARGIN))
        {
          mot1NeckPositionUpIsOK = false;
          mot1HallPulsePeriod = 1000000;//1000;
          mot1StartMilliTime = millis();
          mot1OldPosition  = mot1LastPosition ;
          mot2SwingCounter = 0;
          mot2LastPosition = 0;
          mot2OldPosition = 0;
          oldInterruptMot2MiddleSwitchCnt = 0;
          interruptMot2MiddleSwitchCnt = 0;
          mot1StartCW();
        }
        else
        {
          mot2LastPosition = 0;
          mot1NeckPositionUpIsOK = true;
        }
        mot2NeckSwingDone = false;
        break;
      case DEMO_END:
        break;
      case MOT1_MOV_CW:
        mot2NeckSwingDone = false;
        if (mot1LastPosition < (MOT1_CW_POSITION_UP - MOT1_CW_POSITION_UP_MARGIN))
        {
          mot1NeckPositionUpIsOK = false;
          mot1HallPulsePeriod = 1000000;//1000;
          mot1StartMilliTime = millis();
          mot1OldPosition  = mot1LastPosition ;
          mot1StartCW();
        }
        else
        {
          mot1NeckPositionUpIsOK = true;
        }
        break;
      case MOT1_MOV_CCW:
        mot1HallPulsePeriod = 1000000;//1000;
        mot1StartMilliTime = millis();
        mot1OldPosition = mot1LastPosition;
        mot1StartCCW();
        break;
      case MOT2_MOV_CW:
        mot2StartMilliTime = millis();
        mot2OldPosition = mot2LastPosition;
        mot2MovCW();
        break;
      case MOT2_MOV_CCW:
        mot2StartMilliTime = millis();
        mot2OldPosition = mot2LastPosition;
        mot2MovCCW();
        break;
      case MOT1_STOP:
        if (mot1Direction == MOT_DIRECTION_CW)
        {
          mot1CWStop();
        }
        else
        {
          mot1CCWStop();
        }
        break;
      case MOT2_STOP:
        if (mot2Direction == MOT_DIRECTION_CW)
        {
          mot2CWStop();
        }
        else
        {
          mot2CCWStop();
        }
        break;
      default:
        ;
        //Serial.println("Invalid Command");
    }
  }
  if ( mot1OldPosition  != mot1LastPosition )
  {
#ifdef DEBUG_PRINT
    Serial.print("P>");
    Serial.println(mot1LastPosition);
    Serial.print("D>");
    Serial.println(mot1PWMDutyCycle);
#endif
    mot1OldPosition = mot1LastPosition;
#ifdef DEBUG_PRINT
    Serial.print("S>");
    Serial.println(mot1HallPulsePeriod);
#endif
  }
  if ( mot2OldPosition  != mot2LastPosition )
  {
#ifdef DEBUG_PRINT
    Serial.print("M2P>");
    Serial.println(mot2LastPosition);
    //Serial.print("M2D>");
    //Serial.println(mot2PWMDutyCycle);
#endif
    mot2OldPosition = mot2LastPosition;
#ifdef DEBUG_PRINT
    Serial.print("M2S>");
    Serial.println(mot2HallPulsePeriod);
#endif
  }
  //  if ( oldInterruptMot1HallChBCnt != interruptMot1HallChBCnt)
  //  {
  //    //Serial.print("Hall ChA:");
  //    Serial.println(interruptMot1HallChBCnt);
  //    oldInterruptMot1HallChBCnt = interruptMot1HallChBCnt;
  //  }
//  if (oldInterruptMot1UpSwitchCnt != interruptMot1UpSwitchCnt)
//  {
//#ifdef DEBUG_PRINT
//    Serial.print("Mot1 Top Switch Interrupted :");
//    Serial.println(interruptMot1UpSwitchCnt);
//#endif  //#ifdef DEBUG_PRINT   
//    oldInterruptMot1UpSwitchCnt = interruptMot1UpSwitchCnt;
//  }
  if (oldInterruptMot1DownSwitchCnt != interruptMot1DownSwitchCnt)
  {
#ifdef DEBUG_PRINT
    Serial.print("Mot1 Down Switch Interrupted :");
    Serial.println(interruptMot1DownSwitchCnt);
#endif  //#ifdef DEBUG_PRINT     
    oldInterruptMot1DownSwitchCnt = interruptMot1DownSwitchCnt;
  }
  if (oldInterruptMot2RightSwitchCnt != interruptMot2RightSwitchCnt)
  {
#ifdef DEBUG_PRINT
    Serial.print("Mot2 Right Switch Interrupted :");
    Serial.println(interruptMot2RightSwitchCnt);
#endif
    oldInterruptMot2RightSwitchCnt = interruptMot2RightSwitchCnt;
  }
  if (oldInterruptMot2MiddleSwitchCnt != interruptMot2MiddleSwitchCnt)
  {
#ifdef DEBUG_PRINT
    Serial.print("Mot2 Middle Switch Interrupted :");
    Serial.println(interruptMot2MiddleSwitchCnt);
#endif
    oldInterruptMot2MiddleSwitchCnt = interruptMot2MiddleSwitchCnt;
  }
  if (oldInterruptMot2LeftSwitchCnt != interruptMot2LeftSwitchCnt)
  {
#ifdef DEBUG_PRINT
    Serial.print("Mot2 Left Switch Interrupted :");
    Serial.println(interruptMot2LeftSwitchCnt);
#endif
    oldInterruptMot2LeftSwitchCnt = interruptMot2LeftSwitchCnt;
  }
}
//_____________________________________________________________________________ Function Definitions
void interruptMot1HallChAFunc()
/**************************************************************************************************\
  Description:
\**************************************************************************************************/
{
  //Serial.println(".....");
  //mot1HallPulsePeriod = millis() - mot1OldHallMilliTime;
  mot1HallPulsePeriod = micros() - mot1OldHallMilliTime;
  //mot1OldHallMilliTime = millis();
  mot1OldHallMilliTime = micros();
  if (mot1Direction == MOT_DIRECTION_CW)
  {
    mot1LastPosition++;
  }
  else if (mot1Direction == MOT_DIRECTION_CCW)
  {
    mot1LastPosition--;
  }
}
void interruptMot1HallChBFunc()
/**************************************************************************************************\
  Description:
\**************************************************************************************************/
{
  //Serial.println("interruptMot1HallChBFunc");
  //if (digitalRead(MOT1_PIN_HALL_CH_B) == LOW)
  //  interruptMot1HallChBCnt++;
}
void interruptMot2HallChAFunc()
/**************************************************************************************************\
  Description:
\**************************************************************************************************/
{
  mot2HallPulsePeriod = millis() - mot2OldHallMilliTime;
  mot2OldHallMilliTime = millis();
  if (mot2Direction == MOT_DIRECTION_CW)
  {
    mot2LastPosition++;
  }
  else if (mot2Direction == MOT_DIRECTION_CCW)
  {
    mot2LastPosition--;
  }
#if 1
#ifdef DEBUG_PRINT
  Serial.print("1->");
  Serial.println(mot2LastPosition);
  Serial.print("2->");
  Serial.println(mot2SwingCounter);
  Serial.print("3->");
  Serial.println(mot2PotSwingCntVal);
#endif
#endif
}
void interruptMot2HallChBFunc()
/**************************************************************************************************\
  Description:
\**************************************************************************************************/
{
  //if (digitalRead(MOT2_PIN_HALL_CH_B) == LOW)
  //  interruptMot2HallChBCnt++;
}
//void interruptMot1UpSwitchFunc()
/**************************************************************************************************\
  Description:
\**************************************************************************************************/
//{
//  if (mot1Direction == MOT_DIRECTION_CW)
//  {
//    mot1CWStop();
//  }
//  else
//  {
    //mot1CCWStop();
//  }
//  oldInterruptMot1UpSwitchCnt = interruptMot1UpSwitchCnt;
//  interruptMot1UpSwitchCnt++;
//  EEPROM.write(MOT1_EEPROM_POSITION_ADDR, mot1LastPosition);
//}
void interruptMot1DownSwitchFunc()
/**************************************************************************************************\
  Description:
\**************************************************************************************************/
{
  mot1PWMDutyCycle = 0;
  if (mot1Direction == MOT_DIRECTION_CW)
  {
    //mot1PWMDutyCycle = 0;
    //mot1CWStop();
  }
  else
  {
    mot1PWMDutyCycle = 0;
    mot1CCWStop();
  }
#ifdef DEBUG_PRINT  
  oldInterruptMot1DownSwitchCnt = interruptMot1DownSwitchCnt;
  interruptMot1DownSwitchCnt++;
#endif //DEBUG_PRINT  
  mot1LastPosition = 0;
  EEPROM.write(MOT1_EEPROM_POSITION_ADDR, mot1LastPosition);
}
void interruptMot2RightSwitchFunc()
/**************************************************************************************************\
  Description: Right Position Limit Switch Interrupt Routine
\**************************************************************************************************/
{
  if (mot2Direction == MOT_DIRECTION_CW)
  {
    mot2CWStop();
  }
  else
  {
    //mot2CCWStop();
  }
#ifdef DEBUG_PRINT  
  oldInterruptMot2RightSwitchCnt = interruptMot2RightSwitchCnt;
  interruptMot2RightSwitchCnt++;
#endif  
}
void interruptMot2MiddleSwitchFunc()
/**************************************************************************************************\
  Description: Middle Position Limit Switch Interrupt Routine
\**************************************************************************************************/
{
  if (mot2SwingCounter > mot2PotSwingCntVal/*mot2NeckSwingDone == true*/)
  {
    if (mot2Direction == MOT_DIRECTION_CW)
    {
      //mot2CWStop();
    }
    else
    {
      mot2CCWStop();
    }
  }
  mot2LastPosition = 0;
  mot2SwingCounter++;
#ifdef DEBUG_PRINT  
  oldInterruptMot2MiddleSwitchCnt = interruptMot2MiddleSwitchCnt;
  interruptMot2MiddleSwitchCnt++;
#endif  
}
void interruptMot2LeftSwitchFunc()
/**************************************************************************************************\
  Description: Left Position Limit Switch Interrupt Routine
\**************************************************************************************************/
{
  if (mot2Direction == MOT_DIRECTION_CW)
  {
    mot2CWStop();
  }
  else
  {
    mot2CCWStop();
  }
#ifdef DEBUG_PRINT  
  oldInterruptMot2LeftSwitchCnt = interruptMot2LeftSwitchCnt;
  interruptMot2LeftSwitchCnt++;
#endif //DEBUG_PRINT  
}
void mot1SpeedControl()
/**************************************************************************************************\
  Description:
\**************************************************************************************************/
{
  if (mot1Direction == MOT_DIRECTION_CW && mot1LastPosition > MOT1_CW_SPEED_CONTROL_START_POS) //Second condition added for more power in lifting up in new robot
  {
    if (mot1HallPulsePeriod <= MOT1_CW_HALL_RUN_DESIRED_PERIOD - MOT1_CW_HALL_PERIOD_MARGIN)
    {
      if (mot1PWMDutyCycle <= MOT1_CW_PWM_DOWN_TUNE_STEP)
        mot1PWMDutyCycle = MOT1_CW_PWM_DOWN_MIN;
      else
        mot1PWMDutyCycle = mot1PWMDutyCycle - MOT1_CW_PWM_DOWN_TUNE_STEP;
    }
    else //if (mot1HallPulsePeriod > MOT1_CW_HALL_PULSE_DESIRED_PERIOD + MOT1_CW_HALL_PERIOD_MARGIN)
    {
      mot1PWMDutyCycle = mot1PWMDutyCycle + MOT1_CW_PWM_UP_TUNE_STEP;
      if (mot1PWMDutyCycle > MOT1_CW_PWM_RUN_VAL) mot1PWMDutyCycle = MOT1_CW_PWM_RUN_VAL;
    }
    analogWrite( MOT1_PIN_PWM_B , mot1PWMDutyCycle );
  }
  else if (mot1Direction == MOT_DIRECTION_CCW)
  {
    if (mot1HallPulsePeriod < MOT1_CCW_HALL_RUN_DESIRED_PERIOD - MOT1_CCW_HALL_PERIOD_MARGIN)
    {
      if (mot1PWMDutyCycle <= MOT1_CCW_PWM_DOWN_TUNE_STEP)
        mot1PWMDutyCycle = MOT1_CCW_PWM_DOWN_MIN;
      else
        mot1PWMDutyCycle = mot1PWMDutyCycle - MOT1_CCW_PWM_DOWN_TUNE_STEP;
    }
    else //if (mot1HallPulsePeriod >= (MOT1_CCW_HALL_PULSE_DESIRED_PERIOD + MOT1_CCW_HALL_PERIOD_MARGIN))
    {
      mot1PWMDutyCycle = mot1PWMDutyCycle + MOT1_CCW_PWM_UP_TUNE_STEP;
      if (mot1PWMDutyCycle > MOT1_CCW_PWM_RUN_VAL) mot1PWMDutyCycle = MOT1_CCW_PWM_RUN_VAL;
    }
    analogWrite( MOT1_PIN_PWM_A , mot1PWMDutyCycle);
  }
}
void mot1StartCW()
/**************************************************************************************************\
  Description: Starter for Motor 1 in Clockwise Direction
\**************************************************************************************************/
{
  //  if ( systemMode == MODE_RUN /*&& (digitalRead(MOT1_PIN_UP_SW))*/)
  //  {
  mot1Direction = MOT_DIRECTION_CW;
  analogWrite(MOT1_PIN_PWM_A, 0);
  mot1PWMDutyCycle = MOT1_CW_PWM_START_BASE_LINE;
  unsigned long tempTime = millis();

  //do
  //{
  analogWrite(MOT1_PIN_PWM_B, mot1PWMDutyCycle);
  delay (5);
  //  if (((millis() - tempTime) >= MOT1_CW_STARTUP_TIME))
  //  {
  //    break;
  //  }
  //}
  //while ( (mot1HallPulsePeriod >= MOT1_CW_HALL_START_DESIRED_PERIOD) /*&& (digitalRead(MOT1_PIN_UP_SW) != HIGH)*/);    //Based on Speed (Hall Period)
#ifdef DEBUG_PRINT
  Serial.print("WE :");
  Serial.println(mot1HallPulsePeriod);

#endif
  mot1StartCWFinishedFlag = true;
  //  }
}
void mot1StartCCW()
/**************************************************************************************************\
  Description: Starter for Motor 1 in Counterclockwise Direction
\**************************************************************************************************/
{
  //  if ( systemMode == MODE_RUN /*&& (digitalRead(MOT1_PIN_DOWN_SW))*/)
  //  {
  mot1Direction = MOT_DIRECTION_CCW;
  analogWrite(MOT1_PIN_PWM_B, 0);
  mot1PWMDutyCycle = MOT1_CCW_PWM_START_BASE_LINE;
  unsigned long tempTime = millis();
  analogWrite(MOT1_PIN_PWM_A, mot1PWMDutyCycle);
  //do
  //{
  //  if (((millis() - tempTime) >= MOT1_CCW_STARTUP_TIME))
  //  {
  //    break;
  //  }
  //}
  //while ( (mot1HallPulsePeriod >= MOT1_CCW_HALL_START_DESIRED_PERIOD)/* && (!((millis() - tempTime) >= 1000))*/); /*&& (digitalRead(MOT1_PIN_UP_SW) != HIGH)); */  //Based on Speed (Hall Period)
#ifdef DEBUG_PRINT
  Serial.print("CCW WE :");
  Serial.println(mot1HallPulsePeriod);
#endif
  mot1StartCCWFinishedFlag = true;
  //    neckPositionDownIsOK = true;
  //    neckPositionUpIsOK = false;
  //}
}
void mot2MovCW()
/**************************************************************************************************\
  Description: Starter for Motor 2 in Clockwise Direction
\**************************************************************************************************/
{
  if ( systemMode == MODE_RUN /*&& (digitalRead(MOT2_PIN_LEFT_SW))*/)
  {
    mot2Direction = MOT_DIRECTION_CW;
    analogWrite(MOT2_PIN_PWM_B, 0);
    for (mot2PWMDutyCycle = 0; mot2PWMDutyCycle <= mot2PotRotationSpeedVal ; mot2PWMDutyCycle += MOT2_PWM_START_STEP)
    {
      if (mot2PWMDutyCycle > mot2PotRotationSpeedVal) mot2PWMDutyCycle = mot2PotRotationSpeedVal;
      analogWrite(MOT2_PIN_PWM_A, mot2PWMDutyCycle);
    }
    analogWrite(MOT2_PIN_PWM_A, mot2PotRotationSpeedVal);
  }
}
void mot2MovCCW()
/**************************************************************************************************\
  Description: Starter for Motor 2 in Counterclockwise Direction
\**************************************************************************************************/
{
  if ( systemMode == MODE_RUN/* && (digitalRead(MOT2_PIN_RIGHT_SW))*/)
  {
    mot2Direction = MOT_DIRECTION_CCW;
    analogWrite(MOT2_PIN_PWM_A, 0);
    for (mot2PWMDutyCycle = 0; mot2PWMDutyCycle <= mot2PotRotationSpeedVal; mot2PWMDutyCycle += MOT2_PWM_START_STEP)
    {
      if (mot2PWMDutyCycle > mot2PotRotationSpeedVal) mot2PWMDutyCycle = mot2PotRotationSpeedVal;
      analogWrite(MOT2_PIN_PWM_B, mot2PWMDutyCycle);
    }
    analogWrite(MOT2_PIN_PWM_B, mot2PotRotationSpeedVal);
  }
}
void mot1CWStop()
/**************************************************************************************************\
  Description: Stops Motor 1 when It Turns in Clockwise Direction
\**************************************************************************************************/
{
  if (mot1PWMDutyCycle >= MOT1_CW_PWM_STOP_STEP)
  {
    for (mot1PWMDutyCycle = mot1PWMDutyCycle ; mot1PWMDutyCycle >= 0; mot1PWMDutyCycle -= MOT1_CW_PWM_STOP_STEP)
    {
      analogWrite(MOT1_PIN_PWM_B, mot1PWMDutyCycle);
      delay(MOT1_CW_STOP_STEP_DELAY);
    }
  }
  analogWrite(MOT1_PIN_PWM_A, 0);
  analogWrite(MOT1_PIN_PWM_B, 0);
  EEPROM.write(MOT1_EEPROM_POSITION_ADDR, mot1LastPosition);
  mot1PWMDutyCycle = 0;
  mot1Direction = MOT_DIRECTION_STOP;
  mot1StartCWFinishedFlag = false;
  mot1NeckPositionDownIsOK = false;
  if (mot1LastPosition >= (MOT1_CW_POSITION_UP - MOT1_CW_POSITION_UP_MARGIN))
  {
    mot1NeckPositionUpIsOK = true;
  }

  // tempMillis = millis();
  //Serial.println(MOT1_STOP);
}
void mot1CCWStop()
/**************************************************************************************************\
 Description: Stops Motor 1 when It Turns in Counterclockwise Direction
\**************************************************************************************************/
{
  if (mot1PWMDutyCycle >= MOT1_CCW_PWM_STOP_STEP)
  {
    for (mot1PWMDutyCycle = mot1PWMDutyCycle ; mot1PWMDutyCycle >= 0; mot1PWMDutyCycle -= MOT1_CCW_PWM_STOP_STEP)
    {
      analogWrite(MOT1_PIN_PWM_A, mot1PWMDutyCycle);
      delay(MOT1_CCW_STOP_STEP_DELAY);
    }
  }
  analogWrite(MOT1_PIN_PWM_A, 0);
  analogWrite(MOT1_PIN_PWM_B, 0);
  EEPROM.write(MOT1_EEPROM_POSITION_ADDR, mot1LastPosition);
  mot1PWMDutyCycle = 0;
  mot1Direction = MOT_DIRECTION_STOP;
  mot1StartCCWFinishedFlag = false;
  mot1NeckPositionDownIsOK = true;
  if (mot1LastPosition < (MOT1_CW_POSITION_UP - MOT1_CW_POSITION_UP_MARGIN))
  {
    mot1NeckPositionUpIsOK = false;
  }
  //Serial.println(MOT1_STOP);
}
void mot2CWStop()
/**************************************************************************************************\
  Description: Stops Motor 2 when It Turns in Clockwise Direction
\**************************************************************************************************/
{
#if 0
  if (mot2PWMDutyCycle >= MOT2_PWM_STOP_STEP)
  {
    for (mot2PWMDutyCycle = mot2PWMDutyCycle ; mot2PWMDutyCycle >= 0; mot2PWMDutyCycle -= MOT2_PWM_STOP_STEP)
    {
      analogWrite(MOT2_PIN_PWM_A, mot2PWMDutyCycle);
    }
  }
#endif
  analogWrite(MOT2_PIN_PWM_A, 0);
  analogWrite(MOT2_PIN_PWM_B, 0);
  mot2PWMDutyCycle = 0;
  mot2Direction = MOT_DIRECTION_STOP;
  mot2NeckPositionRightIsOK = true;
  //mot2StartCWFinishedFlag = false;
  //if (mot1LastPosition < (MOT1_CW_POSITION_UP - MOT1_CW_POSITION_UP_MARGIN))
  //{
  //  mot1NeckPositionUpIsOK = false;
  //}
}
void mot2CCWStop()
/**************************************************************************************************\
  Description: Stops Motor 2 when It Turns in Counterclockwise Direction
\**************************************************************************************************/
{
#if 0
  if (mot2PWMDutyCycle >= MOT2_PWM_STOP_STEP)
  {
    for (mot2PWMDutyCycle = mot2PWMDutyCycle; mot2PWMDutyCycle >= 0; mot2PWMDutyCycle -= MOT2_PWM_STOP_STEP)
    {
      analogWrite(MOT2_PIN_PWM_B, mot2PWMDutyCycle);
    }
  }
#endif
  analogWrite(MOT2_PIN_PWM_A, 0);
  analogWrite(MOT2_PIN_PWM_B, 0);
  mot2PWMDutyCycle = 0;
  mot2Direction = MOT_DIRECTION_STOP;
  mot2NeckPositionLeftIsOK = true;
  //mot2StartCCWFinishedFlag = false;
  //if (mot1LastPosition < (MOT1_CW_POSITION_UP - MOT1_CW_POSITION_UP_MARGIN))
  //{
  //  mot1NeckPositionUpIsOK = false;
  //}
}
void mot1MoveDurationProtect()
/**************************************************************************************************\
  Description:
\**************************************************************************************************/
{
  if (mot1Direction == MOT_DIRECTION_CW)
  {
    if ((millis() - mot1StartMilliTime) >= MOT1_CW_TIPTOE_TIME)
    {
      mot1CWStop();
    }
  }
  else if (mot1Direction == MOT_DIRECTION_CCW)
  {
    if ((millis() - mot1StartMilliTime) >= MOT1_CCW_TIPTOE_TIME)
    {
      mot1CCWStop();
    }
  }
}
void mot2MoveDurationProtect()
/**************************************************************************************************\
  Description:
\**************************************************************************************************/
{
  if (mot2Direction == MOT_DIRECTION_CW)
  {
    if ((millis() - mot2StartMilliTime) >= MOT2_TIPTOE_TIME)
    {
      mot2CWStop();
    }
  }
  else if (mot2Direction == MOT_DIRECTION_CCW)
  {
    if ((millis() - mot2StartMilliTime) >= MOT2_TIPTOE_TIME)
    {
      mot2CCWStop();
    }
  }
}
void setPWMFreq(int pwmPin, int divider)
/**************************************************************************************************\
  Description:  Sets Pulse Width Modulation (PWM) frequency based on selected pins
The divider parameter represents the prescaler divider (1, 8, 64, 256, or 1024)
\**************************************************************************************************/
{
  byte timerCtrlReg;
  if (pwmPin == 5 || pwmPin == 6 || pwmPin == 9 || pwmPin == 10) // Timer0 or Timer1
  {
    switch (divider) {
      case 1:
        timerCtrlReg = 0x01;
        break;
      case 8:
        timerCtrlReg = 0x02;
        break;
      case 64:
        timerCtrlReg = 0x03;
        break;
      case 256:
        timerCtrlReg = 0x04;
        break;
      case 1024:
        timerCtrlReg = 0x05;
        break;
      default: return;
    }
    if (pwmPin == 5 || pwmPin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | timerCtrlReg; // Timer0
    } else {
      TCCR1B = TCCR1B & 0b11111000 | timerCtrlReg; // Timer1
    }
  } else if (pwmPin == 3 || pwmPin == 11)
  {
    switch (divider) {
      case 1:
        timerCtrlReg = 0x01;
        break;
      case 8:
        timerCtrlReg = 0x02;
        break;
      case 32:
        timerCtrlReg = 0x03;
        break;
      case 64:
        timerCtrlReg = 0x04;
        break;
      case 128:
        timerCtrlReg = 0x05;
        break;
      case 256:
        timerCtrlReg = 0x06;
        break;
      case 1024:
        timerCtrlReg = 0x7;
        break;
      default:
        return;
    }
    TCCR2B = TCCR2B & 0b11111000 | timerCtrlReg; // Timer2
  }
}
unsigned int readPotentiometerVal(int desiredPotentiometer)
/**************************************************************************************************\
  Description:
\**************************************************************************************************/
{
  unsigned int readVal = 0;
  unsigned int tempSum = 0;
  switch (desiredPotentiometer)
  {
    case MOT2_POT_SWING_CNT:
      for (int i = 1 ; i <= MOT2_POT_ADC_READ_CNT ; i++)
        tempSum = analogRead(MOT2_POT_PIN_SWING_CNT) + tempSum;
      break;
    case MOT1_POT_ELEVATOR_POS:
      for (int i = 1 ; i <= MOT2_POT_ADC_READ_CNT ; i++)
        tempSum = analogRead(MOT2_POT_PIN_ELEVATOR_POS) + tempSum;
      break;
    case MOT2_POT_ROTATION_SPEED:
      for (int i = 1 ; i <= MOT2_POT_ADC_READ_CNT ; i++)
        tempSum = analogRead(MOT2_POT_PIN_ROTATION_SPEED) + tempSum;
      break;
  }
  readVal = tempSum / MOT2_POT_ADC_READ_CNT;
  return (readVal);
}
void mot2SetSwingCnt()
/**************************************************************************************************\
  Description:
\**************************************************************************************************/
{
  unsigned int swingCntADCVal;
  swingCntADCVal = readPotentiometerVal(MOT2_POT_SWING_CNT);
  mot2PotSwingCntVal = (int)(swingCntADCVal / MOT2_POT_SWING_CNT_STEP);
}
void mot1SetElevatorPos()
/**************************************************************************************************\
  Description:
\**************************************************************************************************/
{
#if 0
  unsigned int elevatorPosADCVal;
  elevatorPosADCVal = readPotentiometerVal(MOT1_POT_ELEVATOR_POS);
  mot1PotElevatorPosVal = ((int)(elevatorPosADCVal / MOT1_POT_ELEVATOR_POS_STEP)) + MOT1_CW_POSITION_UP_MIN;
  if (mot1PotElevatorPosVal > MOT1_CW_POSITION_UP_MAX)
    mot1PotElevatorPosVal = MOT1_CW_POSITION_UP_MAX;
#endif
  mot1PotElevatorPosVal = MOT1_CW_POSITION_UP;
}
void mot2SetRotationSpeed()
/**************************************************************************************************\
  Description:
\**************************************************************************************************/
{
  unsigned int rotationSpeedADCVal;
  rotationSpeedADCVal = readPotentiometerVal(MOT2_POT_ROTATION_SPEED);
  mot2PotRotationSpeedVal = ((int)(rotationSpeedADCVal / MOT2_POT_ROTATION_SPEED_STEP));
  //if (rotationSpeedADCVal <= MOT2_ROTATION_SPEED_ADC_STOP_VAL)  mot2PotRotationSpeedVal = 0;
  //if ((rotationSpeedADCVal > MOT2_ROTATION_SPEED_ADC_STOP_VAL) && (rotationSpeedADCVal < MOT2_ROTATION_SPEED_ADC_MIN_VAL))  mot2PotRotationSpeedVal = MOT2_SWING_MIN_PWM;
  if (rotationSpeedADCVal < MOT2_ROTATION_SPEED_ADC_MIN_VAL)  mot2PotRotationSpeedVal = MOT2_SWING_MIN_PWM;
}
void  mot2SetRotationAngle()
/**************************************************************************************************\
  Description:
\**************************************************************************************************/
{
  unsigned int rotationAngleADCVal;
  rotationAngleADCVal = readPotentiometerVal(MOT2_POT_ROTATION_ANGLE);
  mot2PotRotationAngleVal = MOT2_ROTATION_HALL_MUL * ((int)(rotationAngleADCVal / MOT2_POT_ROTATION_ANGLE_STEP));
  if (rotationAngleADCVal <= MOT2_ROTATION_ANGLE_ADC_MIN_VAL)  mot2PotRotationAngleVal = MOT2_ROTATION_ANGLE_MIN_CNT;
}


