/**************************************************************************************************\

  Description:  Implementation of LCD Controller Firmware
                ____________________________________________________________________________________

  Designed by:  Exceed Solutions Engineering Company
  Copyright by: Exceed Solutions Engineering Company
                info@exceedsolutions.ca
                ____________________________________________________________________________________

  yyyymmdd-MA:  Represents Mike Akbari
  yyyymmdd-MM:  Represents Mohammad Mirzaalivandi
                ____________________________________________________________________________________
  __________________________________________________________________________________________
  20160116-MM:  Start to modification of project to debug known issues discussed with Mike.

  \**************************************************************************************************/
//__________________________________________________________________________________________ Headers


#include <genieArduino.h>
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <SPI.h>
#include <Ethernet.h>
#include "Wire.h"
#include <EEPROM.h>
#include <RH_NRF24.h>

//________________________________________________________ Pre-processor User Configurable Constants
//_________________________________________________________________________________ Global Variables
#define DOOR_SENSOR_IS_ALIVE_INTERVAL   1000 // Milliseconds
bool flagDoorSensorIsAlive = false;
unsigned long doorSensorIsAliveTime  = 0;
unsigned long previousDoorSensorIsAliveTime = 0;
#define FLASH_DRIVER_COMMUNICATION_DELAY        200

//____________________________________________________________________________ Function Declarations
//___________________________________________________________________________________ Setup Function
//____________________________________________________________________________________ Loop Function
//_____________________________________________________________________________ Function Definitions
/**************************************************************************************************\
  Description:
  \**************************************************************************************************/
// define TIME_PROCESS 30000
//================ Ethernet Definition ======================
// Network configuration (Gateway and subnet are optional)

// The media access control (Ethernet hardware) address for the shield
#include "Arduino.h"
void setup();
void loop();
void getWireless(void);         //151223 - Added by Mohammad as a compile time error solution
void calculatorKey(int key);    //151223 - Added by Mohammad as a compile time error solution
void checkNeckInput();          //151223 - Added by Mohammad as a compile time error solution
void myGenieEventHandler(void);
void updateNewVal(void);
void return_old_dis1_val(void);
void return_old_dis2_val(void);
void runCont(void);
void runInter(void);
void runInter_Cont(void);
void startFlash(void);
void stopFlash(void);
void sendSetting(void);
void alarmCheck(void);
byte checkSumOfMessage(byte array[], int messageSize);
void checkClient();
void parseCommand();
void answerClient(EthernetClient client);
void getDateDs1307();
byte bcdToDec(byte val);
void checkConsoleInput();
//#line 11
byte mac[] = {0x90, 0xA2, 0xDA, 0x0F, 0x8C, 0x5F };
// The IP address for the shield
byte ip[] = {192, 168, 2, 5};
//byte ip[]={192,168,20,217};
// The router's gateway address
byte gateway[] = {174, 114, 141, 41};
//byte gateway[]={192,168,1,1};
//byte gateway[]={192,168,20,1};
// The subnet
byte subnet[] = {255, 255, 255, 0};

// Telnet defaults to port 23
EthernetServer server = EthernetServer(80);

// Clock
int clockAddress = 0x68;  // This is the I2C address
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;

char c;
String stringReceived;
String commandReceived;
String token;
boolean flagDataReceived = false, flagRunning = false;
long int timeStartRunning;

//int workingFrequency,workingVoltage,runTime,pauseTime,workModeSwitch;  //decleared by luis
int roomNumber;
boolean arm_flag, roomNumberConfirm_flag, reportReceived_flag = false;
//long int totalRunTime=0;  //decleared by luis
int stopType = 0;
//boolean start_flag=false,stop_flag=false; //decleared by luis
String startYear, startMonth, startDay, startHour, startMinute, startSecond;
String stopYear, stopMonth, stopDay, stopHour, stopMinute, stopSecond;

//-----------------------------------------
//=========================== LCD & Phase I Definitions ==========================
Genie genie;
#define RESETLINE 4  // Change this if you are not using an Arduino Adaptor Shield Version 2 (see code below)
#define DEFAULT_FEQUENCY  (EEPROM.read(0))//3
#define DEFAULT_VOLT (EEPROM.read(1))//20
#define DEFAULT_RUN (EEPROM.read(2))//10
#define DEFAULT_PAUSE (EEPROM.read(3))//0
#define DEFAULT_WORKING_MODE (EEPROM.read(4))//1
#define RUN 1
#define STOP 0
#define YES 1
#define NO 0
//#define R_U_READY 11
int ledPin = 13;

//================Global variables =============
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis2 = 0;
unsigned long currentRunTime = 0;
int numberOfCycles = 0;
float flashDuration = 0;
//=================keyboard variables
// Calculator globals
double acc ;
double memory ;
unsigned int display = 0 ;
int    lastOperator ;
int    errorCondition  ;
int keyboardValue;
boolean start_new_number_flag = true ;
boolean enter_pressed_flag = false;
boolean clear_pressed_flag = false;
//==================Function Prototypes ========
//byte checkSumOfMessage(byte array [] ,int );


//==================== flags ===================
boolean start_flag = STOP; // Added for start flag come from App
boolean run_flag = STOP;
boolean run_pressed_flag = NO;
boolean stop_flag = STOP;
boolean stop_pressed_flag = NO;
boolean pause_flag = NO;
//boolean arm_flag = NO;    // defined as part of network flags
boolean neck_ok_flag = NO;
boolean old_neck_ok_flag = YES;
boolean door_ok_flag = NO;
boolean old_door_ok_flag = YES;
boolean stop_door_sensor_flag = YES;//NO;
boolean informed_flag = NO;
boolean first_time_neck_command = YES;

//========== Setting Variable list  =============
unsigned int workingFrequency = DEFAULT_FEQUENCY;        //Leddigits0
unsigned int workingVoltage = DEFAULT_VOLT;              //Leddigits1
boolean voltSwitch = 0;
long int totalRunTime = 0;                      //Leddigits2
long runTime = DEFAULT_RUN;
long pauseTime = DEFAULT_PAUSE;
int workModeSwitch = DEFAULT_WORKING_MODE;
boolean cont = true;
boolean inter = false;
boolean inter_cont = false;

//.............
//==================================================
//======== Message Structure ===========
byte startMessage[6] = {0xaa, 0x55, 0x03, 0x04, 0x53, 0x59};
byte stopMessage[6] = {0xaa, 0x55, 0x03, 0x05, 0x45, 0x4c};
byte settingMessage[32] = {0xaa, 0x55, 0x1d, 0x01, 0x02, 0x01, 0x5a, 0x00,
                           0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0a,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x90
                          };

byte messageRecieved[32] = {0xaa, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                           };
//==========Alert messages ============================
/*
  aa 55 03 07 /04/ 0d             Alert packet
  alert_sign=	04 	//abnormal discharge
  alert_sign= 01; // over current alarm communication
  alert_sign= 02; // overheat alarm parameters
*/

//========== Updated Variable list from defaults =============
unsigned int updated_workingFrequency = DEFAULT_FEQUENCY;
unsigned int updated_workingVoltage = DEFAULT_VOLT;
long int updated_totalRunTime = 0;
unsigned int updated_runTime = DEFAULT_RUN;
unsigned int updated_pauseTime = DEFAULT_PAUSE;
int updated_workModeSwitch = DEFAULT_WORKING_MODE;
boolean updated_cont = false;//true;
boolean updated_inter = true;//false;
boolean updated_inter_cont = false;

//.............
//=======================================================
//========== RF Radio ================
// Singleton instance of the radio driver
RH_NRF24 nrf24(8, 53);
// RH_NRF24 nrf24(8, 7); // use this to be electrically compatible with Mirf
// RH_NRF24 nrf24(8, 10);// For Leonardo, need explicit SS pin
// RH_NRF24 nrf24(8, 7); // For RFM73 on Anarduino Mini
//===========================================================
//unsigned int tempFlashStopCounter = 0;
unsigned long firstDoorOpenDetectTime = 0;
#define DOOR_SENSOR_DETECT_TIME     5000
//===========================================================
void setup()
{
  pinMode(ledPin, OUTPUT);      // sets the digital pin as output
  // NOTE, the genieBegin function (e.g. genieBegin(GENIE_SERIAL_0, 115200)) no longer exists.  Use a Serial Begin and serial port of your choice in
  // your code and use the genie.Begin function to send it to the Genie library (see this example below)
  // 200K Baud is good for most Arduinos. Galileo should use 115200.
  Serial.begin(9600);  // Serial0 @ 9600  Baud
  Serial1.begin(9600);  // Serial1 @ 9600  Baud for Communication with Neck motor driver
  Serial2.begin(9600);  //Serial @ 9600 Baud for communication with Flash driver
  Serial3.begin(9600);  // Serial3 @ 9600  Baud for communicating with LCD //
  Serial.println("Setup Started!");
  //================= RF radio Initialization ==================
  if (!nrf24.init())
    Serial.println("RF init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");
  //....for I2C..............
  Wire.begin();
  //...................
  // Initialize the Ethernet device
  // Ethernet.begin(mac,ip,gateway,subnet);
  Ethernet.begin(mac);
  // Start listing for clients
  server.begin();
  //Serial.print("Arduino-Ethernet is on: ");
  Serial.println(Ethernet.localIP());
  //..............................
  // Initialize the Genie devise
  genie.Begin(Serial3);   // Use Serial3 for talking to the Genie Library, and to the 4D Systems display

  genie.AttachEventHandler(myGenieEventHandler); // Attach the user function Event Handler for processing events

  // Reset the Display (change D4 to D2 if you have original 4D Arduino Adaptor)
  // THIS IS IMPORTANT AND CAN PREVENT OUT OF SYNC ISSUES, SLOW SPEED RESPONSE ETC
  // If NOT using a 4D Arduino Adaptor, digitalWrites must be reversed as Display Reset is Active Low, and
  // the 4D Arduino Adaptors invert this signal so must be Active High.
  pinMode(RESETLINE, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  digitalWrite(RESETLINE, 1);  // Reset the Display via D4
  delay(100);
  digitalWrite(RESETLINE, 0);  // unReset the Display via D4

  delay (3500); //let the display start up after the reset (This is important)

  //Turn the Display on (Contrast) - (Not needed but illustrates how)
  genie.WriteContrast(1); // 1 = Display ON, 0 = Display OFF.
  //For uLCD43, uLCD-70DT, and uLCD-35DT, use 0-15 for Brightness Control, where 0 = Display OFF, though to 15 = Max Brightness ON.

  //Write a string to the Display to show the version of the library used
  //genie.WriteStr(0, GENIE_VERSION);
  //genie.WriteStr(0, "powerd by Exceed Solution");

  calculatorKey ('a') ;	// Clear the calculator

  //------------- Loading default values to the display
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, workingFrequency);    // Write workingFrequency value to to LED Digits 0
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x01, workingVoltage);    // Write workingVoltage value to LED Digits 1
  //.................
  if (workModeSwitch == 0)
  {
    genie.WriteObject(GENIE_OBJ_LED, 0x00, true);    // Write inter value to LED0
    genie.WriteObject(GENIE_OBJ_LED, 0x01, false);    // Write inter value to LED1
    genie.WriteObject(GENIE_OBJ_LED, 0x02, false);    // Write inter value to LED2
  }
  else if (workModeSwitch == 1)
  {
    genie.WriteObject(GENIE_OBJ_LED, 0x00, false);    // Write inter value to LED0
    genie.WriteObject(GENIE_OBJ_LED, 0x01, true);    // Write inter value to LED1
    genie.WriteObject(GENIE_OBJ_LED, 0x02, false);    // Write inter value to LED2
  }
  else
  {
    genie.WriteObject(GENIE_OBJ_LED, 0x00, false);    // Write inter value to LED0
    genie.WriteObject(GENIE_OBJ_LED, 0x01, false);    // Write inter value to LED1
    genie.WriteObject(GENIE_OBJ_LED, 0x02, true);    // Write inter value to LED2
  }
  //......................
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x05, runTime);    // Write runTime value to to LED Digits 5
  genie.WriteObject(GENIE_OBJ_SLIDER, 0x00, runTime);    // Write runTime value to to Slider 0
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x06, pauseTime);    // Write pauseTime value to to LED Digits 6
  genie.WriteObject(GENIE_OBJ_SLIDER, 0x01, pauseTime);    // Write pauseTime value to to Slider 1
  genie.WriteObject(GENIE_OBJ_ROTARYSW, 0x00, workModeSwitch);    // Write pauseTime value to to Rotary Switch
  genie.WriteObject(GENIE_OBJ_KNOB, 0x01, workingFrequency);    // Write pauseTime value to to Rotary Switch
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x04, workingFrequency);    // Write workingFrequency value to to LED Digits 4
  if (workingVoltage == 20)
    genie.WriteObject(GENIE_OBJ_4DBUTTON, 0x02, true);    // Write workingVoltage value of 2.0KV to 4D Button 2
  else
    genie.WriteObject(GENIE_OBJ_4DBUTTON, 0x02, false);    // Write workingVoltage value of 2.5KV to 4D Button 2
  //============= sending setting meessage of Default values once after power up  ====================

  settingMessage[22] = updated_workingFrequency;
  if (updated_workingVoltage == 20)
    settingMessage[30] = 1 ;
  if (updated_workingVoltage == 25)
    settingMessage[30] = 2 ;

  settingMessage[31] = checkSumOfMessage(settingMessage, 32);
  Serial2.flush();
  delay (FLASH_DRIVER_COMMUNICATION_DELAY);
  sendSetting();
  Serial.println("Flash driver config sent - 1.");
  delay (FLASH_DRIVER_COMMUNICATION_DELAY);
  Serial2.flush();
  delay (FLASH_DRIVER_COMMUNICATION_DELAY);
  sendSetting();
  Serial.println("Flash driver config sent - 2.");
  Serial.println("Setup Finished!");

  //=========================   ====================  ==================================================
}

void loop()
{
  static long waitPeriod = millis();
  static int gaugeAddVal = 1;
  static int gaugeVal = 50;
  //............. Updating Message que .............
  settingMessage[22] = updated_workingFrequency;
  if (updated_workingVoltage == 20)
    settingMessage[30] = 1 ;
  if (updated_workingVoltage == 25)
    settingMessage[30] = 2 ;

  settingMessage[31] = checkSumOfMessage(settingMessage, 32);
  //................................
  // Check for client
  commandReceived = "";
  checkClient();
  doorSensorIsAliveTime = millis();
  if (doorSensorIsAliveTime - previousDoorSensorIsAliveTime >= DOOR_SENSOR_IS_ALIVE_INTERVAL) 
  {
    // save the last time you blinked the LED
    previousDoorSensorIsAliveTime = doorSensorIsAliveTime;
    flagDoorSensorIsAlive = false;
  }
  getWireless();
  //if (informed_flag == false)
  // send request again
  //Serial1.write(1);
  //Serial1.print(1);
  //Serial.println("R U READY");
  //delay(25);  // in test progress and willl decide later if i need it
  checkNeckInput();
  //.................................

  genie.DoEvents(); // This calls the library each loop to process the queued responses from the display
  //int led_val = 70;
  //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, led_val);
  //if (millis() >= waitPeriod)
  //{
  // Write to CoolGauge0 with the value in the gaugeVal variable
  //genie.WriteObject(GENIE_OBJ_COOL_GAUGE, 0x00, gaugeVal);
  //gaugeVal += gaugeAddVal;
  //if (gaugeVal == 99) gaugeAddVal = -1;
  //if (gaugeVal == 0) gaugeAddVal = 1;

  // The results of this call will be available to myGenieEventHandler() after the display has responded
  // Do a manual read from the UserLEd0 object
  //genie.ReadObject(GENIE_OBJ_USER_LED, 0x00);

  //updateNewVal();
  //delay(2000);
  //Serial.print("Sum");

  //Serial.println(totalRunTime);
  //Serial.println(settingMessage[31]);
  //Serial.println(stop_pressed_flag);
  // Serial.println(run_pressed_flag);
  //Serial.println(updated_workModeSwitch);
  //Serial.println(updated_pauseTime);
  //Serial.println(updated_runTime);
  //Serial.println(updated_cont);
  //Serial.println(updated_inter);
  // Serial.println(updated_inter_cont);
  //Serial.println(" ");
  //waitPeriod = millis() + 50; // rerun this code to update Cool Gauge and Slider in another 50ms time.
  //}
  alarmCheck();

  //  if (door_ok_flag && neck_ok_flag)
  //  {
  //    old_neck_ok_flag = neck_ok_flag;
  //    old_door_ok_flag = door_ok_flag;

  if (updated_workModeSwitch == 0)
    runCont();
  else if (updated_workModeSwitch == 1)
    runInter();
  else if (updated_workModeSwitch == 2)
    runInter_Cont();
  else
  {
  }
  //}
  //else if (old_door_ok_flag || old_neck_ok_flag)
  //{
  //  old_neck_ok_flag = false;
  //  old_door_ok_flag = false;
  //  //stopFlash();  //160118 Commented by Mohammad
  //  stop_pressed_flag = YES; //
  //  first_time_neck_command = YES;  // when the swing is finished starting parking of the head, motor controller is sending "space" and our neck_flag gets false and for sure program comes here and we change the first_time_print_flag here
  //
  //
  //  //stop_door_sensor_flag = YES;.
  //  //run_flag = NO;
  //  //run_pressed_flag = NO;
  //  //stop_flag = false;
  //  //stopType = 0;
  //
  //  if (updated_workModeSwitch == 0)
  //    runCont();
  //  else if (updated_workModeSwitch == 1)
  //    runInter();
  //  else if (updated_workModeSwitch == 2)
  //    runInter_Cont();
  //  else {}
  //  genie.WriteObject(GENIE_OBJ_FORM, 0x0b, 0x0b);   // Go to Form 11
  //  //delay(1000);
  //}
  //else
  //{
  if ( run_pressed_flag == YES && first_time_neck_command == YES && flagDoorSensorIsAlive == true)
  {
    //FSerial1.print("4");
    Serial1.print("2");     // liftup the head
    Serial.println("Lift up Command Sent.");
    first_time_neck_command = NO;
  }
  //}

  alarmCheck();
}

/////////////////////////////////////////////////////////////////////
//
// This is the user's event handler. It is called by genieDoEvents()
// when the following conditions are true
//
//		The link is in an IDLE state, and
//		There is an event to handle
//
// The event can be either a REPORT_EVENT frame sent asynchronously
// from the display or a REPORT_OBJ frame sent by the display in
// response to a READ_OBJ request.
//

/* COMPACT VERSION HERE, LONGHAND VERSION BELOW WHICH MAY MAKE MORE SENSE
  void myGenieEventHandler(void)
  {
  genieFrame Event;
  int slider_val = 0;
  const int index = 0;  //HARD CODED TO READ FROM Index = 0, ie Slider0 as an example

  genieDequeueEvent(&Event);

  //Read from Slider0 for both a Reported Message from Display, and a Manual Read Object from loop code above
  if (genieEventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_SLIDER, index) ||
    genieEventIs(&Event, GENIE_REPORT_OBJ,   GENIE_OBJ_SLIDER, index))
  {
    slider_val = genieGetEventData(&Event);  // Receive the event data from the Slider0
    genieWriteObject(GENIE_OBJ_LED_DIGITS, 0x00, slider_val);     // Write Slider0 value to to LED Digits 0
  }
  } */

// LONG HAND VERSION, MAYBE MORE VISIBLE AND MORE LIKE VERSION 1 OF THE LIBRARY
void myGenieEventHandler(void)
{
  genieFrame Event;
  genie.DequeueEvent(&Event);

  //mike //int slider_val = 0;
  //If the cmd received is from a Reported Event (Events triggered from the Events tab of Workshop4 objects)
  if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
  {
    //.........................This is for All keyboard Objects.............................
    if (Event.reportObject.object == GENIE_OBJ_KEYBOARD) // If this event is from a Keyboard
    {
      //............ This is for keyboaerd 1 ....Password entery..................
      if (Event.reportObject.index == 0)	// If from Keyboard0
      {
        keyboardValue = genie.GetEventData(&Event); // Get data from Keyboard0
        calculatorKey(keyboardValue); // pass data to the calculatorKey function for processing
        if (enter_pressed_flag)
        {
          enter_pressed_flag = false;
          // Serial.println (display);
          if (display == 8888)
            genie.WriteObject(GENIE_OBJ_FORM, 0x02, 0x02);   // Go to Form 2
          start_new_number_flag = true;
          display = 0;
        }
        else
        {}

      }

      //..........
      //............ This is for keyboaerd 1 ...............Frequency entery.......
      if (Event.reportObject.index == 1)	// If from Keyboard1
      {
        keyboardValue = genie.GetEventData(&Event); // Get data from Keyboard1
        calculatorKey(keyboardValue); // pass data to the calculatorKey function for processing
        if (enter_pressed_flag)
        {
          enter_pressed_flag = false;
          if (display > 50)
            display = 50;
          workingFrequency = display;
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, workingFrequency);    // Write workingFrequency value to to LED Digits 0
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x04, workingFrequency);    // Write workingFrequency value to to LED Digits 4
          genie.WriteObject(GENIE_OBJ_KNOB, 0x01, workingFrequency);    // Write workingFrequency value to knob 1
          EEPROM.write(0, workingFrequency);        // writting working frequency to the Address 0 of the EEPROM
          genie.WriteObject(GENIE_OBJ_FORM, 0x04, 0x04);   // Go to Form 11
          start_new_number_flag = true;
          display = 0;
        }
        else
        {}
      }
      //............
      //............ This is for keyboaerd 2 ................Run time entery......
      if (Event.reportObject.index == 2)	// If from Keyboard2
      {
        keyboardValue = genie.GetEventData(&Event); // Get data from Keyboard2
        calculatorKey(keyboardValue); // pass data to the calculatorKey function for processing
        if (enter_pressed_flag)
        {
          enter_pressed_flag = false;
          if (display > 2700)
            display = 2700;
          runTime = display;
          genie.WriteObject(GENIE_OBJ_SLIDER, 0x00, runTime);    // Write runTime value to to Slider0
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x05, runTime);    // Write workingFrequency value to to LED Digits 4
          EEPROM.write(2, runTime);        // writting runTime to the Address 1 of the EEPROM
          genie.WriteObject(GENIE_OBJ_FORM, 0x05, 0x05);   // Go to Form 5
          start_new_number_flag = true;
          display = 0;
        }
        else
        {}

      }
      //.............
      //............ This is for keyboaerd 3 ................Pause time entery......
      if (Event.reportObject.index == 3)	// If from Keyboard3
      {
        keyboardValue = genie.GetEventData(&Event); // Get data from Keyboard3
        calculatorKey(keyboardValue); // pass data to the calculatorKey function for processing
        if (enter_pressed_flag)
        {
          enter_pressed_flag = false;
          if (display > 100)
            display = 100;
          pauseTime = display;
          genie.WriteObject(GENIE_OBJ_SLIDER, 0x01, pauseTime);    // Write runTime value to to Slider0
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x06, pauseTime);    // Write workingFrequency value to to LED Digits 4
          EEPROM.write(3, pauseTime);        // writting runTime to the Address 1 of the EEPROM
          genie.WriteObject(GENIE_OBJ_FORM, 0x05, 0x05);   // Go to Form 5
          start_new_number_flag = true;
          display = 0;
        }
        else
        {}

      }
      //...........

    }
    //.......................
    /* if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)                // If the Reported Message was from a winbutton
      {
       if (Event.reportObject.index == 17   )                              // If Winbutton 17 (Keyboard for frequency)
       {

         genie.WriteObject(GENIE_OBJ_FORM, 0x0c,0x0c);    // Go to Form12 to enter the frequency value from keyboard
       }
      }
      //......................
      //.......................
      if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)                // If the Reported Message was from a winbutton
      {
       if (Event.reportObject.index == 16   )                              // If Winbutton 16 (Keyboard for run time)
       {

         genie.WriteObject(GENIE_OBJ_FORM, 0x0d,0x0d);    // Go to Form 13 to enter the run time value from keyboard
       }
      }
      //......................
      //.......................
      if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)                // If the Reported Message was from a winbutton
      {
       if (Event.reportObject.index == 18   )                              // If Winbutton 17 (Keyboard for pause time)
       {

         genie.WriteObject(GENIE_OBJ_FORM, 0x0e,0x0e);    // Go to Form 13 to enter the pause time value from keyboard
       }
      }
      //...................... */

    //............................
    if (Event.reportObject.object == GENIE_OBJ_KNOB)                // If the Reported Message was from a Knob
    {
      if (Event.reportObject.index == 1)                              // If Knob1
      {
        workingFrequency = genie.GetEventData(&Event);                      // Receive the event data from the Knob1
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, workingFrequency);    // Write workingFrequency value to to LED Digits 0
        EEPROM.write(0, workingFrequency);        // writting working frequency to the Address 0 of the EEPROM
      }
    }
    //......................
    if (Event.reportObject.object == GENIE_OBJ_4DBUTTON)                // If the Reported Message was from a 4D Button (2.0kv / 2.5Kv)
    {
      if (Event.reportObject.index == 2)                              // If 4Dbutton2
      {
        // int voltSwitch = 0;
        voltSwitch = genie.GetEventData(&Event);                      // Receive the event data from the 4DButon
        if (voltSwitch == 0)                                          // check the switch and update the voltage value for display on LED Digits 1
        {
          workingVoltage = 25;
          EEPROM.write(1, workingVoltage);        // writting workingVoltage to the Address 1 of the EEPROM
        }
        else
        {
          workingVoltage = 20;
          EEPROM.write(1, workingVoltage);        // writting workingVoltage to the Address 1 of the EEPROM
        }
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x01, workingVoltage);    // Write workingVoltage value to LED Digits 1
      }
    }
    //.......................
    if (Event.reportObject.object == GENIE_OBJ_SLIDER)                // If the Reported Message was from a Slider
    {
      if (Event.reportObject.index == 0)                              // If Slider0
      {
        runTime = genie.GetEventData(&Event);                      // Receive the event data from the Knob1
        EEPROM.write(2, runTime);        // writting runTime to the Address 1 of the EEPROM
        //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x05, runTime);    // Write runTime value to to LED Digits 0
      }
    }
    //.......................
    if (Event.reportObject.object == GENIE_OBJ_SLIDER)                // If the Reported Message was from a Slider
    {
      if (Event.reportObject.index == 1)                              // If Slider1
      {
        pauseTime = genie.GetEventData(&Event);                      // Receive the event data from the Knob1
        EEPROM.write(3, pauseTime);        // writting pauseTime to the Address 1 of the EEPROM
        //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x06, pauseTime);    // Write pauseTime value to to LED Digits 0
      }
    }
    //......................
    if (Event.reportObject.object == GENIE_OBJ_ROTARYSW)                // If the Reported Message was from a Rotary Switch [Cont/Inter/(Cont/Inter)]
    {
      if (Event.reportObject.index == 0)                              // If ROTARYSW0
      {
        //workModeSwitch = 0;
        workModeSwitch = genie.GetEventData(&Event);                      // Receive the event data from the Rotary Switch
        EEPROM.write(4, workModeSwitch);        // writting workModeSwitch to the Address 1 of the EEPROM
        if (workModeSwitch == 0)                                          // check the switch and update the voltage value for display on LED Digits 1
        { cont = true;                            //Led0
          inter = false;                           //Led1
          inter_cont = false;                      //Led2
          genie.WriteObject(GENIE_OBJ_LED, 0x00, cont);    // Write cont value to LED0
          genie.WriteObject(GENIE_OBJ_LED, 0x01, inter);    // Write inter value to LED1
          genie.WriteObject(GENIE_OBJ_LED, 0x02, inter_cont);    // Write inter_cont value to LED2
        }
        else if   (workModeSwitch == 1)                                          // check the switch and update the voltage value for display on LED Digits 1
        { cont = false;                            //Led0
          inter = true;                           //Led1
          inter_cont = false;                      //Led2
          genie.WriteObject(GENIE_OBJ_LED, 0x00, cont);    // Write cont value to LED0
          genie.WriteObject(GENIE_OBJ_LED, 0x01, inter);    // Write inter value to LED1
          genie.WriteObject(GENIE_OBJ_LED, 0x02, inter_cont);    // Write inter_cont value to LED2
        }
        else
        { cont = false;                            //Led0
          inter = false;                           //Led1
          inter_cont = true;                      //Led2
          genie.WriteObject(GENIE_OBJ_LED, 0x00, cont);    // Write cont value to LED0
          genie.WriteObject(GENIE_OBJ_LED, 0x01, inter);    // Write inter value to LED1
          genie.WriteObject(GENIE_OBJ_LED, 0x02, inter_cont);    // Write inter_cont value to LED2
        }
      }
    }
    //.......................
    if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)                // If the Reported Message was from a Slider
    {
      if (Event.reportObject.index == 7 || Event.reportObject.index == 9  )                              // If Winbutton 7
      {
        //boolean display1_ok = genie.GetEventData(&Event);                      // Receive the event data from the Winbutton 7
        updateNewVal();
        genie.WriteObject(GENIE_OBJ_FORM, 0x03, 0x03);   // Go to Form3
      }
    }
    //......................
    //.......................
    if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)                // If the Reported Message was from a Winbutton
    {
      if (Event.reportObject.index == 8   )                              // If Winbutton 8 (Back)
      {
        //boolean display1_ok = genie.GetEventData(&Event);                      // Receive the event data from the Winbutton 10 (Back)
        return_old_dis1_val();
        genie.WriteObject(GENIE_OBJ_FORM, 0x03, 0x03);   // Go to Form3
      }
    }
    //......................
    //.......................
    if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)                // If the Reported Message was from a Winbutton
    {
      if (Event.reportObject.index == 10  )                              // If Winbutton 10 (Back)
      {
        //boolean display1_ok = genie.GetEventData(&Event);                      // Receive the event data from the Winbutton 10 (Back)
        return_old_dis2_val();
        genie.WriteObject(GENIE_OBJ_FORM, 0x03, 0x03);   // Go to Form3
      }
    }
    //......................
    //.......................
    if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)                // If the Reported Message was from a Winbutton
    {
      if (Event.reportObject.index == 3  )                              // If Winbutton 3 (Back)
      {
        //boolean display1_ok = genie.GetEventData(&Event);                      // Receive the event data from the Winbutton 10 (Back)
        run_pressed_flag = YES;
        stop_pressed_flag = NO;
        genie.WriteObject(GENIE_OBJ_FORM, 0x06, 0x06);   // Go to Form6
      }
    }
    //......................
    //......................
    if (Event.reportObject.object == GENIE_OBJ_4DBUTTON)                // If the Reported Message was from a 4D Button (2.0kv / 2.5Kv)
    {
      if (Event.reportObject.index == 0 || Event.reportObject.index == 1)                             // If 4Dbutton2
      {
        // int voltSwitch = 0;
        stop_pressed_flag = YES;
        run_pressed_flag = NO;
        //genie.WriteObject(GENIE_OBJ_FORM, 0x02,0x02);    // Go to Form2
      }
    }
    //.......................
    //.......................
    if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)                // If the Reported Message was from a win button
    {
      if (Event.reportObject.index == 6   )                              // If Winbutton 6 (Back)
      {
        //boolean display1_ok = genie.GetEventData(&Event);                      // Receive the event data from the Winbutton 10 (Back)
        sendSetting();
        genie.WriteObject(GENIE_OBJ_FORM, 0x02, 0x02);   // Go to Form3
      }
    }
    //......................
  }

  //If the cmd received is from a Reported Object, which occurs if a Read Object (genie.ReadOject) is requested in the main code, reply processed here.
  if (Event.reportObject.cmd == GENIE_REPORT_OBJ)
  {
    if (Event.reportObject.object == GENIE_OBJ_USER_LED)              // If the Reported Message was from a User LED
    {
      if (Event.reportObject.index == 0)                              // If UserLed0
      {
        bool UserLed0_val = genie.GetEventData(&Event);               // Receive the event data from the UserLed0
        UserLed0_val = !UserLed0_val;                                 // Toggle the state of the User LED Variable
        genie.WriteObject(GENIE_OBJ_USER_LED, 0x00, UserLed0_val);    // Write UserLed0_val value back to to UserLed0
      }
    }
  }

}

//=================================== other Functions ===========================
void updateNewVal(void)
{
  updated_workingFrequency = workingFrequency;
  updated_workingVoltage = workingVoltage;
  //updated_totalRunTime =0;

  updated_cont = cont;
  updated_inter = inter;
  updated_inter_cont = inter_cont;
  updated_runTime = runTime;
  updated_pauseTime = pauseTime;
  updated_workModeSwitch = workModeSwitch ;

}

void return_old_dis1_val(void)
{
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, updated_workingFrequency);    // Write old workingFrequency value back to LED Digits 0
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x01, updated_workingVoltage);    // Write old workingFrequency value back to LED Digits 1
  genie.WriteObject(GENIE_OBJ_KNOB, 0x01, updated_workingFrequency);    // Write old workingFrequency value back to Knob 1
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x04, updated_workingFrequency);    // Write old workingFrequency value back to LED Digits 4
  genie.WriteObject(GENIE_OBJ_4DBUTTON, 0x02, !voltSwitch);    // Write workingVoltage value of 2.0KV to 4D Button 2

}

void return_old_dis2_val(void)
{
  genie.WriteObject(GENIE_OBJ_LED, 0x00, updated_cont);    // Write cont value to LED0
  genie.WriteObject(GENIE_OBJ_LED, 0x01, updated_inter);    // Write inter value to LED1
  genie.WriteObject(GENIE_OBJ_LED, 0x02, updated_inter_cont);    // Write inter_cont value to LED2
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x05, updated_runTime);    // Write old runTime value back to LED Digits 5
  genie.WriteObject(GENIE_OBJ_SLIDER, 0x00, updated_runTime);    // Write old runTime value back to Slider 0
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x06, updated_pauseTime);    // Write old pauseTime value back to LED Digits 6
  genie.WriteObject(GENIE_OBJ_SLIDER, 0x01, updated_pauseTime);    // Write old pauseTime value back to Slider 1
  genie.WriteObject(GENIE_OBJ_ROTARYSW, 0x00, updated_workModeSwitch);    // Write old working mode value back to Rotary Switch

}

void runCont(void)
{
  if (run_flag == STOP && run_pressed_flag == YES && neck_ok_flag == YES && stop_door_sensor_flag == NO)
  { startFlash();
    run_pressed_flag = NO;
    run_flag = RUN;
    //neck_ok_flag == NO;

    currentMillis = millis();
    previousMillis = currentMillis;

  }
  else if (run_flag == RUN && (stop_door_sensor_flag == YES || stop_pressed_flag == YES))
  {
    stopFlash();
    first_time_neck_command = YES;
    flashDuration = updated_workingFrequency * 0.016 * totalRunTime ;
    //Serial.println(flashDuration);
    stop_flag = true;
    stopType = 2;
    //stop_door_sensor_flag =  NO;
    stop_pressed_flag = NO;
    run_flag = STOP;
    genie.WriteObject(GENIE_OBJ_FORM, 0x02, 0x02);   // Go to Form2
  }
  else if (run_flag == RUN && stop_pressed_flag == NO)
  { currentMillis = millis();
    totalRunTime = (currentMillis - previousMillis) / 1000;
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x02, totalRunTime);    // Write Total Run Time value back to LED Digits 2
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x03, totalRunTime);    // Write Total Run Time value back to LED Digits 3
  }
  else {}
}

//=======================
void runInter(void)
{

  if (run_flag == STOP && run_pressed_flag == YES && neck_ok_flag == YES && stop_door_sensor_flag == NO)
  {
    startFlash();
    run_pressed_flag = NO;
    run_flag = RUN;
    //neck_ok_flag = NO;
    currentMillis = millis();
    previousMillis = currentMillis;
  }
  else if ((stop_door_sensor_flag == NO && stop_pressed_flag == NO) && run_flag == RUN && (((millis()) - previousMillis) <= runTime * 1000))

  {
    totalRunTime = (((millis()) - previousMillis)) / 1000;
    //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x02, totalRunTime);    // Write Total Run Time value back to LED Digits 2
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x03, totalRunTime);
  }
  else if ((stop_door_sensor_flag == YES || stop_pressed_flag == YES) || (run_flag == RUN && (((millis()) - previousMillis) > runTime * 1000)))
    //else if (stop_pressed_flag==YES ||(run_flag == RUN &&(((millis())- previousMillis)/1000) > runTime))   ///Old time decision which not working
  {
    // stopFlash();
    first_time_neck_command = YES;
    if ((stop_door_sensor_flag == YES || stop_pressed_flag == YES) && run_flag == RUN)
    {
      stop_flag = true;
      stopType = 2;

    }
    else
    {
      stop_flag = true;
      stopType = 1;
    }    //then stop is caused by door sensor and stoptype will remain stoptype =1  (from door check in the loop)

    //..................

    if (run_flag == RUN)  //if door sensor moved  time should not be updated
      totalRunTime = (((millis()) - previousMillis)) / 1000; ///Need for correct timing count
    //Serial.println(totalRunTime); // for debugging the iming issue
    flashDuration = updated_workingFrequency * 0.016 * totalRunTime ;
    //Serial.println(flashDuration);
    if (stop_door_sensor_flag == YES) stop_door_sensor_flag =  NO; //160229: MMA
    stop_pressed_flag = NO;
    run_pressed_flag = NO;
    run_flag = STOP;
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x02, totalRunTime);    // Write Total Run Time value back to LED Digits 2
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x03, totalRunTime);    // Write Total Run Time value back to LED Digits 3
    genie.WriteObject(GENIE_OBJ_FORM, 0x02, 0x02);   // Go to Form2
    //tempFlashStopCounter = tempFlashStopCounter + 1;
    firstDoorOpenDetectTime = (millis());
    stopFlash();    //moved from top for discarding 1 second additional count on deisplay
  }
  else
  {
  }
}

//==========================================================================
void runInter_Cont(void)
{
  if (run_flag == STOP && run_pressed_flag == YES && neck_ok_flag == YES && stop_door_sensor_flag == NO)
  { genie.WriteObject(GENIE_OBJ_FORM, 0x06, 0x06);   // Go to Form7
    startFlash();
    run_pressed_flag = NO;
    run_flag = RUN;
    //neck_ok_flag = NO;
    pause_flag = NO;
    currentMillis = millis();
    previousMillis = currentMillis;
    numberOfCycles++  ;
  }
  else if (stop_pressed_flag == NO && run_flag == RUN && (((millis()) - previousMillis) <= runTime * 1000))

  {

    totalRunTime = (((millis()) - previousMillis)) / 1000;
    //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x02, (currentRunTime));// Write Total Run Time(total Run time * Number of the cycles) value back to LED Digits 2
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x03, totalRunTime);    // Write Total Run Time value back to LED Digits 3
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x07, numberOfCycles);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x08, totalRunTime);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x09, numberOfCycles);
  }
  else if (pause_flag == NO && run_flag == RUN && (((millis()) - previousMillis) > runTime * 1000)) //stop_door_sensor_flag == YES
  {
    stopFlash();
    //first_time_neck_command = YES;
    totalRunTime = (((millis()) - previousMillis)) / 1000; //this step has no effect on functionality and jus updates displat value for totalruntime. if omit, time will be displayed 1 sec less than actual

    stop_pressed_flag = NO;
    run_pressed_flag = NO;
    pause_flag = YES;
    run_flag = STOP; //mike
    //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x02, (currentRunTime));    // Write Total Run Time(total Run time * Number of the cycles) value back to LED Digits 2
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x03, totalRunTime);    // Write Total Run Time value back to LED Digits 3
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x07, numberOfCycles);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x08, totalRunTime);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x09, numberOfCycles);
    genie.WriteObject(GENIE_OBJ_FORM, 0x07, 0x07);   // Go to Form7
    currentMillis = millis();
    previousMillis2 = currentMillis;

  }

  else if (stop_pressed_flag == YES && run_flag == RUN )
  {
    stopFlash();
    //first_time_neck_command = YES;
    currentRunTime = (numberOfCycles - 1) * runTime + totalRunTime;
    numberOfCycles = 0;
    //totalRunTime = (((millis())- previousMillis))/1000;
    //currentRunTime=currentRunTime*numberOfCycles;
    stop_pressed_flag = NO;
    run_pressed_flag = NO;
    run_flag = STOP;
    pause_flag = NO;
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x02, (int (currentRunTime)));    // Write Total Run Time(total Run time * Number of the cycles) value back to LED Digits 2
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x03, totalRunTime);    // Write Total Run Time value back to LED Digits 3
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x07, numberOfCycles);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x08, totalRunTime);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x09, numberOfCycles);
    genie.WriteObject(GENIE_OBJ_FORM, 0x02, 0x02);   // Go to Form2

    currentRunTime = 0;
    totalRunTime = 0;
  }
  else if (run_flag == STOP && pause_flag == YES && run_pressed_flag == NO && (((millis()) - previousMillis2) >= pauseTime * 1000))
  {
    run_flag == STOP ;
    run_pressed_flag = YES;
    stop_pressed_flag = NO;
    pause_flag = NO;
    //Serial.println(run_pressed_flag);   //for debugging

  }

  else {}
}

//=======================
void startFlash(void)
{
  //digitalWrite(ledPin, HIGH);   // sets the LED on
  Serial2.write(startMessage, sizeof(startMessage));
  //Serial1.write(1); //20151229
}

//=======================
void stopFlash(void)
{
  Serial2.write(stopMessage, sizeof(stopMessage));
  //Serial1.write(2); // 20151229
  delay(1000);
  //Serial.print("tempFlashStopCounter: ");
  //Serial.println(tempFlashStopCounter);
  Serial.println("stopFlash Occured");
  if (updated_workModeSwitch == 0 || updated_workModeSwitch == 1)
  {
    Serial1.print("3");  // this is added by mike to send A park command to Motor driver after flash stops
    neck_ok_flag = false;
    Serial.println("Stop sent");   //for debugging
  }
  // 160120 - Commented Temp it has to be uncomment and fixed to handle int-cnt mode
  //  if (updated_workModeSwitch == 2 && stop_pressed_flag == YES)
  //  {
  //    Serial.println("Stop in int_cnt detected.");
  //    Serial1.print("3");  // this is added by mike to send A park command to Motor driver after flash stops
  //    neck_ok_flag = false;
  //    first_time_neck_command = YES;
  //  }
  //digitalWrite(ledPin, LOW);    // sets the LED off
}

//=======================
void sendSetting(void)
{
  Serial2.write(settingMessage, sizeof(settingMessage));
}

//=======================
// ========== check for Alert ================
void alarmCheck(void)
{
  char numberOfByte = 0;
  //messageRecieved[0]= 1;
  //messageRecieved[1]= 1;
  //messageRecieved[2]= 1;
  //messageRecieved[3]= 1;
  //messageRecieved[4]= 1;


  if (Serial2.available())
  {


    numberOfByte = Serial2.readBytes( (char *)messageRecieved, 32); // read the bytes and fill it in the array   //

    //Serial.println(numberOfByte);   //for debuggin
    //Serial.write( messageRecieved, 32);   // for debugging at the time that flash driver was connected to Serial0


    if (messageRecieved[0] == 0xaa && messageRecieved[1] == 0x55 && messageRecieved[2] == 0x03 && messageRecieved[3] == 0x07 && messageRecieved[4] == 0x01) // over current alarm communication
    {
      genie.WriteObject(GENIE_OBJ_FORM, 0x09, 0x09);   // Go to Form9
      stopFlash();
      first_time_neck_command = YES;
    }
    else if (messageRecieved[0] == 0xaa && messageRecieved[1] == 0x55 && messageRecieved[2] == 0x03 && messageRecieved[3] == 0x07 && messageRecieved[4] == 0x02) // overheat alarm parameters
    {
      genie.WriteObject(GENIE_OBJ_FORM, 0x08, 0x08);   // Go to Form8
      stopFlash();
      first_time_neck_command = YES;
    }
    else if (messageRecieved[0] == 0xaa && messageRecieved[1] == 0x55 && messageRecieved[2] == 0x03 && messageRecieved[3] == 0x07 && messageRecieved[4] == 0x04) //abnormal discharge
    {
      genie.WriteObject(GENIE_OBJ_FORM, 0x0a, 0x0a);   // Go to Form10
      stopFlash();
      first_time_neck_command = YES;
    }
    else {}
  }
  else {}
}


//============= reading 32 bytes recieved via Serialport =====

/* // ========== check for Alert ================
  void echoCheck(void)
  {
   messageRecieved[0]= 0;
   messageRecieved[1]= 0;
   messageRecieved[2]= 0;
   messageRecieved[3]= 0;
   messageRecieved[4]= 0;


     if (Serial1.available() >= 5)
       {

         for (int i=0; i<5; i++)
         {
           messageRecieved[i]=Serial.read();// read the bytes and fill it in the array
         }


         if(messageRecieved[0]== 0x55 && messageRecieved[1]== 0xaa && messageRecieved[2]==0x03 && messageRecieved[3]==0x07 && messageRecieved[3]==0x01) // over current alarm communication
          genie.WriteObject(GENIE_OBJ_FORM, 0x09,0x09);    // Go to Form9
         if else (messageRecieved[0]== 0x55 && messageRecieved[1]== 0xaa && messageRecieved[2]==0x03 && messageRecieved[3]==0x07 && messageRecieved[3]==0x02)  // overheat alarm parameters
          genie.WriteObject(GENIE_OBJ_FORM, 0x08,0x08);    // Go to Form8
         if else (messageRecieved[0]== 0x55 && messageRecieved[1]== 0xaa && messageRecieved[2]==0x03 && messageRecieved[3]==0x07 && messageRecieved[3]==0x04)  //abnormal discharge
          genie.WriteObject(GENIE_OBJ_FORM, 0x10,0x10);    // Go to Form10
         else{}
       }
   else{}
  }

*/

//======================
byte checkSumOfMessage(byte array[], int messageSize)
{
  array[messageSize];
  int i, sum = 0;
  byte *ptr;
  ptr = array; /* a=&a[0] */

  for (i = 0; i < messageSize - 1 ; i++)
  {
    sum = sum + *ptr;
    ptr++;
  }
  return (byte)sum;
}

//************************************
void checkClient()
{
  //  If an incoming client connects, there will be bytes available to read
  EthernetClient client = server.available();
  if (client)
  {
    //Serial.println("New client");
    // An http request ends with a blank line
    boolean currentLineIsBlank = true;
    stringReceived = "";
    while (client.connected())
    {
      if (client.available())
      {
        c = client.read();
        stringReceived += c;
        //Serial.write(c);
        // If you've gotten to the end of the line (received a newline character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank)
        {
          // Send a standard http response header
          parseCommand();
          answerClient(client);
          break;
        }
        if (c == '\n')
        {
          // You're starting a new line
          currentLineIsBlank = true;
        }
        else if (c != '\r')
        {
          // You've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    //Serial.println(stringReceived);            // for Ethernet issue debug(july 8_2015)
    // Give the web browser time to receive the data
    delay(1);
    // Close the connection:
    client.stop();
    //Serial.println("Client disconnected");
  }
}

void parseCommand()
{
  int beginCommandReceived = stringReceived.indexOf("command=");
  beginCommandReceived += 8;
  int endCommandReceived = stringReceived.indexOf(" ", beginCommandReceived);
  commandReceived = stringReceived.substring(beginCommandReceived, endCommandReceived);
}

void answerClient(EthernetClient client)
{
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: keep-alive");       // The connection will be closed after completion of the response
  client.println("Refresh: 5");                  // Refresh the page automatically every 5 sec
  client.println();
  //client.println("<!DOCTYPE HTML>");
  //client.println("<html>");
  char commandReceivedArray[commandReceived.length() + 1];
  commandReceived.toCharArray(commandReceivedArray, commandReceived.length() + 1);
  token = strtok(commandReceivedArray, "_");
  //Serial.println(token);
  if (token.equals("setParameters"))
  {
    token = strtok(NULL, "_");
    workModeSwitch = token.toInt();
    token = strtok(NULL, "_");
    workingFrequency = token.toInt();
    token = strtok(NULL, "_");
    runTime = token.toInt();
    token = strtok(NULL, "_");
    pauseTime = token.toInt();
    token = strtok(NULL, "_");
    workingVoltage = token.toInt();
    client.println("true");
  }
  else if (token.equals("armFlag"))
  {
    client.println("Hi");
    arm_flag = true;
  }
  else if (token.equals("roomNumber"))
  {
    token = strtok(NULL, "_");
    roomNumber = token.toInt();
    roomNumberConfirm_flag = true;
    client.println("true");
  }
  else if (token.equals("settingRequest"))
    client.println("true");
  else if (token.equals("settingRequest2"))
  {
    //workingFrequency = analogRead(A0);
    //workingVoltage = analogRead(A1);
    //runTime = analogRead(A2);
    //pauseTime = analogRead(A3);
    //workModeSwitch = analogRead(A4);
    workingFrequency = updated_workingFrequency;
    workingVoltage = updated_workingVoltage;
    runTime = updated_runTime;
    pauseTime = updated_pauseTime;
    workModeSwitch = updated_workModeSwitch;
    client.println(workingFrequency);
    client.println(" ");
    client.println(workingVoltage);
    client.println(" ");
    client.println(runTime);
    client.println(" ");
    client.println(pauseTime);
    client.println(" ");
    client.println(workModeSwitch);
  }
  else if (token.equals("startFlag"))
  {
    timeStartRunning = millis();
    getDateDs1307();
    startYear = String(year);
    startMonth = String(month);
    startDay = String(dayOfMonth);
    startHour = String(hour);
    startMinute = String(minute);
    startSecond = String(second);
    start_flag = true;
    stop_flag = false;
    stopType = 0;
    //run_flag = start_flag;
    run_pressed_flag = start_flag;    //App will start the process
    stop_pressed_flag = NO;
    genie.WriteObject(GENIE_OBJ_FORM, 0x06, 0x06);   // Go to Form6
  }
  else if (token.equals("currentRunTime"))
  {
    if (!(stop_flag && (stopType == 1 || stopType == 2)))
      client.println((millis() - timeStartRunning) / 1000);
    else
    {
      getDateDs1307();
      stopYear = String(year);
      stopMonth = String(month);
      stopDay = String(dayOfMonth);
      stopHour = String(hour);
      stopMinute = String(minute);
      stopSecond = String(second);
      if (stopType == 1)
        client.println("stop_flag_n");
      else if (stopType == 2)
      {
        client.println("stop_flag_i");
        //cycleInterrupt_flag = false;
      }
      stop_flag = false;
      stopType = 0;
    }
  }
  else if (token.equals("stopFlag"))
  {
    getDateDs1307();
    stopYear = String(year);
    stopMonth = String(month);
    stopDay = String(dayOfMonth);
    stopHour = String(hour);
    stopMinute = String(minute);
    stopSecond = String(second);
    client.println("stopConfirmFlag");
    stop_flag = true;
    stopType = 3;  //1;
    stop_pressed_flag = stop_flag;   //App will stop the process
    run_pressed_flag = NO;
    stop_flag = false;
  }
  else if (token.equals("reportRequest"))
  {
    client.println("true");
  }
  else if (token.equals("reportRequest2"))
  {
    client.println("reportRequest2");
    client.println(" ");
    client.println(roomNumber);
    client.println(" ");
    client.println(totalRunTime);    //(millis()-timeStartRunning)/1000
    client.println(" ");
    client.println(startYear);
    client.println(" ");
    client.println(startMonth);
    client.println(" ");
    client.println(startDay);
    client.println(" ");
    client.println(startHour);
    client.println(" ");
    client.println(startMinute);
    client.println(" ");
    client.println(startSecond);
    client.println(" ");
    client.println(stopYear);
    client.println(" ");
    client.println(stopMonth);
    client.println(" ");
    client.println(stopDay);
    client.println(" ");
    client.println(stopHour);
    client.println(" ");
    client.println(stopMinute);
    client.println(" ");
    client.println(stopSecond);
    client.println(" ");
    client.println(flashDuration);
  }
  else if (token.equals("reportReceivedFlag"))
  {
    reportReceived_flag = true;
    client.println("true");
  }
}

// Gets the date and time from the ds1307
void getDateDs1307() {
  // Reset the register pointer
  Wire.beginTransmission(clockAddress);
  Wire.write(byte(0x00));
  Wire.endTransmission();

  Wire.requestFrom(clockAddress, 7);

  // A few of these need masks because certain bits are control bits
  second     = bcdToDec(Wire.read() & 0x7f);
  minute     = bcdToDec(Wire.read());
  // Need to change this if 12 hour am/pm
  hour       = bcdToDec(Wire.read() & 0x3f);
  dayOfWeek  = bcdToDec(Wire.read());
  dayOfMonth = bcdToDec(Wire.read());
  month      = bcdToDec(Wire.read());
  year       = bcdToDec(Wire.read());
#if 1
    Serial.println();
    Serial.print(hour, DEC);
    Serial.print(":");
    Serial.print(minute, DEC);
    Serial.print(":");
    Serial.print(second, DEC);
    Serial.print("  ");
    Serial.print(month, DEC);
    Serial.print("/");
    Serial.print(dayOfMonth, DEC);
    Serial.print("/");
    Serial.print(year, DEC);
#endif
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return ( (val / 16 * 10) + (val % 16) );
}

String commandReceived2;
boolean flagDataReceived2 = false;
void checkNeckInput()
{
  if (Serial1.available())
  {
    //int val = Serial1.parseInt();
    int val  = Serial1.read();
    //Serial.println(val);
    // int val = Serial1.read();
    if (val == 2)
    {
      informed_flag = true;
      neck_ok_flag = true;
      // 20151229
      do
      {
        delay (3);
        Serial1.read();
        Serial1.flush();
        delay (3);
      } while (Serial1.available());
      // 20151229
      //Serial.print("i'm reading1   ");
      //Serial.println(val);

    }
    //------------------- i did not use that ----------------
    /*else if (val == 3)
      {
       neck_ok_flag = true;
       //Serial1.write(val+1);

      Serial.print("neck_ok_flag   ");
      Serial.println(neck_ok_flag);
      } */
    //--------------------------------------------------
    else
    {
      //neck_ok_flag = false;
      informed_flag = false;
      // Serial.print("neck_ok_flag   ");
      // Serial.println(neck_ok_flag);
      //Serial.print("i'm reading2   ");
      //Serial.println(val);
    }
  }
  //---------------------
  /*  if(Serial1.available())
    {
      commandReceived2 = "";
      while(Serial.available())
      {
        commandReceived2 += char(Serial.read());
        if(!flagDataReceived2)
          flagDataReceived2 = true;
        delay(1);
      }
      if(commandReceived2.equalsIgnoreCase("S"))
      {
        stop_flag = true;
        stopType = 2;
      }
      flagDataReceived2 = false;
    }*/
  //------------------------
}
//======================== Radio function ======================
void getWireless(void)
{
  if (nrf24.available())
  {
    String doorResult;
    // Should be a message for us now
    uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (nrf24.recv(buf, &len))
    {
      //      NRF24::printBuffer("request: ", buf, len);
      // Serial.print("got request: ");
      // Serial.println((char*)buf);
      doorResult = (char*)buf;
      //if (doorResult == "close")
      if (doorResult == "C")
      {
        //door_ok_flag = true;
        Serial.println("+");
        stop_door_sensor_flag = NO;
        flagDoorSensorIsAlive = true;
        //Serial.println("close");
        //doorResult = "open";

      }
      else if ((doorResult == "O") || (doorResult == "S"))
      {
        //door_ok_flag = false;
        Serial.println("-");
        unsigned long compareTime = ((millis()) - firstDoorOpenDetectTime);
        if (compareTime >= DOOR_SENSOR_DETECT_TIME)
        {
        stop_door_sensor_flag = YES;
        }
        //Serial.println("moved");
      }
      // Send a reply
      uint8_t data[] = "And hello back to you";
      nrf24.send(data, sizeof(data));
      nrf24.waitPacketSent();
      //Serial.println("Sent a reply");
    }
    else
    {
      stop_door_sensor_flag = YES;
      //Serial.println("recv failed");
    }
  }
  //  else
  //  {
  //    stop_door_sensor_flag = YES;
  //  }

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//===== Keyboard related functions ======================

void updateDisplay(void)
{
  char *p ;
  char buf [32] ;

  if (errorCondition)
    sprintf (buf, "%s", "ERROR") ;
  else
    //dtostrf (display, 11, 5, buf) ;
    dtostrf (display, 4, 0, buf) ;

  genie.WriteStr (1, buf) ;	// Text box number 0
  genie.WriteStr (2, buf) ;	// Text box number 0
  genie.WriteStr (3, buf) ;	// Text box number 0
  genie.WriteStr (4, buf) ;	// Text box number 0

}


/*
   processOperator:
 	Take our operator and apply it to the accumulator and display registers
 *********************************************************************************


  /*
   calculatorKey:
 	We've pressed a key on our calculator.
 *********************************************************************************
*/

void calculatorKey(int key)
{
  boolean gotDecimal = false ;
  //static int startNewNumber = TRUE ;
  static double multiplier  = 1.0 ;
  float digit ;

  // Eeeeee...

  if (errorCondition && key != 'a')
    return ;

  if (isdigit(key))
  {
    if (start_new_number_flag)
    {
      start_new_number_flag = false ;
      multiplier     = 1.0 ;
      display        = 0 ;
    }

    digit = (double)(key - '0') ;
    if (multiplier == 1.0)
      display = (unsigned int)(display * 10 + (double)digit) ;
    else
    {
      display     = (unsigned int)(display + (multiplier * digit)) ;
      multiplier /= 10.0 ;
    }
    updateDisplay () ;
    return ;
  }

  switch (key)
  {
    case 'a':			// AC - All Clear
      lastOperator   = 0 ;
      acc            = 0.0 ;
      memory         = 0.0 ;
      display        = 0 ;
      gotDecimal     = false ;
      errorCondition = false ;
      start_new_number_flag = true ;
      break ;

    /*case 'c':			// Clear entry or operator
      if (lastOperator != 0)
        lastOperator = 0 ;
      else
      {
        display        = 0.0 ;
        gotDecimal     = false ;
        start_new_number_flag = true ;
      }
      break ;

      // Memory keys

      case 128:		// Mem Store
      memory = display ;
      break ;

      case 129:		// M+
      memory += display ;
      break ;*/

    case 130:		// Enter
      //start_new_number_flag = true ;
      enter_pressed_flag = true;
      //display = 0 ;
      break ;

    case 131:		// Clear
      clear_pressed_flag = true;
      display = 0 ;
      start_new_number_flag = true ;
      break ;

    /*case 132:		// MC - Memory Clear
      memory = 0.0 ;
      break ;

      // Other functions

      case 140:		// +/-
      display = -display ;
      break ;

      case 's':	// Square root
      if (display < 0.0)
        errorCondition = true ;
      else
      {
        display        = sqrt (display) ;
        gotDecimal     = false ;
        start_new_number_flag = true ;
      }
      break ;

      // Operators


      case '=':
      if (lastOperator != 0)
        processOperator (lastOperator) ;
      lastOperator    = 0 ;
      gotDecimal     = false ;
      start_new_number_flag = true ;
      acc            = 0.0 ;
      break ;

      case '.':
      if (!gotDecimal)
      {
        if (start_new_number_flag)
        {
          start_new_number_flag = false ;
          display        = 0.0 ;
        }
        multiplier = 0.1 ;
        gotDecimal = true ;
        break ;
      }*/

    default:
      printf ("*** Unknown key from display: 0x%02X\n", key) ;
      break ;
  }

  updateDisplay() ;
}

