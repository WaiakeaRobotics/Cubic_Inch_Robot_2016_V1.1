// ================================================================
// ===         2016 Waiakea Robotics Cubic Inch Robot           ===
// ================================================================
/*

Final goal is to navigate a fixed grid maze using the gyroscope sensor and front IR sensors to detect gaps in the "maze"
The "maze" is not a real maze but actually a grid of blocks arranged in a 3 x 3 pattern with each block being 150mm square and a 30mm gap between them.
The goal is to drive over the 7 control points. Further details are here: http://imd.eng.kagawa-u.ac.jp/maze/reg_c2.html
The difficulty is doing this with a robot fitting in 1 cubic inch and also doing it faster than all of your opponents as the fastest robot wins. 

There is also a human driver controlled category in the competition, so this program also will have a user operated mode.
Autonomous mode will most likely be activated by pushing a certain button on the remote control. 

The remote control has the same 2.4ghz transceiver connected to the SPI port so they can communicate both ways.  
The remote contains 8 pushbuttons, arranged in two "D" pads one on the left and one on the right.
It also contains a 128x64 pixel OLED display wired to the I2C port. 

The robot has a 2:1 voltage divider to allow battery voltage monitoring in real time. 
6
*/

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  #define Serial SERIAL_PORT_USBVIRTUAL  // Required for Serial on Zero based boards
#endif

#include <Arduino.h>
#include <wiring_private.h>
//#include "libs/Analog_Write_SAMD/Analog_Write_SAMD.h"
//#include "libs/Analog_Write_SAMD/Analog_Write_SAMD.cpp"

// ================================================================
// ===               2.4Ghz Transceiver Includes                ===
// ================================================================

#include <SPI.h>  // Library for SPI communications used by the nRF24L01 radio
#include <Wire.h>
#include "libs/RadioHead/RH_NRF24.h"        // Max we can send is 28 bytes of data 
#include "libs/RadioHead/RH_NRF24.cpp"
#include "libs/RadioHead/RHGenericSPI.h"
#include "libs/RadioHead/RHGenericSPI.cpp"
#include "libs/RadioHead/RHHardwareSPI.h"
#include "libs/RadioHead/RHHardwareSPI.cpp"
#include "libs/RadioHead/RadioHead.h"
#include "libs/RadioHead/RHGenericDriver.h"
#include "libs/RadioHead/RHGenericDriver.cpp"
#include "libs/RadioHead/RHNRFSPIDriver.h"
#include "libs/RadioHead/RHNRFSPIDriver.cpp"

RH_NRF24 nrf24(2, 38); //CE, CSN

// ================================================================
// ===                  OLED Display Includes                   ===
// ================================================================

#include "libs/Adafruit_GFX/glcdfont.c"
#include "libs/Adafruit_GFX/Adafruit_GFX.h"
#include "libs/Adafruit_GFX/Adafruit_GFX.cpp"
#include "libs/Adafruit_SSD1306/Adafruit_SSD1306.h"
#include "libs/Adafruit_SSD1306/Adafruit_SSD1306.cpp"
#include "libs/Adafruit_GFX/Fonts/TomThumb.h"

#define OLED_RESET A5
Adafruit_SSD1306 display(OLED_RESET);

// ================================================================
// ===                    Robot Pin Defines                     ===
// ================================================================

// Lets define some nice handy constants eh?

#define LED_L_R 9  // Left side Red LED
#define LED_L_G 25 // Left side Green LED - only green seems to work with analog write
#define LED_L_B 8  // Left side Blue LED

#define LED_R_R 13 // Right side Red LED - Also standard arduino LED pin
#define LED_R_G 7  // Right side Green LED - No PWM on this pin - all others have PWM
#define LED_R_B 26 // Right side Blue LED

#define LED_ON 0 // LEDs are active LOW
#define LED_OFF 1 // LEDs are active LOW

#define IR_38Khz 5    // IR LED 38khz Mosfet Driver Pin

#define IR_SENSOR_R A1 // Right input from 38khz bandpass filter connected to IR PIN diode
#define IR_SENSOR_L A2 // 

#define WALL_DETECTED 0 // Wall detected
#define WALL_NOT_DETECTED 1 // Wall not detected

#define MOTOR_R_DIR 11 // Right motor direction pin
#define MOTOR_R_SPD 10 // Right motor speed pin - apply PWM signal (analog out) to this pin
#define MOTOR_L_DIR 12
#define MOTOR_L_SPD 6

#define MOT_R_SPD_REG REG_TC3_COUNT8_CC0
#define MOT_L_SPD_REG REG_TCC0_CCB2

#define BATT_VOLTAGE A4 //Battery voltage monitor pin - connected to 50% divider to allow the measurment of voltages higher than the vcc of 3.3v

#define FWD 0 // 0 = forward in our robot wiring
#define BWD 1 // 1 = backward in our robot wiring

// The below defines are for the bit location of the corresponding buttons in our 8 bit encoded buttons variable received from the transmitter
#define REMOTE_A 0  // Right D pad up button
#define REMOTE_B 1  // Right D pad right button
#define REMOTE_C 2  // Right D pad down button
#define REMOTE_D 3  // Right D pad left button

#define REMOTE_FWD 4    // Left D pad up button
#define REMOTE_RIGHT 5 // Left D pad right button
#define REMOTE_BWD 6  // Left D pad down button
#define REMOTE_LEFT 7  // Left D pad left button

// ================================================================
// ===                  Variable Definitions                    ===
// ================================================================

uint8_t sendBuff[7];  // 28 element array of unsigned 8-bit type - 28 is the max message length for the nrf24L01 radio
uint8_t recvBuff[RH_NRF24_MAX_MESSAGE_LEN]; // Receive Buffer
uint8_t lenRecv = sizeof(recvBuff);
bool recvFlag  = false;

int iteration=0;

bool blinkState = false;
bool blinkState1 = false;
bool blinkState2 = false;

bool startAButton;
bool startBButton;
bool startCButton;
bool startDButton;
 
int battVoltage;

int outputInt;

int forwardRamp;
int forwardRampD;
int loopTimer;

int slowTimer;
int autoState;

unsigned char loopCounter;

unsigned long lastMillis, timeAway;
unsigned long lastMillisGyro, timeAwayGyro;

unsigned long autoMillis;

unsigned char sendCounter;

unsigned long loopTime;
unsigned long lastMillisLoop;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  
  Serial.begin(115200); // 
//    while (!Serial);      // WARNING - sketch will not start until USB serial port opened on computer 
//                             only uncomment if you need to read the first serial prints 
  Serial.println("Hello :)"); // Testing initial serial print

    
// ================================================================
// ===                     Robot Pin Setup                      ===
// ================================================================

  pinMode(LED_L_R, OUTPUT);
  pinMode(LED_L_G, OUTPUT);
  pinMode(LED_L_B, OUTPUT);

  pinMode(LED_R_R, OUTPUT);
  pinMode(LED_R_G, OUTPUT);
  pinMode(LED_R_B, OUTPUT);

  digitalWrite(LED_L_R, 1);  
  digitalWrite(LED_L_G, 1); 
  digitalWrite(LED_L_B, 1); 

  digitalWrite(LED_R_R, 1); 
  digitalWrite(LED_R_G, 1); 
  digitalWrite(LED_R_B, 1);  

  digitalWrite(MOTOR_R_DIR, FWD);
  //analogWrite(MOTOR_R_SPD, 0);
  digitalWrite(MOTOR_L_DIR, FWD);
  //analogWrite(MOTOR_L_SPD, 0);
    
// ================================================================
// ===               2.4Ghz Transceiver Setup                   ===
// ================================================================  

  //nrf24.init(); // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  
  if (!nrf24.init()) Serial.println("Radio init failed"); else Serial.println("Radio init success"); // Debug radio init
  
  nrf24.setChannel(2); // Set the desired Transceiver channel valid values are 0-127, in the US only channels 0-83 are within legal bands
  nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm);   
         
// ================================================================
// ===                       38Khz SETUP                        ===
// ================================================================ 

  //pinMode(IR_38Khz, OUTPUT);
  //tone(IR_38Khz, 38000); // Set IR LED pin to 38khz 50% duty cycle square wave
  analogWrite(IR_38Khz, 127); 
  analogWrite(MOTOR_R_SPD, 0); // Make sure both motors are off
  analogWrite(MOTOR_L_SPD, 0);

// Configure TC3 for IR_LED and MR_PWM
// Set up the generic clock (GCLK4) used to clock timers
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(5) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

// Feed GCLK4 to TCC2 (and TC3)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC2 (and TC3)
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed GCLK4 to TCC2 (and TC3)
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

//////////////////////////////////////////////////////////////////////////////////////////////////// END TC3 setup for MR_PWM and IR_LED
// Configure TCC0 setup for ML_PWM

  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(5) |          // Divide the 48MHz clock source by 1
                    GCLK_GENDIV_ID(5);            // Select Generic Clock (GCLK) 5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(5);          // Select GCLK5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for the 4 PWM channels: timer TCC0 outputs
  const uint8_t CHANNELS = 1;
  const uint8_t pwmPins[] = {6,}; // 5 for IR 38khz, 6 for ML_PWM, 10 for MR_PWM
  for (uint8_t i = 0; i < CHANNELS; i++)
  {
     PORT->Group[g_APinDescription[pwmPins[i]].ulPort].PINCFG[g_APinDescription[pwmPins[i]].ulPin].bit.PMUXEN = 1;
  }

  //The PMUX registers are arranged in pin pairs odd and even. So for example Arduino's digital pin 13 (D13) 
  //is actually the SAMD21 port A, pin 17, or PA17. An odd pin. (See Arduino Zero's schematic diagram). 
  //This is paired with its neighboring even pin PA16, (actually digital pin 11). 
  //To connect the TCC2 timer to the D13 we have to specify the even SAMD21 pin, in this case PA16 (D11), 
  //then connect the timer to D13 using the odd PORT_PMUX_PMUXO_E mask. PMUXO stands for: port multiplexer odd. 
  //The E and F suffixes specify that the timers are to be connected. 
  //Connect the TCC0 timer to the port outputs - port pins are paired odd PMUO and even PMUXE
  //F & E specify the timers: TCC0, TCC1 and TCC2
  //The g_APinDesciription() function simply converts the Arduino pin numbers into the SAMD21's port and pin representation.
 // PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg = PORT_PMUX_PMUXO_F; // for arduino pin D2,D5 or PA14,PA15 or pin23,pin24 -- only use odd PMUXO for PA15
  PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg = PORT_PMUX_PMUXE_F; // for arduino pin D6, or PA20, or pin 29 use only even PMUXE for PA20
 // PORT->Group[g_APinDescription[10].ulPort].PMUX[g_APinDescription[10].ulPin >> 1].reg = PORT_PMUX_PMUXE_F; // for arduino pin D10, or PA18, or pin 27 use only even PMUXE for PA20

  // Feed GCLK5 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_CLKCTRL_CLKEN |         // Enable GCLK5 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK5 |     // Select GCLK5
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK5 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
 // REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC0 outputs
 //                   TCC_WAVE_WAVEGEN_DSBOTTOM;    // Setup dual slope PWM on TCC0
 // while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        // Setup single slope PWM on TCC0 - Using single slope as 8 bit is plenty for motors and TC3 for other motor is only 8 bit. 
  while (TCC0->SYNCBUSY.bit.WAVE);                // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  // 400 = 20kHz
  REG_TCC0_PER = 255;      // Set the frequency of the PWM on TCC0 to 50Hz
  while(TCC0->SYNCBUSY.bit.PER);

  // The CCBx register value corresponds to the pulsewidth in microseconds (us)
  REG_TCC0_CCB2 = 127;       // TCC0 CCB2 - 50% duty cycle on D6 - PIN 29 AND D10
  while(TCC0->SYNCBUSY.bit.CCB2);

  // Divide the 16MHz signal by 1 giving 16MHz (62.5ns) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization



// ================================================================
// ===                    OLED Display SETUP                    ===
// ================================================================ 

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C
  display.display();
  display.setFont(&TomThumb);
  display.setRotation(1);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,7); // TomThumb font is referenced from the bottom of the font, which is 7 pixels high
  display.println("0 Hello"); // usable screen area is 7 characters wide
  display.println("1 World"); // by 10 rows tall
  display.println("2");
  display.println("3");
  display.println("4");
  display.println("5");
  display.println("6");
  display.println("7");
  display.println("8");
  display.println("9");
  display.display();

  
// ================================================================
// ===           FINAL ACTIONS BEFORE EXITING SETUP             ===
// ================================================================  
//  analogWrite(MOTOR_R_SPD, 25); // Make sure both motors are off
//  analogWrite(MOTOR_L_SPD, 225);
      
}// end setup loop

// =========================================================================================================================
// ===                                                 MAIN PROGRAM LOOP                                                  ==
// =========================================================================================================================

void loop()
{
  loopCounter ++; // 8 bit unsigned counter variable will overflow back to 0 after 255
  if (loopCounter > 127) digitalWrite(LED_L_B, LED_OFF); // Blink Left Blue LED
  else digitalWrite(LED_L_B, LED_ON); // Blinking shows the main program loop is working at least :D

  // The below referenced functions are all below this main loop
  updateRadio();   // Receive from and Send data to the remote
  debugSensors();  // IR sensors turn on/off the motor outputs and green LEDs
  updateDisplay(); // Update the display with new data
  reactRemote();   // Do something with remote inputs - usually for manual RC mode driving
  //autoz();         // Update custom auto maze running state machine

} // end main loop

void updateDisplay()
{
  display.clearDisplay(); // Clear display at start of this cycle
  display.setCursor(0,7); // Set printing to start at top left of screen again - 7 pixels down for TomThumb font height offset
  
  display.print("B ");
  display.println(analogRead(BATT_VOLTAGE)); // print battery voltage
  display.println(millis());
  display.print("Loop ");
  display.println(loopTime);
  display.print("R0   ");
  display.println(recvBuff[0]);
  display.print("R1   ");
  display.println(recvBuff[1]);

  if (recvFlag) // was data received?
  {  
    recvFlag = false;
    display.println("Receive");
  }
  else
  {
    display.println("!");
  }
  
  loopTime = millis() - lastMillisLoop;
  //Serial.print("main loop time: ");
  //Serial.println(loopTime);
  
  lastMillisLoop = millis();
  
  display.display(); // write data to display - before this is called display is not changed
}

void debugSensors()
{
  if (digitalRead(IR_SENSOR_L) == WALL_DETECTED) // this if statement does the same thing as the above lines but is written with an IF statement
  {
    MOT_L_SPD_REG = 127;       // Left motor PWM register
    digitalWrite(LED_L_G, LED_ON); // Turn ON the Green LED
  } else 
  {
    MOT_L_SPD_REG = 0;        // Left motor PWM register
    digitalWrite(LED_L_G, LED_OFF); // Turn OFF the Green LED
  }
   
  if (digitalRead(IR_SENSOR_R) == WALL_DETECTED) // this if statement does the same thing as the above lines but is written with an IF statement
  {
    MOT_R_SPD_REG = 127; // Right motor PWM register                    
    digitalWrite(LED_R_G, LED_ON); // Turn ON the Green LED
  } else 
  {
    MOT_R_SPD_REG = 0; // Right motor PWM register                     
    digitalWrite(LED_R_G, LED_OFF); // Turn OFF the Green LED
  }
}

void reactRemote()
{
  
  if (bitRead(recvBuff[0], REMOTE_A) == HIGH){ //if button A pushed // drive at start degrees

    digitalWrite(LED_L_R, LED_ON);
    if(startAButton == true) // debouncing
    {
      startAButton = false;
      // Do something on first press
    }
  }
  else
  {
    digitalWrite(LED_L_R, LED_OFF);
    startAButton = true; // once released reset start variable so button can be pushed again
  }
  
  if (bitRead(recvBuff[0], REMOTE_B) == HIGH){ // do something when button B is pushed
    if(startBButton == true)
    {
      startBButton = false;
      // do something here once
    }
    // or do something here while button pushed
  }
  else
  {
    startBButton = true;
    digitalWrite(LED_R_G, LED_OFF); // turn off led
  }
  
  if (bitRead(recvBuff[0], REMOTE_C) == HIGH){ // when button C is pushed
    if(startCButton == true)
    {
      startCButton = false;
      // do something once
    }
    //do something while button pushed down
  }
  else
  {
    startCButton = true;
  }
  
  if (bitRead(recvBuff[0], REMOTE_D) == HIGH){ // 
    if(startDButton == true)
    {
      startDButton = false;
    }
  }
  else
  {
    startDButton = true;
  }

  if (bitRead(recvBuff[0], REMOTE_FWD) == HIGH){ // Forward
  
    if (forwardRamp > 244){ // keep ramp value from overflowing back to 0
      forwardRamp = 244;
    } 
    else forwardRamp = forwardRamp + 10; // increment ramp value by 1 

   // motors(FWD,FWD,forwardRamp,forwardRamp);// leftDirection, RightDirection, leftSpeed, rightSpeed
  }
  else{
    forwardRamp = 30;
    loopTimer = 0;
  }  
  if (bitRead(recvBuff[0], REMOTE_BWD) == HIGH){ // Backwards
    //motors(BWD,BWD,100,100);// leftDirection, RightDirection, leftSpeed, rightSpeed
  }
  if (bitRead(recvBuff[0], REMOTE_LEFT) == HIGH){ // Left
    //motors(BWD,FWD,70,70);// leftDirection, RightDirection, leftSpeed, rightSpeed
  }
  else if (bitRead(recvBuff[0], REMOTE_RIGHT) == HIGH){ // Right
    //motors(FWD,BWD,70,70);// leftDirection, RightDirection, leftSpeed, rightSpeed
  } 
  if (recvBuff[0] == 0) // No buttons pushed
  {
    //motors(FWD,FWD,0,0); //turn off motors
  }
 
}

// ================================================================
// ===              RECEIVE AND SEND DATA TO REMOTE             ===
// ================================================================
void updateRadio()
{
  if (nrf24.available()) // Is there received data from the remote control?
  {
    if (nrf24.recv(recvBuff, &lenRecv)) // receive the available data into the "receivebuffer" variable
    {
      sendCounter++;
      nrf24.send(sendBuff, sizeof(sendBuff)); // send the data inside the "sendBuffer" variable
      nrf24.waitPacketSent(); // Return true if data sent, false if MAX_RT
      recvFlag = true;  

      sendBuff[0] = sendCounter;
      sendBuff[1] = loopTime; //map(battVoltage,0,1023,0,255);
      sendBuff[2] = 123; //timeAway;
      sendBuff[3] = 12;
      sendBuff[4] = 12; // autoState; // Send some new data to the remote here for debugging
      sendBuff[5] = 12;
      sendBuff[6] = 12; // Send some new data to the remote here for debugging
    }
  }
}

void autoz()
{
  switch (autoState)
  {
    case 0:
      motors(FWD,FWD,130,100); // drive straight for a few ms
      autoMillis = millis();
      autoState++;
      break; 
    case 1:
      if (millis() > autoMillis + 200) autoState++;
      break; 
    case 2: // check for right gap
      motors(FWD,FWD,140,60); // turn slightly into right wall
      if (digitalRead(IR_SENSOR_R) != WALL_DETECTED) autoState++;
      break;
    case 3: // start right turn into first lane
      
      motors(FWD,BWD,85,0);// leftDirection, RightDirection, leftSpeed, rightSpeed
      autoState++;
      break;
    case 4: // check if turn completed
      
      break; 
    case 5: // start driving down lane
      motors(FWD,FWD,155,155); // start speed value
      autoMillis = millis();
      autoState++;
      break;
    case 6: // wait a few ms to check sensor
      motors(FWD,FWD,255,230); // full speed
      if (millis() > autoMillis + 150) autoState++;
      break; 
    case 7: //wait for first gap
      if ((digitalRead(IR_SENSOR_R) != WALL_DETECTED)&&(digitalRead(IR_SENSOR_L) != WALL_DETECTED)) autoState++;
      break;
    case 8: //wait while in gap
      if ((digitalRead(IR_SENSOR_R) == WALL_DETECTED)&&(digitalRead(IR_SENSOR_L) == WALL_DETECTED)) autoState++;
      autoMillis = millis();
      break;
    case 9: // wait a few ms
      if (millis() > autoMillis + 20) autoState++;
      break;
    case 10: //wait for second gap
      if ((digitalRead(IR_SENSOR_R) != WALL_DETECTED)&&(digitalRead(IR_SENSOR_L) != WALL_DETECTED)) autoState++;
      break;
    case 11: //wait while in second gap
      if ((digitalRead(IR_SENSOR_R) == WALL_DETECTED)&&(digitalRead(IR_SENSOR_L) == WALL_DETECTED)) autoState++;
      digitalWrite(LED_R_G, LED_ON); // Turn ON the Green LED
      autoMillis = millis();
      break;   
    case 12: // wait a few ms
      if (millis() > autoMillis + 100) autoState++;
      break;
    case 13: // check for last gap
      motors(FWD,FWD,80,80); // slow down for end of channel
      if ((digitalRead(IR_SENSOR_R) != WALL_DETECTED)&&(digitalRead(IR_SENSOR_L) != WALL_DETECTED)) autoState++;
      break;
    case 14: // start left turn
      
      motors(FWD,FWD,0,85); // start left turn
      if (0 < 35) autoState++;
      break;
    case 15: // drive straight until start of channel
      
      //if (digitalRead(IR_SENSOR_L) != WALL_DETECTED) autoState++;
      break;
           
        
/*
               targetYaw = yaw - 90;
        if (targetYaw < -180) targetYaw = targetYaw + 360;
        
        motors(BWD,FWD,60,60);// leftDirection, RightDirection, leftSpeed, rightSpeed
  
        while (yawDiff(targetYaw) > 40){ // if we are closer than 10 degrees - (because of overshoot)
          getGyro();
        }
        motors(FWD,FWD,0,0); //turn off motors
        */
  }
}
// -------------------------------------------------------------------
//        MOTOR CONTROLLER
// -------------------------------------------------------------------
 
void Move(int motor, int direction, int speed) 
{            
  if (motor == 0){ // Left Motor
    digitalWrite(MOTOR_L_DIR, direction);
    // Send PWM data to motor A
    REG_TCC0_CCB2 = speed;       // Left motor PWM register
    //analogWrite(MOTOR_L_SPD, speed);
  } else if (motor == 1){  // Right Motor
    digitalWrite(MOTOR_R_DIR, direction);
    // Send PWM data to motor A
    REG_TC3_COUNT8_CC0 = speed; // Right motor PWM register  
    //analogWrite(MOTOR_R_SPD, speed);
  }
}
void motors(char leftDirection,char rightDirection, char leftSpeed, char rightSpeed)
{
  digitalWrite(MOTOR_L_DIR, leftDirection);
  digitalWrite(MOTOR_R_DIR, rightDirection);
  REG_TCC0_CCB2 = leftSpeed;       // Left motor PWM register
  REG_TC3_COUNT8_CC0 = rightSpeed; // Right motor PWM register     
  //analogWrite(MOTOR_L_SPD, leftSpeed);
  //analogWrite(MOTOR_R_SPD, rightSpeed);
}
void autozBlocking()
{
  while(autoState < 15)
  {
    // autoz();
  }
}


