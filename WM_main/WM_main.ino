/*******************************************************************************
@Filename       WM_main.ino
--------------------------------------------------------------------------------
@Description    This is the main file. This is where the functions are called inside
        setup() function or in the loop() function

--------------------------------------------------------------------------------
@Author        Dragos B., Marian S., Stefan I.
@Date          23.03.2020

@Copyright     Miele  Cie Copyright 2020

*******************************************************************************/



/*******************************************************************************
@Project Includes
*******************************************************************************/
#include "ControlPanel.h"
#include "LED.h"
#include "TwinDos.h"
#include "LCDDisplay.h"
#include "MotorDriver.h"
#include "GenericDisplay.h"
/*******************************************************************************
@Constants (global)
*******************************************************************************/
#define ERROR_SERIAL(err){Serial.print(_LINE_);Serial.print("ERROR is: ");Serial.println((int)err);}


uint16_t Initialise_u16(uint8_t MotorPinClockwise_u8, uint8_t MotorPinCounterClockwise_u8);
    // This should be in the setup
// Main TODO
// 2. TODO: Move all the Miele Libraries and CPP files into a separate folder than Adafruit, LiquidCrista_I2c and etc.
// details about the board ESP32 can be added
// 5. Create a fully functional washing machine using all the components
LCDDisplay disp_o(21, 22, 0x27);
#ifndef _GENERICDISPLAY_h
#define _GENERICDISPLAY_h
  
#include "LiquidCrystal_I2C.h"



// Since most of the times a display will be LCD 16x2, then this will be the standard definitions.

// Initial starting point of the LCD cursor (0,0) 
#define CURSOR_INITIAL_INDEX 0
#define NUMBER_OF_COLUMNS    16
#define NUMBER_OF_ROWS       2

class GenericDisplay
{
  public:
    virtual bool init_b();
    virtual bool DisplayString_b(char* StringToDisplay_pc);
    virtual bool ClearScreen_b();
        virtual bool SendCharacter_b(char CharacterToDisplay);
    //todo not implemented
    virtual bool AutoScroll_b();
    virtual bool ChangeFontSize_b();
    
    
  // Since most of the times a display will be LCD 16x2, then this will be the standard settings.
  protected:
  LiquidCrystal_I2C * lcd_po;
  // Writing coordinates of the LCD 16x2 cursor.
    uint8_t WritingCursorLine_u8;
    uint8_t WritingCursorColumn_u8;
  
  // These are the pins that we use for the I2C protocol.
  // SdaPinNumber_u8 - is used to control the Serial Data Line
  // SclPinNumber_u8 - is used to control the Serial Clock Line 
  // DeviceAdress_u8 - holds the display device address
    uint8_t SdaPinNumber_u8;
    uint8_t SclPinNumber_u8;
    uint8_t DeviceAdress_u8;

};
#endif
MotorDriver motor_ob;
void setup()
{
    // put your setup code here, to run once:
    
    Serial.begin(9600);                 // serial Init for test messages
    delay(1500);                        // give time to Wemos Lolin32 to finish setup
    disp_o.init_b();
    ERROR_SERIAL(motor_ob.Initialise_u16(19, 25));
}

void loop()
{ 
    disp_o.DisplayString_b("Hello world1");
    delay(1000);
    disp_o.ClearScreen_b();
    disp_o.DisplayString_b("Hello world2");   


    ERROR_SERIAL(motor_ob.MoveMotor_u16(80, MOTOR_ROTATION_CLOCKWISE, 2));
    ERROR_SERIAL(motor_ob.StopMotor_u16(2));
    ERROR_SERIAL(motor_ob.MoveMotor_u16(80, MOTOR_ROTATION_COUNTER_CLOCKWISE, 2));
    ERROR_SERIAL(motor_ob.StopMotor_u16(2));
}
