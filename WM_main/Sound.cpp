/*******************************************************************************
@Module         MD used for WM module.
--------------------------------------------------------------------------------
@Filename       Sound.cpp
--------------------------------------------------------------------------------
@Description    Check the header and class description for more details.

--------------------------------------------------------------------------------
@Author        Dragos B., Fabian V.
@Date          13.11.2018

@Copyright     Miele  Cie Copyright 2020

*******************************************************************************/

/*******************************************************************************
@Project Includes
*******************************************************************************/
#include "Sound.h"
#include "PWM_Timer.h"
/*******************************************************************************
@Constants (global)
*******************************************************************************/
#define SECOND_TO_MILLISECONDS      (1000)
//#define PWM_CHANNEL                  (1)
#define BUZZER_PIN                   (12)


#define PWM_FREQUENCY               (2000)
#define RESOLUTION_BITS             (8)

// This delay is used in order to call one line for
// a certain number of milliseconds.
// It should be used like this: DELAY_NON_BREAKING(WaitMs_u32) CodeToBeCalled();
#define DELAY_NON_BREAKING(WaitMs_u32) for (unsigned long time_now = millis(); millis() < time_now + WaitMs_u32;)

// This part is the replacement of delay from Arduino
#define DELAY_DO_NOTHING(WaitMs_u32) DELAY_NON_BREAKING(WaitMs_u32);
/*******************************************************************************
@Macros (global)
*******************************************************************************/

/*******************************************************************************
@Type definitions  (global)
*******************************************************************************/

/*******************************************************************************
@Local Variables
*******************************************************************************/
static  uint8_t channel_u8;
/*******************************************************************************
@External Prototypes
*******************************************************************************/

/*******************************************************************************
@Prototypes local Functions
*******************************************************************************/


/*******************************************************************************
Function description and additional notes,
are given at the function prototype in the header file
*******************************************************************************/

void InitialiseSound_v()
{
    channel_u8 = GetAvailableChannel_u8();
    if (channel_u8 != 17)
    {
        ledcSetup(channel_u8, PWM_FREQUENCY, 8);
        ledcAttach(BUZZER_PIN, channel_u8);
    }
    else
    {
        Serial.println("PWM channel is occupied");
    }
}

/*******************************************************************************
Function description and additional notes,
are given at the function prototype in the header file
*******************************************************************************/
void PlaySound_v(uint16_t TimeInSeconds_u16, Sounds_te sounds)
{
	GenerateSounds(sounds, TimeInSeconds_u16);
}
/*******************************************************************************
Function description and additional notes,
are given at the function prototype in the header file
*******************************************************************************/
int GenerateSounds(int sound_freq, int time) 
{
    
    ledcWriteTone(channel_u8, sound_freq);  

	return sound_freq;
}
/*******************************************************************************
Function description and additional notes,
are given at the function prototype in the header file
*******************************************************************************/
void StopSound_v()
{
    DELAY_NON_BREAKING(500) ledcWriteTone(channel_u8, 0);
}
