/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB® Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC16F1704
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v2.35 or v3.00
 */

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

#include "mcc_generated_files/mcc.h"

#define BOOT_DELAY 60
// potentiometer range deciCelsius
#define POT_TEMP_FROM   350
#define POT_TEMP_TO     500

// temperature limit (deciCelsius)
#define TEMP_FULLSPEED_ON    530
#define TEMP_FULLSPEED_OFF   500

#define TEMP_RANGE_OK   50      // for led indicator, deciCelsius

#define POWER_ON_DELAY  5
// PID regulator
// error input is in deciCelsius, max output is 155
// decimal numbers are allowed
#define Kp 6   
#define Ki 0
#define Kd 0

// max absolute influence of PIDi on output
#define PIDi_CLAMP 160

#define PIDd_DIVIDER 10

#define MAX_OUTPUT_CURRENT_mA   20
#define MIN_OUTPUT_CURRENT_mA    0
#define R   150
#define Vcc   5
#define OUT_MIN (256*MIN_OUTPUT_CURRENT_mA*R/1000/Vcc)
#define OUT_MAX (256*MAX_OUTPUT_CURRENT_mA*R/1000/Vcc)


#define LED_OFF         0
#define LED_FLASH_SLOW  4
#define LED_FLASH_FAST  1
#define LED_FLASH_SHORT 0x07
#define LED_ON          0xff

uint8_t jiffies=0;
uint8_t led_flash=LED_OFF;

uint8_t new_second=0;

struct conv_table
{
    uint16_t in;
    int16_t out;
};

const struct conv_table ad_to_temp[]={				
//	AD value		temp*10	
  {	21845	,	0	},
  {	22406	,	100	},
  {	22951	,	200	},
  {	23481	,	300	},
  {	23997	,	400	},
  {	24498	,	500	},
  {	24986	,	600	},
  {	25462	,	700	},
  {	25924	,	800	},
  {	26375	,	900	},
  {	26815	,	1000	},
  {	65535	,	1001	},
};				

const struct conv_table pot_to_temp[]={				
//	AD value		temp*10	
  {	    0	,	POT_TEMP_FROM*10	},
  {	65535	,	POT_TEMP_TO  *10	},
};

int16_t convert(const struct conv_table *table, uint16_t value )
{
    if (value <= table->in) return table->out;
    while (1)
    {
        if (value <= table[1].in)
        {
            int32_t d_i=table[1].in-table[0].in;
            int32_t d_o=table[1].out-table[0].out;
            int32_t d_v=table[1].in-value;
            return table->out+d_v*d_o/d_i;
        }
        table++;
    }
    return 0xffff;
}

int32_t pid_i=0;

uint8_t pid(int16_t error)
{
    static uint16_t prev_error=0;
    static uint16_t prev2_error=0;
    static uint8_t difference_counter=0;
    if (difference_counter--==0)
    {
        difference_counter=PIDd_DIVIDER;
        prev2_error=prev_error;
        prev_error=error;
    }
    int16_t d=prev_error-prev2_error;
    if (!(  error>0 && pid_i > (int32_t)(PIDi_CLAMP*Ki*(1L<<20)) 
         || error<0 && pid_i < -(int32_t)(PIDi_CLAMP*Ki*(1L<<20)) 
       ) )  
        pid_i+=error;
    int32_t o=((int32_t)OUT_MAX<<20)-((int32_t)error*(int32_t)(Kp*(1L<<20)) + pid_i*(int32_t)(Ki*(1L<<20)) + d*(int32_t)(Kd*(1L<<20)));
// clamp output    
    if (o<((int32_t)OUT_MIN<<20)) o=((int32_t)OUT_MIN<<20);
    else if (o>((int32_t)OUT_MAX<<20)) o=(int32_t)OUT_MAX<<20;
    return o>>21;
}



/*
                         Main application
 */
void main(void) 
{
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    
    uint8_t hot=0;
    uint8_t boot_delay=BOOT_DELAY;
    uint8_t power_on_delay=POWER_ON_DELAY;
    uint8_t ctrl_in_delay=0;
    printf("constant_temp started\n");
    while (1) 
    {
        while(!new_second) /* do nothing */;
        new_second=0;
        if (CTRL_IN_GetValue()==0)
        {
            nPOWER_ON_SetHigh();
            power_on_delay=POWER_ON_DELAY;
        }    
        else if (power_on_delay)
        {
            power_on_delay--;
        }
        else
        {
            nPOWER_ON_SetLow();
        }
            
        int16_t temperature=convert(ad_to_temp, ADC_GetConversion(TEMP));
        int16_t reference=convert(pot_to_temp, ADC_GetConversion(POT));
        int16_t error=reference-temperature;
        uint8_t out=pid(reference-temperature);
        if (boot_delay)
        {
            boot_delay--;
            out=0;
        }
        if (temperature<TEMP_FULLSPEED_OFF)
        {
            hot=0;
        }
        else if (hot || temperature>TEMP_FULLSPEED_ON)
        {
            hot=1;
            out=OUT_MAX;
        }

        DAC_SetOutput(out);
        printf("%d,%d,%d,%d,%ld\n", temperature, reference, out, nPOWER_ON_LAT?0:1, pid_i);
        led_flash=(boot_delay)?      LED_FLASH_SHORT:
                  (error>TEMP_RANGE_OK)? LED_FLASH_SLOW:
                  (error<-TEMP_RANGE_OK)?LED_FLASH_FAST:
                                         LED_ON;
    }
}


// 125ms, 8Hz, defined in code configurator
void periodic(void)
{
    jiffies++;
    if (jiffies&0x07 == 0) new_second=1;
    if (led_flash==LED_ON) LED_SetHigh();
    else if (led_flash) LED_LAT=(jiffies&led_flash==led_flash)?1:0;
    else LED_SetLow();
}

/**
 End of File
 */