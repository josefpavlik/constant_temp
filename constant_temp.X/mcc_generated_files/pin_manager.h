/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB® Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC16F1704
        Version           :  1.01
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

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set TX aliases
#define TX_TRIS               TRISA0
#define TX_LAT                LATA0
#define TX_PORT               RA0
#define TX_WPU                WPUA0
#define TX_ANS                ANSA0
#define TX_SetHigh()    do { LATA0 = 1; } while(0)
#define TX_SetLow()   do { LATA0 = 0; } while(0)
#define TX_Toggle()   do { LATA0 = ~LATA0; } while(0)
#define TX_GetValue()         RA0
#define TX_SetDigitalInput()    do { TRISA0 = 1; } while(0)
#define TX_SetDigitalOutput()   do { TRISA0 = 0; } while(0)

#define TX_SetPullup()    do { WPUA0 = 1; } while(0)
#define TX_ResetPullup()   do { WPUA0 = 0; } while(0)
#define TX_SetAnalogMode()   do { ANSA0 = 1; } while(0)
#define TX_SetDigitalMode()   do { ANSA0 = 0; } while(0)
// get/set Vref aliases
#define Vref_TRIS               TRISA1
#define Vref_LAT                LATA1
#define Vref_PORT               RA1
#define Vref_WPU                WPUA1
#define Vref_ANS                ANSA1
#define Vref_SetHigh()    do { LATA1 = 1; } while(0)
#define Vref_SetLow()   do { LATA1 = 0; } while(0)
#define Vref_Toggle()   do { LATA1 = ~LATA1; } while(0)
#define Vref_GetValue()         RA1
#define Vref_SetDigitalInput()    do { TRISA1 = 1; } while(0)
#define Vref_SetDigitalOutput()   do { TRISA1 = 0; } while(0)

#define Vref_SetPullup()    do { WPUA1 = 1; } while(0)
#define Vref_ResetPullup()   do { WPUA1 = 0; } while(0)
#define Vref_SetAnalogMode()   do { ANSA1 = 1; } while(0)
#define Vref_SetDigitalMode()   do { ANSA1 = 0; } while(0)
// get/set SWITCH aliases
#define SWITCH_TRIS               TRISA2
#define SWITCH_LAT                LATA2
#define SWITCH_PORT               RA2
#define SWITCH_WPU                WPUA2
#define SWITCH_ANS                ANSA2
#define SWITCH_SetHigh()    do { LATA2 = 1; } while(0)
#define SWITCH_SetLow()   do { LATA2 = 0; } while(0)
#define SWITCH_Toggle()   do { LATA2 = ~LATA2; } while(0)
#define SWITCH_GetValue()         RA2
#define SWITCH_SetDigitalInput()    do { TRISA2 = 1; } while(0)
#define SWITCH_SetDigitalOutput()   do { TRISA2 = 0; } while(0)

#define SWITCH_SetPullup()    do { WPUA2 = 1; } while(0)
#define SWITCH_ResetPullup()   do { WPUA2 = 0; } while(0)
#define SWITCH_SetAnalogMode()   do { ANSA2 = 1; } while(0)
#define SWITCH_SetDigitalMode()   do { ANSA2 = 0; } while(0)
// get/set POT aliases
#define POT_TRIS               TRISA4
#define POT_LAT                LATA4
#define POT_PORT               RA4
#define POT_WPU                WPUA4
#define POT_ANS                ANSA4
#define POT_SetHigh()    do { LATA4 = 1; } while(0)
#define POT_SetLow()   do { LATA4 = 0; } while(0)
#define POT_Toggle()   do { LATA4 = ~LATA4; } while(0)
#define POT_GetValue()         RA4
#define POT_SetDigitalInput()    do { TRISA4 = 1; } while(0)
#define POT_SetDigitalOutput()   do { TRISA4 = 0; } while(0)

#define POT_SetPullup()    do { WPUA4 = 1; } while(0)
#define POT_ResetPullup()   do { WPUA4 = 0; } while(0)
#define POT_SetAnalogMode()   do { ANSA4 = 1; } while(0)
#define POT_SetDigitalMode()   do { ANSA4 = 0; } while(0)
// get/set LED aliases
#define LED_TRIS               TRISA5
#define LED_LAT                LATA5
#define LED_PORT               RA5
#define LED_WPU                WPUA5
#define LED_SetHigh()    do { LATA5 = 1; } while(0)
#define LED_SetLow()   do { LATA5 = 0; } while(0)
#define LED_Toggle()   do { LATA5 = ~LATA5; } while(0)
#define LED_GetValue()         RA5
#define LED_SetDigitalInput()    do { TRISA5 = 1; } while(0)
#define LED_SetDigitalOutput()   do { TRISA5 = 0; } while(0)

#define LED_SetPullup()    do { WPUA5 = 1; } while(0)
#define LED_ResetPullup()   do { WPUA5 = 0; } while(0)
// get/set TEMP aliases
#define TEMP_TRIS               TRISC0
#define TEMP_LAT                LATC0
#define TEMP_PORT               RC0
#define TEMP_WPU                WPUC0
#define TEMP_ANS                ANSC0
#define TEMP_SetHigh()    do { LATC0 = 1; } while(0)
#define TEMP_SetLow()   do { LATC0 = 0; } while(0)
#define TEMP_Toggle()   do { LATC0 = ~LATC0; } while(0)
#define TEMP_GetValue()         RC0
#define TEMP_SetDigitalInput()    do { TRISC0 = 1; } while(0)
#define TEMP_SetDigitalOutput()   do { TRISC0 = 0; } while(0)

#define TEMP_SetPullup()    do { WPUC0 = 1; } while(0)
#define TEMP_ResetPullup()   do { WPUC0 = 0; } while(0)
#define TEMP_SetAnalogMode()   do { ANSC0 = 1; } while(0)
#define TEMP_SetDigitalMode()   do { ANSC0 = 0; } while(0)
// get/set IO_RC1 aliases
#define IO_RC1_TRIS               TRISC1
#define IO_RC1_LAT                LATC1
#define IO_RC1_PORT               RC1
#define IO_RC1_WPU                WPUC1
#define IO_RC1_ANS                ANSC1
#define IO_RC1_SetHigh()    do { LATC1 = 1; } while(0)
#define IO_RC1_SetLow()   do { LATC1 = 0; } while(0)
#define IO_RC1_Toggle()   do { LATC1 = ~LATC1; } while(0)
#define IO_RC1_GetValue()         RC1
#define IO_RC1_SetDigitalInput()    do { TRISC1 = 1; } while(0)
#define IO_RC1_SetDigitalOutput()   do { TRISC1 = 0; } while(0)

#define IO_RC1_SetPullup()    do { WPUC1 = 1; } while(0)
#define IO_RC1_ResetPullup()   do { WPUC1 = 0; } while(0)
#define IO_RC1_SetAnalogMode()   do { ANSC1 = 1; } while(0)
#define IO_RC1_SetDigitalMode()   do { ANSC1 = 0; } while(0)
// get/set OPA1OUT aliases
#define OPA1OUT_TRIS               TRISC2
#define OPA1OUT_LAT                LATC2
#define OPA1OUT_PORT               RC2
#define OPA1OUT_WPU                WPUC2
#define OPA1OUT_ANS                ANSC2
#define OPA1OUT_SetHigh()    do { LATC2 = 1; } while(0)
#define OPA1OUT_SetLow()   do { LATC2 = 0; } while(0)
#define OPA1OUT_Toggle()   do { LATC2 = ~LATC2; } while(0)
#define OPA1OUT_GetValue()         RC2
#define OPA1OUT_SetDigitalInput()    do { TRISC2 = 1; } while(0)
#define OPA1OUT_SetDigitalOutput()   do { TRISC2 = 0; } while(0)

#define OPA1OUT_SetPullup()    do { WPUC2 = 1; } while(0)
#define OPA1OUT_ResetPullup()   do { WPUC2 = 0; } while(0)
#define OPA1OUT_SetAnalogMode()   do { ANSC2 = 1; } while(0)
#define OPA1OUT_SetDigitalMode()   do { ANSC2 = 0; } while(0)
// get/set OPA2OUT aliases
#define OPA2OUT_TRIS               TRISC3
#define OPA2OUT_LAT                LATC3
#define OPA2OUT_PORT               RC3
#define OPA2OUT_WPU                WPUC3
#define OPA2OUT_ANS                ANSC3
#define OPA2OUT_SetHigh()    do { LATC3 = 1; } while(0)
#define OPA2OUT_SetLow()   do { LATC3 = 0; } while(0)
#define OPA2OUT_Toggle()   do { LATC3 = ~LATC3; } while(0)
#define OPA2OUT_GetValue()         RC3
#define OPA2OUT_SetDigitalInput()    do { TRISC3 = 1; } while(0)
#define OPA2OUT_SetDigitalOutput()   do { TRISC3 = 0; } while(0)

#define OPA2OUT_SetPullup()    do { WPUC3 = 1; } while(0)
#define OPA2OUT_ResetPullup()   do { WPUC3 = 0; } while(0)
#define OPA2OUT_SetAnalogMode()   do { ANSC3 = 1; } while(0)
#define OPA2OUT_SetDigitalMode()   do { ANSC3 = 0; } while(0)
// get/set OPA2INNeg aliases
#define OPA2INNeg_TRIS               TRISC4
#define OPA2INNeg_LAT                LATC4
#define OPA2INNeg_PORT               RC4
#define OPA2INNeg_WPU                WPUC4
#define OPA2INNeg_ANS                ANSC4
#define OPA2INNeg_SetHigh()    do { LATC4 = 1; } while(0)
#define OPA2INNeg_SetLow()   do { LATC4 = 0; } while(0)
#define OPA2INNeg_Toggle()   do { LATC4 = ~LATC4; } while(0)
#define OPA2INNeg_GetValue()         RC4
#define OPA2INNeg_SetDigitalInput()    do { TRISC4 = 1; } while(0)
#define OPA2INNeg_SetDigitalOutput()   do { TRISC4 = 0; } while(0)

#define OPA2INNeg_SetPullup()    do { WPUC4 = 1; } while(0)
#define OPA2INNeg_ResetPullup()   do { WPUC4 = 0; } while(0)
#define OPA2INNeg_SetAnalogMode()   do { ANSC4 = 1; } while(0)
#define OPA2INNeg_SetDigitalMode()   do { ANSC4 = 0; } while(0)
// get/set OPA2INPos aliases
#define OPA2INPos_TRIS               TRISC5
#define OPA2INPos_LAT                LATC5
#define OPA2INPos_PORT               RC5
#define OPA2INPos_WPU                WPUC5
#define OPA2INPos_ANS                ANSC5
#define OPA2INPos_SetHigh()    do { LATC5 = 1; } while(0)
#define OPA2INPos_SetLow()   do { LATC5 = 0; } while(0)
#define OPA2INPos_Toggle()   do { LATC5 = ~LATC5; } while(0)
#define OPA2INPos_GetValue()         RC5
#define OPA2INPos_SetDigitalInput()    do { TRISC5 = 1; } while(0)
#define OPA2INPos_SetDigitalOutput()   do { TRISC5 = 0; } while(0)

#define OPA2INPos_SetPullup()    do { WPUC5 = 1; } while(0)
#define OPA2INPos_ResetPullup()   do { WPUC5 = 0; } while(0)
#define OPA2INPos_SetAnalogMode()   do { ANSC5 = 1; } while(0)
#define OPA2INPos_SetDigitalMode()   do { ANSC5 = 0; } while(0)

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    GPIO and peripheral I/O initialization
 * @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize(void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);

#endif // PIN_MANAGER_H
/**
 End of File
 */