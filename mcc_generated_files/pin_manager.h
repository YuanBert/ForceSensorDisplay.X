/**
  System Interrupts Generated Driver File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the generated manager file for the MPLAB(c) Code Configurator device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description:
    This source file provides implementations for MPLAB(c) Code Configurator interrupts.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 4.45
        Device            :  PIC32MX150F128D
        Version           :  1.02
    The generated drivers are tested against the following:
        Compiler          :  XC32 1.42
        MPLAB             :  MPLAB X 3.55

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

#ifndef _PIN_MANAGER_H
#define _PIN_MANAGER_H
/**
    Section: Includes
*/
#include <xc.h>
/**
    Section: Device Pin Macros
*/
/**
  @Summary
    Sets the GPIO pin, RC0, high using LATCbits.LATC0.

  @Description
    Sets the GPIO pin, RC0, high using LATCbits.LATC0.

  @Preconditions
    The RC0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC0 high (1)
    IO_RC0A_SetHigh();
    </code>

*/
#define IO_RC0A_SetHigh()          LATCbits.LATC0 = 1
/**
  @Summary
    Sets the GPIO pin, RC0, low using LATCbits.LATC0.

  @Description
    Sets the GPIO pin, RC0, low using LATCbits.LATC0.

  @Preconditions
    The RC0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC0 low (0)
    IO_RC0A_SetLow();
    </code>

*/
#define IO_RC0A_SetLow()           LATCbits.LATC0 = 0
/**
  @Summary
    Toggles the GPIO pin, RC0, using LATCbits.LATC0.

  @Description
    Toggles the GPIO pin, RC0, using LATCbits.LATC0.

  @Preconditions
    The RC0 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC0
    IO_RC0A_Toggle();
    </code>

*/
#define IO_RC0A_Toggle()           LATCbits.LATC0 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC0.

  @Description
    Reads the value of the GPIO pin, RC0.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC0
    postValue = IO_RC0A_GetValue();
    </code>

*/
#define IO_RC0A_GetValue()         PORTCbits.RC0
/**
  @Summary
    Configures the GPIO pin, RC0, as an input.

  @Description
    Configures the GPIO pin, RC0, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC0 as an input
    IO_RC0A_SetDigitalInput();
    </code>

*/
#define IO_RC0A_SetDigitalInput()  TRISCbits.TRISC0 = 1
/**
  @Summary
    Configures the GPIO pin, RC0, as an output.

  @Description
    Configures the GPIO pin, RC0, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC0 as an output
    IO_RC0A_SetDigitalOutput();
    </code>

*/
#define IO_RC0A_SetDigitalOutput() TRISCbits.TRISC0 = 0
/**
  @Summary
    Sets the GPIO pin, RC1, high using LATCbits.LATC1.

  @Description
    Sets the GPIO pin, RC1, high using LATCbits.LATC1.

  @Preconditions
    The RC1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC1 high (1)
    IO_RC1B_SetHigh();
    </code>

*/
#define IO_RC1B_SetHigh()          LATCbits.LATC1 = 1
/**
  @Summary
    Sets the GPIO pin, RC1, low using LATCbits.LATC1.

  @Description
    Sets the GPIO pin, RC1, low using LATCbits.LATC1.

  @Preconditions
    The RC1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC1 low (0)
    IO_RC1B_SetLow();
    </code>

*/
#define IO_RC1B_SetLow()           LATCbits.LATC1 = 0
/**
  @Summary
    Toggles the GPIO pin, RC1, using LATCbits.LATC1.

  @Description
    Toggles the GPIO pin, RC1, using LATCbits.LATC1.

  @Preconditions
    The RC1 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC1
    IO_RC1B_Toggle();
    </code>

*/
#define IO_RC1B_Toggle()           LATCbits.LATC1 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC1.

  @Description
    Reads the value of the GPIO pin, RC1.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC1
    postValue = IO_RC1B_GetValue();
    </code>

*/
#define IO_RC1B_GetValue()         PORTCbits.RC1
/**
  @Summary
    Configures the GPIO pin, RC1, as an input.

  @Description
    Configures the GPIO pin, RC1, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC1 as an input
    IO_RC1B_SetDigitalInput();
    </code>

*/
#define IO_RC1B_SetDigitalInput()  TRISCbits.TRISC1 = 1
/**
  @Summary
    Configures the GPIO pin, RC1, as an output.

  @Description
    Configures the GPIO pin, RC1, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC1 as an output
    IO_RC1B_SetDigitalOutput();
    </code>

*/
#define IO_RC1B_SetDigitalOutput() TRISCbits.TRISC1 = 0
/**
  @Summary
    Sets the GPIO pin, RC2, high using LATCbits.LATC2.

  @Description
    Sets the GPIO pin, RC2, high using LATCbits.LATC2.

  @Preconditions
    The RC2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC2 high (1)
    IO_RC2C_SetHigh();
    </code>

*/
#define IO_RC2C_SetHigh()          LATCbits.LATC2 = 1
/**
  @Summary
    Sets the GPIO pin, RC2, low using LATCbits.LATC2.

  @Description
    Sets the GPIO pin, RC2, low using LATCbits.LATC2.

  @Preconditions
    The RC2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC2 low (0)
    IO_RC2C_SetLow();
    </code>

*/
#define IO_RC2C_SetLow()           LATCbits.LATC2 = 0
/**
  @Summary
    Toggles the GPIO pin, RC2, using LATCbits.LATC2.

  @Description
    Toggles the GPIO pin, RC2, using LATCbits.LATC2.

  @Preconditions
    The RC2 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC2
    IO_RC2C_Toggle();
    </code>

*/
#define IO_RC2C_Toggle()           LATCbits.LATC2 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC2.

  @Description
    Reads the value of the GPIO pin, RC2.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC2
    postValue = IO_RC2C_GetValue();
    </code>

*/
#define IO_RC2C_GetValue()         PORTCbits.RC2
/**
  @Summary
    Configures the GPIO pin, RC2, as an input.

  @Description
    Configures the GPIO pin, RC2, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC2 as an input
    IO_RC2C_SetDigitalInput();
    </code>

*/
#define IO_RC2C_SetDigitalInput()  TRISCbits.TRISC2 = 1
/**
  @Summary
    Configures the GPIO pin, RC2, as an output.

  @Description
    Configures the GPIO pin, RC2, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC2 as an output
    IO_RC2C_SetDigitalOutput();
    </code>

*/
#define IO_RC2C_SetDigitalOutput() TRISCbits.TRISC2 = 0
/**
  @Summary
    Sets the GPIO pin, RC3, high using LATCbits.LATC3.

  @Description
    Sets the GPIO pin, RC3, high using LATCbits.LATC3.

  @Preconditions
    The RC3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC3 high (1)
    IO_RC3D_SetHigh();
    </code>

*/
#define IO_RC3D_SetHigh()          LATCbits.LATC3 = 1
/**
  @Summary
    Sets the GPIO pin, RC3, low using LATCbits.LATC3.

  @Description
    Sets the GPIO pin, RC3, low using LATCbits.LATC3.

  @Preconditions
    The RC3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC3 low (0)
    IO_RC3D_SetLow();
    </code>

*/
#define IO_RC3D_SetLow()           LATCbits.LATC3 = 0
/**
  @Summary
    Toggles the GPIO pin, RC3, using LATCbits.LATC3.

  @Description
    Toggles the GPIO pin, RC3, using LATCbits.LATC3.

  @Preconditions
    The RC3 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC3
    IO_RC3D_Toggle();
    </code>

*/
#define IO_RC3D_Toggle()           LATCbits.LATC3 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC3.

  @Description
    Reads the value of the GPIO pin, RC3.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC3
    postValue = IO_RC3D_GetValue();
    </code>

*/
#define IO_RC3D_GetValue()         PORTCbits.RC3
/**
  @Summary
    Configures the GPIO pin, RC3, as an input.

  @Description
    Configures the GPIO pin, RC3, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC3 as an input
    IO_RC3D_SetDigitalInput();
    </code>

*/
#define IO_RC3D_SetDigitalInput()  TRISCbits.TRISC3 = 1
/**
  @Summary
    Configures the GPIO pin, RC3, as an output.

  @Description
    Configures the GPIO pin, RC3, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC3 as an output
    IO_RC3D_SetDigitalOutput();
    </code>

*/
#define IO_RC3D_SetDigitalOutput() TRISCbits.TRISC3 = 0
/**
  @Summary
    Sets the GPIO pin, RC4, high using LATCbits.LATC4.

  @Description
    Sets the GPIO pin, RC4, high using LATCbits.LATC4.

  @Preconditions
    The RC4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC4 high (1)
    IO_RC4E_SetHigh();
    </code>

*/
#define IO_RC4E_SetHigh()          LATCbits.LATC4 = 1
/**
  @Summary
    Sets the GPIO pin, RC4, low using LATCbits.LATC4.

  @Description
    Sets the GPIO pin, RC4, low using LATCbits.LATC4.

  @Preconditions
    The RC4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC4 low (0)
    IO_RC4E_SetLow();
    </code>

*/
#define IO_RC4E_SetLow()           LATCbits.LATC4 = 0
/**
  @Summary
    Toggles the GPIO pin, RC4, using LATCbits.LATC4.

  @Description
    Toggles the GPIO pin, RC4, using LATCbits.LATC4.

  @Preconditions
    The RC4 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC4
    IO_RC4E_Toggle();
    </code>

*/
#define IO_RC4E_Toggle()           LATCbits.LATC4 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC4.

  @Description
    Reads the value of the GPIO pin, RC4.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC4
    postValue = IO_RC4E_GetValue();
    </code>

*/
#define IO_RC4E_GetValue()         PORTCbits.RC4
/**
  @Summary
    Configures the GPIO pin, RC4, as an input.

  @Description
    Configures the GPIO pin, RC4, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC4 as an input
    IO_RC4E_SetDigitalInput();
    </code>

*/
#define IO_RC4E_SetDigitalInput()  TRISCbits.TRISC4 = 1
/**
  @Summary
    Configures the GPIO pin, RC4, as an output.

  @Description
    Configures the GPIO pin, RC4, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC4 as an output
    IO_RC4E_SetDigitalOutput();
    </code>

*/
#define IO_RC4E_SetDigitalOutput() TRISCbits.TRISC4 = 0
/**
  @Summary
    Sets the GPIO pin, RC5, high using LATCbits.LATC5.

  @Description
    Sets the GPIO pin, RC5, high using LATCbits.LATC5.

  @Preconditions
    The RC5 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC5 high (1)
    IO_RC5F_SetHigh();
    </code>

*/
#define IO_RC5F_SetHigh()          LATCbits.LATC5 = 1
/**
  @Summary
    Sets the GPIO pin, RC5, low using LATCbits.LATC5.

  @Description
    Sets the GPIO pin, RC5, low using LATCbits.LATC5.

  @Preconditions
    The RC5 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC5 low (0)
    IO_RC5F_SetLow();
    </code>

*/
#define IO_RC5F_SetLow()           LATCbits.LATC5 = 0
/**
  @Summary
    Toggles the GPIO pin, RC5, using LATCbits.LATC5.

  @Description
    Toggles the GPIO pin, RC5, using LATCbits.LATC5.

  @Preconditions
    The RC5 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC5
    IO_RC5F_Toggle();
    </code>

*/
#define IO_RC5F_Toggle()           LATCbits.LATC5 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC5.

  @Description
    Reads the value of the GPIO pin, RC5.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC5
    postValue = IO_RC5F_GetValue();
    </code>

*/
#define IO_RC5F_GetValue()         PORTCbits.RC5
/**
  @Summary
    Configures the GPIO pin, RC5, as an input.

  @Description
    Configures the GPIO pin, RC5, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC5 as an input
    IO_RC5F_SetDigitalInput();
    </code>

*/
#define IO_RC5F_SetDigitalInput()  TRISCbits.TRISC5 = 1
/**
  @Summary
    Configures the GPIO pin, RC5, as an output.

  @Description
    Configures the GPIO pin, RC5, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC5 as an output
    IO_RC5F_SetDigitalOutput();
    </code>

*/
#define IO_RC5F_SetDigitalOutput() TRISCbits.TRISC5 = 0
/**
  @Summary
    Sets the GPIO pin, RC6, high using LATCbits.LATC6.

  @Description
    Sets the GPIO pin, RC6, high using LATCbits.LATC6.

  @Preconditions
    The RC6 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC6 high (1)
    IO_RC6G_SetHigh();
    </code>

*/
#define IO_RC6G_SetHigh()          LATCbits.LATC6 = 1
/**
  @Summary
    Sets the GPIO pin, RC6, low using LATCbits.LATC6.

  @Description
    Sets the GPIO pin, RC6, low using LATCbits.LATC6.

  @Preconditions
    The RC6 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC6 low (0)
    IO_RC6G_SetLow();
    </code>

*/
#define IO_RC6G_SetLow()           LATCbits.LATC6 = 0
/**
  @Summary
    Toggles the GPIO pin, RC6, using LATCbits.LATC6.

  @Description
    Toggles the GPIO pin, RC6, using LATCbits.LATC6.

  @Preconditions
    The RC6 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC6
    IO_RC6G_Toggle();
    </code>

*/
#define IO_RC6G_Toggle()           LATCbits.LATC6 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC6.

  @Description
    Reads the value of the GPIO pin, RC6.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC6
    postValue = IO_RC6G_GetValue();
    </code>

*/
#define IO_RC6G_GetValue()         PORTCbits.RC6
/**
  @Summary
    Configures the GPIO pin, RC6, as an input.

  @Description
    Configures the GPIO pin, RC6, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC6 as an input
    IO_RC6G_SetDigitalInput();
    </code>

*/
#define IO_RC6G_SetDigitalInput()  TRISCbits.TRISC6 = 1
/**
  @Summary
    Configures the GPIO pin, RC6, as an output.

  @Description
    Configures the GPIO pin, RC6, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC6 as an output
    IO_RC6G_SetDigitalOutput();
    </code>

*/
#define IO_RC6G_SetDigitalOutput() TRISCbits.TRISC6 = 0
/**
  @Summary
    Sets the GPIO pin, RC7, high using LATCbits.LATC7.

  @Description
    Sets the GPIO pin, RC7, high using LATCbits.LATC7.

  @Preconditions
    The RC7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC7 high (1)
    IO_RC7DP_SetHigh();
    </code>

*/
#define IO_RC7DP_SetHigh()          LATCbits.LATC7 = 1
/**
  @Summary
    Sets the GPIO pin, RC7, low using LATCbits.LATC7.

  @Description
    Sets the GPIO pin, RC7, low using LATCbits.LATC7.

  @Preconditions
    The RC7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC7 low (0)
    IO_RC7DP_SetLow();
    </code>

*/
#define IO_RC7DP_SetLow()           LATCbits.LATC7 = 0
/**
  @Summary
    Toggles the GPIO pin, RC7, using LATCbits.LATC7.

  @Description
    Toggles the GPIO pin, RC7, using LATCbits.LATC7.

  @Preconditions
    The RC7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC7
    IO_RC7DP_Toggle();
    </code>

*/
#define IO_RC7DP_Toggle()           LATCbits.LATC7 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC7.

  @Description
    Reads the value of the GPIO pin, RC7.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC7
    postValue = IO_RC7DP_GetValue();
    </code>

*/
#define IO_RC7DP_GetValue()         PORTCbits.RC7
/**
  @Summary
    Configures the GPIO pin, RC7, as an input.

  @Description
    Configures the GPIO pin, RC7, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC7 as an input
    IO_RC7DP_SetDigitalInput();
    </code>

*/
#define IO_RC7DP_SetDigitalInput()  TRISCbits.TRISC7 = 1
/**
  @Summary
    Configures the GPIO pin, RC7, as an output.

  @Description
    Configures the GPIO pin, RC7, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC7 as an output
    IO_RC7DP_SetDigitalOutput();
    </code>

*/
#define IO_RC7DP_SetDigitalOutput() TRISCbits.TRISC7 = 0

/**
    Section: Function Prototypes
*/
/**
  @Summary
    Configures the pin settings of the PIC32MX150F128D
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description
    This is the generated manager file for the MPLAB(c) Code Configurator device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    void SYSTEM_Initialize(void)
    {
        // Other initializers are called from this function
        PIN_MANAGER_Initialize();
    }
    </code>

*/
void PIN_MANAGER_Initialize(void);

#endif
