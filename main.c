//
// Copyright (C) 2013 Blue Chip Technology Ltd
//
// Main power control loop
//

#include <GenericTypeDefs.h>
#include "p24Fxxxx.h"
#include <timer.h>
#include "i2c.h"

#include "SystemStates.h"
#include "Protocol.h"
#include "PinDefs.h"
#include "Version.h"

#define TRUE 1
#define FALSE 0

#define FCY 4000000UL
#include "libpic30.h"

#define DEBOUNCE        1000 //DPR 200ms causes reset presses to be missed. 1000 * 50us = 50 ms


/* Macros for Configuration Fuse Registers (copied from PIC24F08KA102 header file):*/

_FBS(BWRP_OFF & BSS_OFF);                           // Boot Segment Write Protect (off)
                                                    // Boot Segment Security (off)

_FGS(GWRP_OFF & GCP_OFF);                           // General Write Protect (off)
                                                    // General Code Protection (off)

_FOSCSEL(FNOSC_FRC & IESO_OFF);                     // Oscillator Select (Internal Fast RC)
                                                    // Internal External Switch Over (off)

_FOSC(POSCMOD_NONE & OSCIOFNC_ON & FCKSM_CSDCMD);   // Primary Oscillator Mode (disabled)
                                                    // CLKO Output (disabled - pin is normal I/O)
                                                    // Clock Switching & Monitoring (both disabled)

_FWDT(FWDTEN_OFF);                                  // Watch-Dog Timer Enable (disabled)

_FPOR(MCLRE_ON & BORV_V18 & I2C1SEL_PRI & PWRTEN_ON & BOREN_BOR3);
                                                    // MCLR pin Enable (enabled)
                                                    // BOR Voltage (1.8V)
                                                    // I2C pins (use default)
                                                    // Power-up Timer (enabled)
                                                    // BOR Mode (enabled in hardware, SBOREN bit disabled)

_FICD(ICS_PGx3);                                    // ICD pins (use PGC3/PGD3)

_FDS(DSWDTEN_OFF);                                  // Deep Sleep Watchdog (disabled)


/* Global Variables and Functions */
int main (void);
void SystemSetup();
void IO_Init(void);
void I2C_Init();
void AssertResets(int active);
void EnablePowerSupplies(int on);
int  IsPowerGood(void);
WORD GetPowerSwitchState();
WORD GetResetSwitchState();
WORD GetDebouncedPowerSwitchState();
WORD GetDebouncedResetSwitchState();

void __attribute__((__interrupt__)) __attribute__((no_auto_psv))  _SI2C1Interrupt(void); /*Declare external interrupt ISRs*/

BYTE    SystemState = SYSTEM_OFF;       // State variable for system power control - see "SystemStates.h"
BYTE    SystemResetRequested = 0;       // Flag to force System reset in main loop
BYTE    DVIResetRequested = 0;          // Flag to force DVI reset in main loop from I2C ISR
BYTE    PCIEResetRequested = 0;         // Flag to force PCIE reset in main loop from I2C ISR

//Watchdog Globals
BYTE    WatchDogTimerRunning = 0;
BYTE    WatchDogTickInterval = TICK_INTERVAL_SEC;
BYTE    WatchDogTimeOut = 0xFF;
BYTE    WatchDogEnable  = 0x00;
BYTE    WatchDogCurrentCount = 0x00;

//Timer Globals
unsigned long TimerCountValueForOneWatchDogTick = 0;
unsigned long CurrentTimerCount = 0;

//Misc I2c Registers
BYTE    ScratchpadRegister;



int main (void)
{
    SystemSetup();

    //Delay to improve power on reliability
    __delay_ms(100);
    
    while(1)  // Our main program loop
    {
        switch(SystemState)
        {
            case SYSTEM_OFF:

                WatchDogEnable = 0;
                SystemResetRequested = 0;
                WatchDogTimerRunning = 0;

                AssertResets(TRUE);

                if(GetPowerSwitchState() > 0)
                {
                    if(GetDebouncedPowerSwitchState() > 0 || ((PORTB & PEN_IRQ_N) == 0))
                    {
                        __delay_ms(100);
                        EnablePowerSupplies(TRUE);
                        SystemState = SYSTEM_POWERING_ON;
                    }
                }
            break;

            case SYSTEM_POWERING_ON:

                if(GetPowerSwitchState() == 0)
                {
                    if(GetDebouncedPowerSwitchState() == 0)
                    {
                        EnablePowerSupplies(FALSE);
                        SystemState = SYSTEM_OFF;
                        break;
                    }
                }
                
                if (IsPowerGood() && (GetDebouncedResetSwitchState() > 0))
                {
                    __delay_ms(100);
                    AssertResets(FALSE);
                    SystemState = SYSTEM_RUNNING;
                }
            break;

            case SYSTEM_RUNNING:

                if(GetPowerSwitchState() == 0)
                {
                    if(GetDebouncedPowerSwitchState() == 0)
                    {
                        EnablePowerSupplies(FALSE);
                        SystemState = SYSTEM_OFF;
                    }
                }
                else if (GetResetSwitchState() == 0)
                {
                    if(GetDebouncedResetSwitchState() == 0)
                    {
                        AssertResets(TRUE);
                        SystemState = SYSTEM_POWERING_ON;
                    }
                }
            break;

            default:    // Should never happen!
                SystemState = SYSTEM_OFF;
            break;
        }

        if (SystemResetRequested > 0)
        {
            SystemResetRequested = 0;
            AssertResets(TRUE);
            SystemState = SYSTEM_POWERING_ON;
            continue;
        }

        if (DVIResetRequested > 0)
        {
            DVIResetRequested = 0;
            LATA = LATA & ~DVI_RST_N;
            __delay_ms(10);
            LATA = LATA | DVI_RST_N;
        }
        
        if (PCIEResetRequested > 0)
        {
            PCIEResetRequested = 0;
            LATB = LATB & ~PCIE_RST_N;
            __delay_ms(2);              // Spec says minimum 1ms
            LATB = LATB | PCIE_RST_N;
        }

        if(WatchDogEnable == 1)
        {
            if(WatchDogTimerRunning == 0)
            {
                switch(WatchDogTickInterval)
                {
                    case TICK_INTERVAL_4MS:
                        TimerCountValueForOneWatchDogTick = 10;  //4ms
                    break;
                    case TICK_INTERVAL_SEC:
                        TimerCountValueForOneWatchDogTick = 2500; //1 second
                    break;
                    case TICK_INTERVAL_MIN:
                        TimerCountValueForOneWatchDogTick = 150000; //1 minute
                    break;
                    default:
                        TimerCountValueForOneWatchDogTick = 2500; //1 second
                    break;
                }

                ConfigIntTimer1(T1_INT_PRIOR_1 & T1_INT_ON);
                WriteTimer1(0);
                WatchDogCurrentCount = 0;
                OpenTimer1(T1_ON & T1_GATE_OFF & T1_IDLE_STOP & T1_PS_1_8 & T1_SYNC_EXT_OFF & T1_SOURCE_INT, 200);
                WatchDogTimerRunning = 1;
            }
        }
        else
        {   
            ConfigIntTimer1(T1_INT_PRIOR_1 & T1_INT_OFF);
            CloseTimer1();
            WatchDogTimerRunning = 0;
            WatchDogCurrentCount = 0;
            CurrentTimerCount = 0;
        }
    }
}


///
/// Drives the reset outputs.
///
/// @param active non-zero to drive the outputs active, zero for inactive
///
void AssertResets(int active)
{
    WORD value;

    if (active)
    {
        value = LATB;
        value &= (~SYS_RESWARM_N);  // drive pin low
        value &= (~USBRSTN);        // drive pin low
        LATB = value;
    }
    else
    {
        value = LATB;
        value |= SYS_RESWARM_N;     // drive pin high
        value |= USBRSTN;           // drive pin high
        LATB = value;
    }
}


///
/// Drives the (active low) PCIE_DIS_N line.
///
/// @param high non-zero to drive the output high (i.e. inactive), zero for low (i.e. active)
///
void DrivePCIE_DIS_N(unsigned char high)
{
    WORD value;

    if (high)
    {
        value = LATA;
        value |= PCIE_DIS_N;            // drive pin high
        LATA = value;
    }
    else
    {
        value = LATA;
        value &= (~PCIE_DIS_N);         // drive pin low
        LATA = value;
    }
}


///
/// Drives the power supply enable outputs.
///
/// @param on non-zero to drive the outputs active, zero for inactive
///
void EnablePowerSupplies(int on)
{
    WORD value;

    if (on)
    {
        value = LATA;
        value &= (~VCC_3VON_N);     // drive pin low
        value &= (~VCC_5VON_N);     // drive pin low
        LATA = value;

        value = LATB;
        value |= VCC_1V8ON;         // drive pin high
        LATB = value;
        
        __delay_ms(50);             // Allow voltage monitor IC to stabilise
    }
    else
    {
        value = LATB;
        value &= (~VCC_1V8ON);      // drive pin low
        LATB = value;

        value = LATA;
        value |= VCC_3VON_N;        // drive pin high
        value |= VCC_5VON_N;        // drive pin high
        LATA = value;
    }
}


///
/// Determines whether the power supplies have come up by reading the appropriate input pins.
///
/// @return non-zero if all power supplies are good
/// @return zero if one or more power supplies are not good
///
int IsPowerGood(void)
{
    // Check 5V power
    if ((PORTB & VCC5_OK) == 0)
        return 0;

    if ((PORTB & VCC3V3_OK) == 0)
        return 0;

    if ((PORTB & VCC1V8_OK) == 0)
        return 0;

    if ((PORTB & VCC1V5_OK) == 0)
        return 0;

    return 1;
}


///
/// Determines whether the power switch (PSON input) is on.
///
/// @return non-zero if power switch is ON
/// @return zero if power switch is OFF
///
WORD GetPowerSwitchState()
{
    return (PORTB & PSON);
}


///
/// Determines whether the power switch (PSON input) is on and stable.
///
/// Note that this function may take an indeterminate length of time to
/// establish the stable state of the signal if the signal is fluctuating.
///
/// @return non-zero if power switch is ON
/// @return zero if power switch is OFF
///
WORD GetDebouncedPowerSwitchState()
{
    WORD wInitialValue; 
    WORD wReadValue;
    WORD wSuccessiveReads;

    wSuccessiveReads = 0;
    wInitialValue = GetPowerSwitchState();

    while(1)
    {
        wReadValue = GetPowerSwitchState();
        if(wInitialValue == wReadValue)
        {   
            wSuccessiveReads++;
        }
        else
        {       
            wSuccessiveReads = 0;
            wInitialValue = wReadValue;
        }

        if(wSuccessiveReads >= DEBOUNCE)
        {
            break;
        }
        __delay_us(50);
    }

    return wInitialValue;
}

///
/// Determines whether the reset switch (PIC_SYS_RESWARM_N input) is on.
///
/// @return non-zero if reset switch is ON
/// @return zero if reset switch is OFF
///
WORD GetResetSwitchState()
{
    return (PORTA & PIC_SYS_RESWARM_N);
}


///
/// Determines whether the reset switch (PIC_SYS_RESWARM_N input) is on.
///
/// Note that this function may take an indeterminate length of time to
/// establish the stable state of the signal if the signal is fluctuating.
///
/// @return non-zero if reset switch is ON
/// @return zero if reset switch is OFF
///
WORD GetDebouncedResetSwitchState()
{
    WORD wInitialValue; 
    WORD wReadValue;
    WORD wSuccessiveReads;

    wSuccessiveReads = 0;
    wInitialValue = GetResetSwitchState();

    while(1)
    {
        wReadValue = GetResetSwitchState();
        if(wInitialValue == wReadValue)
        {   
            wSuccessiveReads++;
        }
        else
        {       
            wSuccessiveReads = 0;
            wInitialValue = wReadValue;
        }

        if(wSuccessiveReads >= DEBOUNCE)
        {
            break;
        }
        __delay_us(50);
    }

    return wInitialValue;
}


void SystemSetup()
{
    SystemState = SYSTEM_OFF;
    WatchDogTickInterval = TICK_INTERVAL_SEC;
    WatchDogTimeOut = 0xFF;
    WatchDogEnable  = 0x00;
    WatchDogCurrentCount = 0x00;

    IO_Init();
    I2C_Init();
}


void IO_Init()
{
    //All pins are digital
    AD1PCFG = 0xFFFF;

    // Enable pullups
    CNPU1 = SYS_RESWARMN_PULL_UP;

    // Set up PortA
    PORTA = PORTA_DEFAULT_VALUE;        // Set all outputs to default value
    LATA  = PORTA_DEFAULT_VALUE;        // Set all outputs to default value
    ODCA  = PORTA_OPEN_DRAIN_OUTPUTS;   // Set output pins to open drain where applicable
    TRISA = PORTA_DIRECTIONS;           // Set pin directions
    Nop();

    // Set up PortB
    PORTB = PORTB_DEFAULT_VALUE;        // Set all outputs to default value
    LATB  = PORTB_DEFAULT_VALUE;        // Set all outputs to default value
    ODCB  = PORTB_OPEN_DRAIN_OUTPUTS;   // Set output pins to open drain where applicable
    TRISB = PORTB_DIRECTIONS;           // Set pin directions
    Nop();

}

void I2C_Init()
{
    unsigned int config2, config1;

    config1=I2C_ON & I2C_IDLE_CON & I2C_CLK_HLD & I2C_IPMI_DIS & I2C_7BIT_ADD & I2C_SLW_DIS & I2C_SM_DIS & I2C_GCALL_DIS & I2C_STR_EN & I2C_NACK & I2C_ACK_DIS & I2C_RCV_DIS & I2C_STOP_DIS & I2C_RESTART_DIS & I2C_START_DIS; 
    config2 = 0x12; //404khz

    OpenI2C1(config1,config2);

    I2C1MSK = 0;
    I2C1ADD = I2C_SLAVE_ADDRESS;

    ConfigIntI2C1(SI2C_INT_ON & SI2C_INT_PRI_5 & MI2C_INT_OFF);
    
    IdleI2C1(); 

    I2C1CONbits.IPMIEN = 0;
}

 

/*
_SI2C1Interrupt() is the I2C slave interrupt service routine (ISR).
The routine must have global scope in order to be an ISR.
The ISR name is chosen from the device linker script.
*/
void __attribute__((__interrupt__)) __attribute__((no_auto_psv)) _SI2C1Interrupt(void)
{
    #define REGISTER_ADDRESS 0 
    #define REGISTER_VALUE 1

    unsigned char byte; 
    unsigned char slaveStatus;
    static unsigned char NextByteType;
    static unsigned char registerAddress;
    static unsigned char registerData;
      
    slaveStatus=(I2C1STATbits.TBF)+(I2C1STATbits.RBF*2)+(I2C1STATbits.R_W*4)+(I2C1STATbits.D_A*8); 
    
    if(slaveStatus==0x0A) // write mode - the master is writing to the slave 
    { 
        byte=SlaveReadI2C1(); 
        if(NextByteType == REGISTER_ADDRESS) // first byte of message - it's the register address 
        { 
            registerAddress=byte; 
            NextByteType=REGISTER_VALUE; 
        } 
        else if(NextByteType == REGISTER_VALUE) // second byte of message - it's the register contents 
        { 
            registerData=byte;
            switch(registerAddress)
            {       
                case WATCHDOG_TICK_INTERVAL:
                    WatchDogTickInterval = registerData;
                break;
                case WATCHDOG_TIME_OUT:
                    WatchDogTimeOut = registerData;
                break;
                case WATCHDOG_ENABLE:
                    if(registerData > 0)
                    {
                        WatchDogEnable = 1;
                    }
                    else
                    {           
                        WatchDogEnable = 0;
                    }
                break;
                case WATCHDOG_REFRESH:
                    WriteTimer1(0);
                    CurrentTimerCount = 0;
                    WatchDogCurrentCount = 0;
                break;
                case SCRATCHPAD:
                    ScratchpadRegister = registerData;
                break;
                case SYSTEM_RESET:
                    SystemResetRequested = 1;
                break;
                case DVI_RESET:
                    DVIResetRequested = 1;
                break;
                case PCIE_RESET:
                    PCIEResetRequested = 1;
                break;
                case PCIE_DIS:
                    DrivePCIE_DIS_N(registerData);
                break;
            }
        } 
    } 
    else if(slaveStatus==0x2) // write mode - we have just had an address match 
    { 
        SlaveReadI2C1(); //dummy read
        NextByteType = REGISTER_ADDRESS; 
    } 
    else if(slaveStatus==0x6) // read mode - it wants a byte 
    { 
        switch(registerAddress)
        {   
            case FIRMWARE_ID:
                registerData = FIRMWARE_ID_VALUE;
            break;  
            case FIRMWARE_VERSION_MAJOR:
                registerData = FIRMWARE_VERSION_MAJOR_VALUE;
            break;
            case FIRMWARE_VERSION_MINOR:
                registerData = FIRMWARE_VERSION_MINOR_VALUE;
            break;
            case WATCHDOG_TICK_INTERVAL:
                registerData = WatchDogTickInterval;
            break;
            case WATCHDOG_TIME_OUT:
                registerData = WatchDogTimeOut;
            break;
            case WATCHDOG_ENABLE:
                registerData = WatchDogEnable;
            break;
            case SCRATCHPAD:
                registerData = ScratchpadRegister;
            break;
            default:
                registerData = ERROR_REPLY_VALUE;
            break;  
        }
            
        SlaveWriteI2C1(registerData); 
    } 
    
    I2C1CONbits.SCLREL=1; 
    IFS1bits.SI2C1IF=0; 
}


void __attribute__((__interrupt__)) __attribute__((no_auto_psv)) _T1Interrupt(void)
{
    WriteTimer1(0);
    CurrentTimerCount++;
    if(CurrentTimerCount >= TimerCountValueForOneWatchDogTick)
    {   
        WatchDogCurrentCount++;
        CurrentTimerCount = 0;
    }

    if(WatchDogCurrentCount >= WatchDogTimeOut)
    {
        if(WatchDogTimerRunning == 1)
        {
            SystemResetRequested = 1;
            CurrentTimerCount = 0;
            WatchDogCurrentCount = 0;
        }
    }
    
    IFS0bits.T1IF = 0;    /* Clear Timer interrupt flag */
}
