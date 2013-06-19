//
// Copyright (C) 2013 Blue Chip Technology Ltd
//
// These definitions implement the I2C protocol used to
// communicate between the main CPU and the PIC firmware.
//


// I2C Slave Address

#define I2C_SLAVE_ADDRESS       0x56


// I2C Registers / Sub addresses

#define FIRMWARE_ID             0x00
#define FIRMWARE_VERSION_MAJOR  0x01
#define FIRMWARE_VERSION_MINOR  0x02
#define WATCHDOG_TICK_INTERVAL  0x03
#define WATCHDOG_TIME_OUT       0x04
#define WATCHDOG_ENABLE         0x05
#define WATCHDOG_REFRESH        0x06    // write-only
#define SCRATCHPAD              0x07
#define SYSTEM_RESET            0x08    // write-only
#define DVI_RESET               0x09    // write-only
#define PCIE_RESET              0x0A    // write-only
#define PCIE_DIS                0x0B    // write-only


// Reply value for read from unknown sub address 
// or read from write-only sub address

#define ERROR_REPLY_VALUE       0xFF


// Valid Watchdog Tick Intervals

#define TICK_INTERVAL_4MS       0x01
#define TICK_INTERVAL_SEC       0x02
#define TICK_INTERVAL_MIN       0x03
