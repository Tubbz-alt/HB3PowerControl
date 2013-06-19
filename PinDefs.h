//
// Copyright (C) 2013 Blue Chip Technology Ltd
//
// These are the IO pin definitions for the HB3 host board.
//

//
// GPIO pin definitions
//
// Names ending in "_N" denote active low signals 
// These names often (but not always) end with "#" on the schematic.
//

// PORTA GPIO pins
#define PIC_SYS_RESWARM_N   (1<<0)
#define DVI_RST_N           (1<<1)
#define VCC_3VON_N          (1<<2)  // Prefixed with "VCC_" because identifier cannot start with a number
#define VCC_5VON_N          (1<<3)  // Prefixed with "VCC_" because identifier cannot start with a number
#define OTGPWRON            (1<<4)
#define PCIE_DIS_N          (1<<6)
#define DCJACK_OK_N         (1<<7)

#define PORTA_OUTPUTS               (DVI_RST_N | VCC_3VON_N | VCC_5VON_N | OTGPWRON | PCIE_DIS_N)
#define PORTA_OPEN_DRAIN_OUTPUTS    (DVI_RST_N | VCC_3VON_N | VCC_5VON_N | PCIE_DIS_N)
#define PORTA_DIRECTIONS            (0xFFFF & (~PORTA_OUTPUTS))
#define PORTA_DEFAULT_VALUE         (VCC_3VON_N | VCC_5VON_N | PCIE_DIS_N)

// PORTB GPIO pins
#define PEN_IRQ_N           (1<<0)
#define PSON                (1<<1)
#define SYS_RESWARM_N       (1<<2)
#define POE_OK_N            (1<<3)
#define VCC_1V8ON           (1<<4)  // Prefixed with "VCC_" because identifier cannot start with a number
#define PIC_ALARM           (1<<7)
#define VCC1V8_OK           (1<<10)
#define VCC1V5_OK           (1<<11)
#define PCIE_RST_N          (1<<12)
#define USBRSTN             (1<<13)
#define VCC3V3_OK           (1<<14)
#define VCC5_OK             (1<<15)

#define PORTB_OUTPUTS               (SYS_RESWARM_N | VCC_1V8ON | PIC_ALARM | PCIE_RST_N | USBRSTN)
#define PORTB_OPEN_DRAIN_OUTPUTS    (SYS_RESWARM_N | PCIE_RST_N | USBRSTN)
#define PORTB_DIRECTIONS            (0xFFFF & (~PORTB_OUTPUTS))
#define PORTB_DEFAULT_VALUE         (0)

// Pullups
#define SYS_RESWARMN_PULL_UP        (1<<6)
