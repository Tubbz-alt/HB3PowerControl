//
// Copyright (C) 2013 Blue Chip Technology Ltd
//
// These definitions are used in a state machine to control the system power.
//

//
// System State
// The system starts in "SYSTEM_OFF" state 
// and moves between the other states according to various inputs.
//
#define SYSTEM_OFF          0       // Main power supplies are shut down
#define SYSTEM_POWERING_ON  1       // Main power supplies are coming up, but may not yet be stable
#define SYSTEM_RUNNING      2       // Main power supplies are up and stable
