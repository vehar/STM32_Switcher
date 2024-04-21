// switches.h
#ifndef SWITCHES_H_
#define SWITCHES_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h> // For strcmp

typedef struct
{
    uint8_t V1;
    uint8_t V2;
    uint8_t V3;
} RFC_sel;

typedef struct
{
    const char *switchName;   // Name or description of the switch
    const RFC_sel *control;   // Pointer to an array of configurations
    size_t numConfigs;        // Number of configurations
    const char *datasheetURL; // URL to the datasheet
} SwitchEntity;

// SP8T switch SKY13418 Control Logic (6GHz)
static const RFC_sel sp8_SKY13418_Controls[] = {
    // V1, V2, V3
    { 0, 0, 0 }, // RF1
    { 0, 0, 1 }, // RF2
    { 0, 1, 0 }, // RF3
    { 0, 1, 1 }, // RF4
    { 1, 0, 0 }, // RF5
    { 1, 0, 1 }, // RF6
    { 1, 1, 0 }, // RF7
    { 1, 1, 1 }  // RF8
};

static const SwitchEntity SP8T_SKY13418 = {
    "SKY13418 6GHz SP8T Switch", //
    sp8_SKY13418_Controls,       //
    sizeof(sp8_SKY13418_Controls) / sizeof(sp8_SKY13418_Controls[0]),
    "https://www.skyworksinc.com/-/media/SkyWorks/Documents/Products/701-800/"
    "SKY13418_485LF_201712F.pdf"
};

// SP4T switch HMC7992  Control Logic (6GHz)
static const RFC_sel sp4_HMC7992_tControls[] = {
    // A, B, nc
    { 0, 0, 0 }, // RF1
    { 1, 0, 0 }, // RF2
    { 0, 1, 0 }, // RF3
    { 1, 1, 0 }  // RF4
};

static const SwitchEntity SP4T_HMC7992 = {
    "HMC7992 6GHz SP4T Switch", sp4_HMC7992_tControls,
    sizeof(sp4_HMC7992_tControls) / sizeof(sp4_HMC7992_tControls[0]),
    "https://www.analog.com/media/en/technical-documentation/data-sheets/"
    "HMC7992.pdf"
};

// SP8T switch HMC253 Control Logic (3GHz)
static const RFC_sel sp8_HMC253_Controls[] = {
    // A, B, C
    { 0, 0, 0 }, // RF1
    { 1, 0, 0 }, // RF2
    { 0, 1, 0 }, // RF3
    { 1, 1, 0 }, // RF4
    { 0, 0, 1 }, // RF5
    { 1, 0, 1 }, // RF6
    { 0, 1, 1 }, // RF7
    { 1, 1, 1 }  // RF8
};

static const SwitchEntity SP8T_HMC253 = {
    "HMC253 3GHz SP8T Switch", sp8_HMC253_Controls,
    sizeof(sp8_HMC253_Controls) / sizeof(sp8_HMC253_Controls[0]),
    "https://www.analog.com/media/en/technical-documentation/data-sheets/"
    "hmc253qs24.pdf"
};

// SPDT switch HMC349ALP4CE Control Logic (DC-3.5 GHz)
static const RFC_sel spdt_HMC349ALP4CE_Controls[] = {
    // EN, CTRL, nc
    { 0, 0, 0 }, // RF1 (0V on control pin)
    { 0, 1, 0 }  // RF2 (+ voltage on control pin)
};

static const SwitchEntity SPDT_HMC349ALP4CE = {
    "HMC349ALP4CE DC-3.5 GHz SPDT Switch", spdt_HMC349ALP4CE_Controls,
    sizeof(spdt_HMC349ALP4CE_Controls) / sizeof(spdt_HMC349ALP4CE_Controls[0]),
    "https://www.analog.com/media/en/technical-documentation/data-sheets/"
    "hmc349alp4ce.pdf"
};

///////////////

// Array of pointers to Switch structures
static const SwitchEntity *switchArray[] = { &SP8T_SKY13418, &SP4T_HMC7992, &SP8T_HMC253,
                                             &SPDT_HMC349ALP4CE };

// Error handling function
void handleError(const char *message);

// Sets GPIO pins based on the RFC_sel settings
void setPins(const RFC_sel *ctrl);

// Retrieves a switch entity by index
const SwitchEntity *getSwitchEntityByIndex(int index);

// Retrieves the number of configurations for a switch by index
const int getSwitchNumConfigsByIndex(int index);

// Retrieves the datasheet URL for a switch by index
const char *getSwitchDatasheetURLByIndex(int index);

// Retrieves the name of a switch by index
const char *getSwitchNameByIndex(int index);

// Finds a switch by its name
const SwitchEntity *getSwitchByName(const char *name);

// Retrieves RFC selection safely
const RFC_sel *getAntSelector(const SwitchEntity *switchInfo, int position);

// Sets the antenna switch to a given position using the specified switch
// configuration
int SetAntennaSwitch(const SwitchEntity *switchInfo, int position);

#endif // SWITCHES_H_