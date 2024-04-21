#include "switches.h"
#include "main.h"

extern const SwitchEntity *switchArray[];

// Error handling function
void handleError(const char *message)
{
    // printf("%s\n", message);
}

void setPins(const RFC_sel *ctrl)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, ctrl->V1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, ctrl->V2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, ctrl->V3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Utility function to check index validity
const SwitchEntity *getSwitchEntityByIndex(int index)
{
    if (index < 0 || index >= sizeof(switchArray) / sizeof(switchArray[0]))
    {
        handleError("Error: Invalid SwitchEntity index");
        return NULL;
    }

    return switchArray[index];
}

const RFC_sel *getAntSelector(const SwitchEntity *switchInfo, int position)
{
    if (!switchInfo || (position < 0) || (position >= switchInfo->numConfigs))
    {
        handleError("Error: Invalid selector position or switch info.");
        return NULL;
    }

    return &switchInfo->control[position];
}

const int getSwitchNumConfigsByIndex(int index)
{
    const SwitchEntity *sw = getSwitchEntityByIndex(index);
    if (sw == NULL)
        return 0;

    return sw->numConfigs;
}

const char *getSwitchDatasheetURLByIndex(int index)
{
    const SwitchEntity *sw = getSwitchEntityByIndex(index);
    if (sw == NULL)
        return "Invalid index";

    return sw->datasheetURL;
}

const char *getSwitchNameByIndex(int index)
{
    const SwitchEntity *sw = getSwitchEntityByIndex(index);
    if (!sw)
        return "Invalid index";

    return sw->switchName;
}

const SwitchEntity *getSwitchByName(const char *name)
{
    size_t numSwitches = sizeof(switchArray) / sizeof(switchArray[0]);
    for (size_t i = 0; i < numSwitches; i++)
    {
        if (strcmp(switchArray[i]->switchName, name) == 0)
            return switchArray[i];
    }

    return NULL; // Return NULL if no switch with the specified name is found
}

int SetAntennaSwitch(const SwitchEntity *switchInfo, int position)
{
    const RFC_sel *ctrl = getAntSelector(switchInfo, position);
    if (!ctrl)
        return -1;

    setPins(ctrl);
    return 0;
}
