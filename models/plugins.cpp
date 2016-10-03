//register the two plugin
#include <driver/sensors/models/LucidControl/LucidTemperatureDriver.h>
#include <driver/sensors/models/Simulator/VTemperatureDriver.h>
OPEN_REGISTER_PLUGIN
REGISTER_PLUGIN(LucidTemperatureDriver)
REGISTER_PLUGIN(LucidOuputDriver)
REGISTER_PLUGIN(VTemperatureDriver)
CLOSE_REGISTER_PLUGIN

