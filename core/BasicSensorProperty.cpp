/*
 * Copyright 2012, 2017 INFN
 *
 * Licensed under the EUPL, Version 1.2 or – as soon they
 * will be approved by the European Commission - subsequent
 * versions of the EUPL (the "Licence");
 * You may not use this work except in compliance with the
 * Licence.
 * You may obtain a copy of the Licence at:
 *
 * https://joinup.ec.europa.eu/software/page/eupl
 *
 * Unless required by applicable law or agreed to in
 * writing, software distributed under the Licence is
 * distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied.
 * See the Licence for the specific language governing
 * permissions and limitations under the Licence.
 */

#include "BasicSensorProperty.h"
#include <chaos/cu_toolkit/control_manager/ControlUnitTypes.h>
using namespace chaos;
using namespace chaos::common::data::cache;
using namespace chaos::cu::driver_manager::driver;

PUBLISHABLE_CONTROL_UNIT_IMPLEMENTATION(BasicSensorProperty)

#define BasicSensorPropertyLAPP_ LAPP_ << "[BasicSensorProperty] "
#define BasicSensorPropertyLDBG_                                               \
  LDBG_ << "[BasicSensorProperty] " << __PRETTY_FUNCTION__ << " "
#define BasicSensorPropertyLERR_                                               \
  LERR_ << "[BasicSensorProperty] " << __PRETTY_FUNCTION__ << "(" << __LINE__  \
        << ") "

/*
 Construct
 */
BasicSensorProperty::BasicSensorProperty(
    const string &_control_unit_id, const string &_control_unit_param,
    const ControlUnitDriverList &_control_unit_drivers)
    : chaos::cu::control_manager::DriverPropertyCU(
          _control_unit_id, _control_unit_param, _control_unit_drivers) {}

/*
 Destructor
 */
BasicSensorProperty::~BasicSensorProperty() {}

  //! Return the definition of the control unit
  /*!
   The api that can be called withi this method are listed into
   "Control Unit Definition Public API" module into html documentation
   (chaosframework/Documentation/html/group___control___unit___definition___api.html)
   */

#include "AbstractSensorDriver.h"
using namespace chaos::cu::control_manager;
void BasicSensorProperty::unitDefineActionAndDataset() {
  DriverPropertyCU::unitDefineActionAndDataset();

  addStateVariable( chaos::cu::control_manager::StateVariableTypeAlarmDEV, "underflow",
                   "the sensor is detecting an underflow");

  addStateVariable( chaos::cu::control_manager::StateVariableTypeAlarmDEV, "overflow",
                   "the sensor is detecting an overflow");

  addStateVariable( chaos::cu::control_manager::StateVariableTypeAlarmDEV, "error",
                   "the sensor is detecting an error");

  addStateVariable( chaos::cu::control_manager::StateVariableTypeAlarmDEV, "off", "the sensor is OFF");
  addStateVariable( chaos::cu::control_manager::StateVariableTypeAlarmDEV, "nosensor",
                   "the sensor is not present or broken");

  addStateVariable( chaos::cu::control_manager::StateVariableTypeAlarmDEV, "iderror",
                   "cannot identify sensor ID");
  addStateVariable( chaos::cu::control_manager::StateVariableTypeAlarmDEV, "communication_error",
                   "serial communication error");
}

//! Define custom control unit attribute
void BasicSensorProperty::unitDefineCustomAttribute() {}

//! Initialize the Custom Control Unit
void BasicSensorProperty::unitInit() {
  // driver->init();
}

//! Execute the work, this is called with a determinated delay, it must be as
//! fast as possible
void BasicSensorProperty::unitStart() {}

//! Execute the Control Unit work
void BasicSensorProperty::unitRun() {
  // get the output attribute pointer form the internal cache
  chaos::cu::control_manager::DriverPropertyCU::unitRun();

  int32_t state =
      *getAttributeCache()->getROPtr<int32_t>(DOMAIN_OUTPUT, "STATE");
  if (state == 0) {
    setStateVariableSeverity(
         chaos::cu::control_manager::StateVariableTypeAlarmDEV, "underflow",
        chaos::common::alarm::MultiSeverityAlarmLevelClear);
    setStateVariableSeverity(
         chaos::cu::control_manager::StateVariableTypeAlarmDEV, "overflow",
        chaos::common::alarm::MultiSeverityAlarmLevelClear);
    setStateVariableSeverity(
         chaos::cu::control_manager::StateVariableTypeAlarmDEV, "error",
        chaos::common::alarm::MultiSeverityAlarmLevelClear);
    setStateVariableSeverity(
         chaos::cu::control_manager::StateVariableTypeAlarmDEV, "nosensor",
        chaos::common::alarm::MultiSeverityAlarmLevelClear);
    setStateVariableSeverity(
         chaos::cu::control_manager::StateVariableTypeAlarmDEV, "off",
        chaos::common::alarm::MultiSeverityAlarmLevelClear);
    setStateVariableSeverity(
         chaos::cu::control_manager::StateVariableTypeAlarmDEV, "iderror",
        chaos::common::alarm::MultiSeverityAlarmLevelClear);
    setStateVariableSeverity(
         chaos::cu::control_manager::StateVariableTypeAlarmDEV, "communication_error",
        chaos::common::alarm::MultiSeverityAlarmLevelClear);
        
  }
  if (state & SENSOR_STATE_UNDERFLOW) {
    setStateVariableSeverity(
         chaos::cu::control_manager::StateVariableTypeAlarmDEV, "underflow",
        chaos::common::alarm::MultiSeverityAlarmLevelWarning);
  }

  if (state & SENSOR_STATE_OVERFLOW) {
    setStateVariableSeverity(
         chaos::cu::control_manager::StateVariableTypeAlarmDEV, "overflow",
        chaos::common::alarm::MultiSeverityAlarmLevelWarning);
  }
  if (state & SENSOR_STATE_ERROR) {
    setStateVariableSeverity( chaos::cu::control_manager::StateVariableTypeAlarmDEV, "error",
                             chaos::common::alarm::MultiSeverityAlarmLevelHigh);
  }

  if (state & SENSOR_STATE_NOSENSOR) {
    setStateVariableSeverity( chaos::cu::control_manager::StateVariableTypeAlarmDEV, "nosensor",
                             chaos::common::alarm::MultiSeverityAlarmLevelHigh);
  }

  if (state & SENSOR_STATE_OFF) {
    setStateVariableSeverity( chaos::cu::control_manager::StateVariableTypeAlarmDEV, "off",
                             chaos::common::alarm::MultiSeverityAlarmLevelHigh);
  }
  if (state & SENSOR_STATE_IDERROR) {
    setStateVariableSeverity( chaos::cu::control_manager::StateVariableTypeAlarmDEV, "iderror",
                             chaos::common::alarm::MultiSeverityAlarmLevelHigh);
  }
  if (state & SENSOR_STATE_CONN_ERROR) {
    setStateVariableSeverity( chaos::cu::control_manager::StateVariableTypeAlarmDEV, "communication_error",
                             chaos::common::alarm::MultiSeverityAlarmLevelHigh);
  }

  // getAttributeCache()->setOutputDomainAsChanged();
}

//! Execute the Control Unit work
void BasicSensorProperty::unitStop() {}

//! Deinit the Control Unit
void BasicSensorProperty::unitDeinit() {
  // driver->deinit();
}

/*
 CDataWrapper *BasicSensorProperty::my_custom_action(CDataWrapper *actionParam,
 bool& detachParam) { CDataWrapper *result =  new CDataWrapper(); return result;
 }
 */
