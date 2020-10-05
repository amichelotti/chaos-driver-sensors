/*
 * Copyright 2020, 2017 INFN
 * Andrea Michelotti
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
/*
  This CU use the property CU to wrap STATE and produce errors

*/
#ifndef __BasicSensorProperty_H__
#define __BasicSensorProperty_H__
#include <chaos/cu_toolkit/control_manager/DriverPropertyCU.h>

class BasicSensorProperty : public chaos::cu::control_manager::DriverPropertyCU {
	PUBLISHABLE_CONTROL_UNIT_INTERFACE(BasicSensorProperty);
public:
    /*!
     Construct a new CU with full constructor
     */
    BasicSensorProperty(const std::string& _control_unit_id, const std::string& _control_unit_param, const ControlUnitDriverList& _control_unit_drivers);
    /*!
     Destructor a new CU
     */
    ~BasicSensorProperty();

protected:
		/*!
		Define the Control Unit Dataset and Actions
		*/
		void unitDefineActionAndDataset();
		/*!(Optional)
		Define the Control Unit custom attribute
		*/
		void unitDefineCustomAttribute();
    /*!(Optional)
     Initialize the Control Unit and all driver, with received param from MetadataServer
     */
		void unitInit();
    /*!(Optional)
     Execute the work, this is called with a determinated delay
     */
    void unitStart();
    /*!
     Execute the work, this is called with a determinated delay, it must be as fast as possible
     */
    void unitRun();

    /*!(Optional)
     The Control Unit will be stopped
     */
    void unitStop();

    /*!(Optional)
     The Control Unit will be deinitialized and disposed
     */
    void unitDeinit();


};

#endif
