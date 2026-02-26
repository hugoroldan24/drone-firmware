#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include "dron_structs.h"

void send_telemetry(Telem_t *telem_data);
void ProcessBatteryAndAltitude(Raw_Telem_t rx_data, Telem_t *telem_data);

#endif /* TELEMETRY_H_ */

