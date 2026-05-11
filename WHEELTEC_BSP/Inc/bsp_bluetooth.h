#ifndef __BSP_BLUETOOTH_H
#define __BSP_BLUETOOTH_H

#include "sys.h"

typedef struct{
	uint8_t dirkey; //????????
	uint8_t page;   //APP?????
	uint8_t saveflash;//????flash???
	uint8_t reportparam;//??????????????
}WHEELTEC_APPKey_t;

extern WHEELTEC_APPKey_t wheeltecApp;

void uart2_init(u32 bound);
void BlueToothAPPDecode(uint8_t recv);

#endif /* __BSP_BLUETOOTH_H */

