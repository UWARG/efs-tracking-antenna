#include <BMX160.h>

Chip_Status BMX160::_Get_ChipID() {
	for (int i = 0; i< 10; i++){
    	uint8_t chip_id = 0;
    		if (Mem_Read(CHIP_ID_ADDR, &chip_id)==HAL_OK){
    			return chip_id;//read CPU chip ID
    		}
    	HAL_Delay(100);
	}
	return E_NOT_OK;
}

Sensor_Status BMX160::Sensor_Available(uint8_t timeout_count) {
	if (timeout_count == NULL) {//var cast
		while(!(_Get_ChipID()==CHIP_ID)){}//exit inf loop is == default chip_id
		return E_OK;
	} else {
		for (uint8_t i = 0; i < timeout_count; i++) {
			if((_Get_ChipID()==CHIP_ID)){
				return E_OK;
			}
		}
	}
	return E_NOT_OK;
}

