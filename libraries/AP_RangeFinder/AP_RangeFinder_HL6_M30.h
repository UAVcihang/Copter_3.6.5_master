/*
 * AP_RangeFinder_HL6_M30.h
 *
 *  Created on: Jan 3, 2019
 *      Author: coco
 */

#ifndef ARDUPILOT_LIBRARIES_AP_RANGEFINDER_AP_RANGEFINDER_HL6_M30_H_
#define ARDUPILOT_LIBRARIES_AP_RANGEFINDER_AP_RANGEFINDER_HL6_M30_H_
#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

#define HL6_M30_SYS_HEAP    0x90EB
#define HL6_M30_SYS_HEAP1    0xEB
#define HL6_M30_SYS_HEAP2    0x90
#define HL6_M30_SYS_STATUS1 0x01
#define HL6_M30_SYS_STATUS2 0x00
#define HL6_M30_CRC_LENGTH  5

typedef enum HL6_M30_Status {
	HL6_M30_GOT_NO_STATUS=-1,
	HL6_M30_IDLE     = 0,
	HL6_M30_GOT_START1 = 1, //同步头1
	HL6_M30_GOT_START2 = 2, //同步头2
	HL6_M30_GOT_STATUS = 3, //状态位
	HL6_M30_GOT_DATA1=4,   //状态位
	HL6_M30_GOT_DATA2=5,
	HL6_M30_GOT_CRC,
	HL6_M30_GOT_END,


}HL6_M30_parse_status;


class AP_RangeFinder_HL6_M30 : public AP_RangeFinder_Backend
{
public:

	//构造函数
	AP_RangeFinder_HL6_M30(RangeFinder::RangeFinder_State &_state,
			               AP_SerialManager &serial_manager,
			               uint8_t serial_instance);


    // 芯片识别函数
	 static bool detect(AP_SerialManager &serial_manager, uint8_t serial_instance);

    // 数据更新函数
    void update(void);

    bool hl6_m30_CRC16(uint8_t *aBuffer, uint8_t aLength,uint8_t check_data);
protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override
	{
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }
private:
      bool get_reading(uint16_t &reading_cm);


      HL6_M30_parse_status hl6_m30_parse_status;
      AP_HAL::UARTDriver *uart = nullptr;
      uint32_t last_reading_ms = 0;
      uint8_t hl6_m30_buf[5];
      uint16_t hl6_m30_data;

      uint8_t hl6_m30_crc_data;
      uint16_t crc_data;
};




#endif /* ARDUPILOT_LIBRARIES_AP_RANGEFINDER_AP_RANGEFINDER_HL6_M30_H_ */
