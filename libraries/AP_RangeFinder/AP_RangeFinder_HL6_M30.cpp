/*************************************************************************************************************
*头  文件：
*函数功能：任务组
*修改日期：2019-1-3
*备   注：
***************************************************************************************************************/

#include "AP_RangeFinder_HL6_M30.h"

#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/AP_HAL.h>
#include <ctype.h>

extern const AP_HAL::HAL &hal;

AP_RangeFinder_HL6_M30::AP_RangeFinder_HL6_M30(RangeFinder::RangeFinder_State &_state,
		                                       AP_SerialManager &serial_manager,
                                               uint8_t serial_instance):
AP_RangeFinder_Backend( _state)
{

    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_HL6_M30, serial_instance);
    if (uart != nullptr)
    {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_HL6_M30, serial_instance));
    }
    hl6_m30_parse_status = HL6_M30_IDLE;


}

/*************************************************************************************************************************************
*函数原型：bool AP_RangeFinder_HL6_30M::detect(RangeFinder &_ranger, uint8_t instance, AP_SerialManager &serial_manager)
*函数功能：识别传感器
*修改日期：2019-1-3
*备   注：
**********************************************************************************************************************************/
bool AP_RangeFinder_HL6_M30::detect(AP_SerialManager &serial_manager,uint8_t serial_instance)
{
	return serial_manager.find_serial(AP_SerialManager::SerialProtocol_HL6_M30, serial_instance) != nullptr;
}


/*********************************************************************************************************************************
*函数原型：bool AP_RangeFinder_HL6_30M::get_reading(uint16_t &reading_cm)
*函数功能：获取高度数据
*修改日期：2019-1-3
*备   注：read - return last value measured by sensor
**********************************************************************************************************************************/

bool AP_RangeFinder_HL6_M30::get_reading(uint16_t &reading_cm)
{
    uint32_t hl6_m30_sum_data=0;
    uint16_t hl6_m30_count=0;
    if (uart == nullptr)
    {
        return false;
    }

    int16_t nbytes  = uart->available();      //获取串口有效位
    hal.uartC->printf("nbytes=%d\r\n",nbytes);
    if(nbytes == 0)
    {
    	 return false;
    }

    while(nbytes-- >0)
    {
    	 uint8_t c = uart->read();
         switch(hl6_m30_parse_status)
         {
            case HL6_M30_IDLE:

            	if(c == HL6_M30_SYS_HEAP1) //0xEB=235
            	{
            		hl6_m30_parse_status = HL6_M30_GOT_START1;
            	}
            	break;
            case HL6_M30_GOT_START1://0x90=144
            	if(c == HL6_M30_SYS_HEAP2)
            	{
            		hl6_m30_parse_status = HL6_M30_GOT_START2;
            	}
            	else
            	{
            		hl6_m30_parse_status = HL6_M30_IDLE;
            	}
            	break;
            case HL6_M30_GOT_START2:

            	if(c == HL6_M30_SYS_STATUS1)                     //状态位正常测高0x01H
            	{
            		hl6_m30_buf[0]=0x01;

            		hl6_m30_parse_status = HL6_M30_GOT_STATUS;  //获取正常状态位高度状态

            	}
            	else if(c == HL6_M30_SYS_STATUS2)               //状态位无效测高0x00H
            	{

            		hl6_m30_buf[0]=0x00;
            		hl6_m30_parse_status = HL6_M30_IDLE;
            	}
            	else
            	{
            		hl6_m30_parse_status = HL6_M30_IDLE;
            	}
            	break;

            case HL6_M30_GOT_STATUS:	  //开始读取数据第1字节
            	hl6_m30_buf[1]=c;
            	hl6_m30_parse_status = HL6_M30_GOT_DATA1;
            	break;

            case HL6_M30_GOT_DATA1:	    //开始读取数据第2字节

                hl6_m30_buf[2]=c;
                hl6_m30_parse_status = HL6_M30_GOT_DATA2;

                break;
            case HL6_M30_GOT_DATA2:	  //读取校验位

            	hl6_m30_crc_data=c;
                hl6_m30_parse_status = HL6_M30_GOT_CRC;

                break;
            case HL6_M30_GOT_CRC:	  //有效开始读取数据

            	 hl6_m30_buf[3]=0xEB;
            	 hl6_m30_buf[4]=0x90;

            	if(hl6_m30_CRC16(hl6_m30_buf,HL6_M30_CRC_LENGTH,hl6_m30_crc_data))
            	{
            		 hl6_m30_data=(uint16_t)(hl6_m30_buf[2]*0x100+hl6_m30_buf[1]);  //得到的数据高位左移8位与低8位按位进行或运算
            		 hl6_m30_parse_status = HL6_M30_GOT_END;
            	}
            	else
				{

            		hl6_m30_parse_status = HL6_M30_IDLE;
				}

                break;
            case HL6_M30_GOT_END:	  //有效开始读取数据

            	hl6_m30_parse_status = HL6_M30_IDLE;  //这次计算完成，下次重新计算
            	hl6_m30_count++;
            	hl6_m30_sum_data+=hl6_m30_data;
         }
    }
    if (hl6_m30_count == 0)
    {
        return false;
    }
    else
    {
     hal.uartC->printf("hl6_m30_buf[1]=%d\r\n",hl6_m30_buf[1]);
     hal.uartC->printf("hl6_m30_buf[2]=%d\r\n",hl6_m30_buf[2]);
     hal.uartC->printf("hl6_m30_data=%d\r\n",hl6_m30_data);
     hal.uartC->printf("hl6_m30_count=%d\r\n",hl6_m30_count);
     hal.uartC->printf("hl6_m30_sum_data=%d\r\n",hl6_m30_sum_data);
   	 reading_cm= (uint16_t)(hl6_m30_sum_data/hl6_m30_count);
   	 reading_cm=reading_cm*10; //单位是cm

   	 hal.uartC->printf("reading_cm=%d\r\n",reading_cm);
   	 return true;
    }


}

/*********************************************************************************************************************************
*函数原型：bool AP_RangeFinder_HL6_30M::get_reading(uint16_t &reading_cm)
*函数功能：获取高度数据
*修改日期：2019-1-3
*备   注：read - return last value measured by sensor
**********************************************************************************************************************************/

void AP_RangeFinder_HL6_M30::update(void)
{
	hal.uartC->printf("HL6_M30 update \r\n");
    if (get_reading(state.distance_cm))
    {
    	hal.uartC->printf("HL6_M30 Read complete  \r\n");
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200)
    {
    	hal.uartC->printf("HL6_M30 Read fail  \r\n");
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

/*********************************************************************************************************************************
*函数原型：bool AP_RangeFinder_HL6_M30::hl6_m30_CRC16(uint8_t *aBuffer, uint8_t aLength)
*函数功能：计算校验和
*修改日期：2019-1-4
*备   注：read - return last value measured by sensor
**********************************************************************************************************************************/
bool AP_RangeFinder_HL6_M30::hl6_m30_CRC16(uint8_t *aBuffer, uint8_t aLength,uint8_t check_data)
{
	 uint16_t crc = 0X0000;
	 for (uint8_t i=0; i<aLength; i++)
	 {
		 crc+=aBuffer[i];
	 }
	 hal.uartC->printf("crc=%d\r\n",crc);
	 crc_data=(uint8_t)(~crc)+1;
	 hal.uartC->printf("check_data=%d\r\n",check_data);
	 hal.uartC->printf("crc_data=%d\r\n",crc_data);

	 if(crc_data==check_data) //查看最低位是否是1，是1说明数据无效，因为没有数据是0，取反得到1，
	 {
		 return true;
	 }
	 else
	 {
		 return false;
	 }


}






