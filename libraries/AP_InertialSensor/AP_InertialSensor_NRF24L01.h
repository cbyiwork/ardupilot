#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include <StorageManager/StorageManager.h>


typedef struct Rudder_Para{

	uint8_t isLeft;
	uint8_t powLeft;
	uint8_t isUp;
	uint8_t powUp;
	uint8_t isReverse;
	uint8_t oprMode;
	uint8_t ch5set;
	uint8_t ch6set;
	int8_t ch1_mft;
	int8_t ch2_mft;
	int8_t isResponse;
	uint8_t dummy;

}RudderPara;

typedef enum ModeType
{
	MODE_TX = 0,
	MODE_RX
}nRf24l01ModeType;


typedef enum {
	CHAN_NULL = 0,
	CHAN_1,
	CHAN_2,
	CHAN_3,
	CHAN_4,
	CHAN_5,
	CHAN_6,
	CHAN_7,
	CHAN_8,
}RF_CHANNEL;

typedef enum OperationType{
	OPERA_M = 0,
	OPERA_H,
	OPERA_L

}OperaType;


class AP_InertialSensor_NRF24L01 : public AP_InertialSensor_Backend
{
public:
    virtual ~AP_InertialSensor_NRF24L01() { }
    void start(void) override;
    bool update() override;

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev_accel);

	static AP_InertialSensor_NRF24L01 *getInstance();
	bool get_nrf24l01_input(float &throttle, float &steering);
	bool get_nrf24l01_mode(OperaType &mode);

											
private:

	AP_InertialSensor_NRF24L01(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev_accel);

	static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);
	static void setInstance(AP_InertialSensor_NRF24L01 * inst);

	uint8_t _register_read(uint8_t reg);
    void _register_write(uint8_t reg, uint8_t val, bool checked=false);

	void _set_tx_addr(uint8_t *pAddr, uint8_t len);
	void _set_rx_addr(uint8_t pipeNum, uint8_t* pAddr, uint8_t len);
	void _write_buf(uint8_t RegAddr, uint8_t *pBuf, uint8_t len);
	void _read_buf( uint8_t RegAddr, uint8_t *pBuf, uint8_t len );
	bool do_set_servo(uint8_t _channel, uint16_t pwm);
	void ImpChanDuty();

	AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
	AP_HAL::OwnPtr<AP_HAL::Device> _dev_acc;
    AP_HAL::Semaphore *_spi_sem;
    AP_HAL::DigitalSource * _drdy_pin_xg;
	static AP_InertialSensor_NRF24L01 *mInstance;

	bool _init_sensor();
	void _accumulate();
	void _set_mode(nRf24l01ModeType Mode);

	/* update accel and gyro state */

	enum Rotation _rotation;

	uint8_t _read_status_register();
	uint8_t clear_IRQ_Flag(uint8_t IRQ_Source);
	uint8_t rx_packet( uint8_t *rx_buf);
	uint8_t rcvDataHandle(uint8_t *buf);
	uint8_t BuildRspPkt(uint8_t *buf);
	void responseProc();
	uint8_t txPacket(uint8_t *txbuf, uint8_t Length);
	void _cmd_write(uint8_t cmd);
	RudderPara _rudderPara;
	bool need_handle;
	uint32_t pwr_voltage[2];
	void save_rf_id(const void *ptr, uint16_t ofs, uint8_t size);
	void read_rf_id(void *ptr, uint16_t ofs, uint8_t size);
	bool _check_id(uint8_t* id);
	void EnginesDutyCalc();
	
	static StorageAccess        _storage;
	uint8_t _uniqueId[5];
	uint16_t engDuty[8];
	int16_t _rc_throttle;
	int16_t _rc_steeing;
	// gyro and accel instances
    //uint8_t _gyro_instance;
    //uint8_t _accel_instance;

	static void printf(const char *fmt, ...);
	static void dprintBuf(const char* buf, uint8_t len, char* name);
	uint8_t pairProc();
};


/** NRF24L01定义 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//寄存器操作命�?
#define NRF_READ_REG    0x00	//读配置寄存器，低5位为寄存器地�?
#define NRF_WRITE_REG   0x20	//写配置寄存器，低5位为寄存器地�?
#define RD_RX_PLOAD     0x61	//读RX有效数据�?1~32字节
#define WR_TX_PLOAD     0xA0	//写TX有效数据�?1~32字节
#define FLUSH_TX        0xE1	//清除TX FIFO寄存器，发射模式下使�?
#define FLUSH_RX        0xE2	//清除RX FIFO寄存器，接收模式下使�?
#define REUSE_TX_PL     0xE3	//重新使用上一包数据，CE为高，数据包被不断发�?
#define R_RX_PL_WID     0x60
#define NOP             0xFF	//空操作，可以用来读状态寄存器
#define W_ACK_PLOAD		0xA8
#define WR_TX_PLOAD_NACK 0xB0
//SPI(NRF24L01)寄存器地�?
#define CONFIG          0x00	//配置寄存器地�?，bit0:1接收模式,0发射模式;bit1:电�?�择;bit2:CRC模式;bit3:CRC使能;
							    //bit4:中断MAX_RT(达到�?大重发次数中�?)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能	
#define EN_AA           0x01	//使能自动应答功能 bit0~5 对应通道0~5
#define EN_RXADDR       0x02	//接收地址允许 bit0~5 对应通道0~5
#define SETUP_AW        0x03	//设置地址宽度(�?有数据�?�道) bit0~1: 00,3字节 01,4字节, 02,5字节
#define SETUP_RETR      0x04	//建立自动重发;bit0~3:自动重发计数�?;bit4~7:自动重发延时 250*x+86us
#define RF_CH           0x05	//RF通道,bit0~6工作通道频率
#define RF_SETUP        0x06	//RF寄存器，bit3:传输速率( 0:1M 1:2M);bit1~2:发射功率;bit0:噪声放大器增�?
#define STATUS          0x07	//状�?�寄存器;bit0:TX FIFO满标�?;bit1~3:接收数据通道�?(�?�?:6);bit4:达到�?多次重发次数
								//bit5:数据发�?�完成中�?;bit6:接收数据中断
#define MAX_TX  		0x10	//达到�?大发送次数中�?
#define TX_OK   		0x20	//TX发�?�完成中�?
#define RX_OK   		0x40	//接收到数据中�?

#define OBSERVE_TX      0x08	//发�?�检测寄存器,bit7~4:数据包丢失计数器;bit3~0:重发计数�?
#define CD              0x09	//载波�?测寄存器,bit0:载波�?�?
#define RX_ADDR_P0      0x0A	//数据通道0接收地址，最大长�?5个字节，低字节在�?
#define RX_ADDR_P1      0x0B	//数据通道1接收地址，最大长�?5个字节，低字节在�?
#define RX_ADDR_P2      0x0C	//数据通道2接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define RX_ADDR_P3      0x0D	//数据通道3接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define RX_ADDR_P4      0x0E	//数据通道4接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define RX_ADDR_P5      0x0F	//数据通道5接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define TX_ADDR         0x10	//发�?�地�?(低字节在�?),ShockBurstTM模式下，RX_ADDR_P0与地�?相等
#define RX_PW_P0        0x11	//接收数据通道0有效数据宽度(1~32字节),设置�?0则非�?
#define RX_PW_P1        0x12	//接收数据通道1有效数据宽度(1~32字节),设置�?0则非�?
#define RX_PW_P2        0x13	//接收数据通道2有效数据宽度(1~32字节),设置�?0则非�?
#define RX_PW_P3        0x14	//接收数据通道3有效数据宽度(1~32字节),设置�?0则非�?
#define RX_PW_P4        0x15	//接收数据通道4有效数据宽度(1~32字节),设置�?0则非�?
#define RX_PW_P5        0x16	//接收数据通道5有效数据宽度(1~32字节),设置�?0则非�?
#define NRF_FIFO_STATUS 0x17	//FIFO状�?�寄存器;bit0:RX FIFO寄存器空标志;bit1:RX FIFO满标�?;bit2~3保留
								//bit4:TX FIFO 空标�?;bit5:TX FIFO满标�?;bit6:1,循环发�?�上�?数据�?.0,不循�?								
#define DYNPD			0x1C
#define FEATRUE			0x1D

#define CONT_WAVE     	7 
#define RF_DR_LOW     	5 
#define PLL_LOCK      	4 
#define RF_DR_HIGH    	3 
//bit2-bit1:
#define PWR_18DB  		(0x00<<1)
#define PWR_12DB  		(0x01<<1)
#define PWR_6DB   		(0x02<<1)
#define PWR_0DB   		(0x03<<1)

#define RX_DR         	6 
#define TX_DS         	5 
#define MAX_RT        	4 
//for bit3-bit1, 
#define TX_FULL_0     	0 

#define RPD           	0 

#define TX_REUSE      	6 
#define TX_FULL_1     	5 
#define TX_EMPTY      	4 
//bit3-bit2, reserved, only '00'
#define RX_FULL       	1 
#define RX_EMPTY      	0 

#define DPL_P5        	5 
#define DPL_P4        	4 
#define DPL_P3        	3 
#define DPL_P2        	2 
#define DPL_P1        	1 
#define DPL_P0        	0 

#define EN_DPL        	2 
#define EN_ACK_PAY    	1 
#define EN_DYN_ACK    	0 
#define IRQ_ALL  ( (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT) )

#define REPEAT_CNT          0//15		//重复次数

//位定�?
#define MASK_RX_DR   	6 
#define MASK_TX_DS   	5 
#define MASK_MAX_RT  	4 
#define EN_CRC       	3 
#define CRCO         	2 
#define PWR_UP       	1 
#define PRIM_RX      	0 

#define ENAA_P5      	5 
#define ENAA_P4      	4 
#define ENAA_P3      	3 
#define ENAA_P2      	2 
#define ENAA_P1      	1 
#define ENAA_P0      	0 

#define ERX_P5       	5 
#define ERX_P4       	4 
#define ERX_P3       	3 
#define ERX_P2      	2 
#define ERX_P1       	1 
#define ERX_P0       	0 

#define AW_RERSERVED 	0x0 
#define AW_3BYTES    	0x1
#define AW_4BYTES    	0x2
#define AW_5BYTES    	0x3

#define ARD_250US    	(0x00<<4)
#define ARD_500US    	(0x01<<4)
#define ARD_750US    	(0x02<<4)
#define ARD_1000US   	(0x03<<4)
#define ARD_2000US   	(0x07<<4)
#define ARD_4000US   	(0x0F<<4)
#define ARC_DISABLE   	0x00
#define ARC_15        	0x0F

#define PAIR_ADDR			0x67,0x75,0x6f,0x62,0x6f
#define PAIR_REQ_HEAD "req:gb"
#define UNIQUE_ID_HEAD "candy"
#define PAIR_OK_HEAD "pair ok"


