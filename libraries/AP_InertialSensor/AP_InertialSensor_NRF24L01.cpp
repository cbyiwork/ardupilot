/*
 *   created by ycb, for nrf24l01 data transfer
 *   2020/03/03
 */


#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Arming/AP_Arming.h>

#include "AP_InertialSensor_NRF24L01.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;
#define HAL_PIN_NRF24L01_CE PAL_LINE(GPIOA,8U)
#define HAL_PIN_NRF24L01_INT PAL_LINE(GPIOC,6U)
#define DATA_CHECK_HZ   50
#define RSP_HEAD "rsp:"
#define RSP_END  "end"
const uint8_t default_addr[5] = {0x31,0x16,0x20,0x04,0x57};

//AP_Int32 logger_bitmask;

#define RF_ID_OFFSET 0//(6*1024)
#define INIT_PULSE_WIDTH 1500


//#define RF_DATA_OFFSET 256

StorageAccess AP_InertialSensor_NRF24L01::_storage(StorageManager::StorageRfId);
AP_InertialSensor_NRF24L01 *AP_InertialSensor_NRF24L01::mInstance;


AP_InertialSensor_NRF24L01::AP_InertialSensor_NRF24L01(AP_InertialSensor &imu,
								 AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
								 AP_HAL::OwnPtr<AP_HAL::Device> dev_accel)

    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev_gyro))
    , _dev_acc(std::move(dev_accel))
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_NRF24L01::probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev_accel)
{

	// setdebug uart boatrate
	//hal.uartE->begin(115200);

	//mInstance = nullptr;

	//printf("%s nrf24l01 probe\n", __FUNCTION__);
	
	if (!dev_gyro) {
		//printf("nrf24l01 get spi device error\n");
        return nullptr;
    }
	AP_InertialSensor_NRF24L01 *sensor
        = new AP_InertialSensor_NRF24L01(imu, std::move(dev_gyro), std::move(dev_accel));

	if (!sensor || !sensor->_init_sensor()) {
		//printf("nrf24l01 get get sensor or init error\n");
        delete sensor;
        return nullptr;
    }

	// ��������ʵ��
	//mInstance = sensor;

	sensor->pairProc();
	sensor->setInstance(sensor);

	printf("probe ok\n");

	return sensor;

}


AP_InertialSensor_NRF24L01 *
AP_InertialSensor_NRF24L01::getInstance()
{
	return mInstance;
}

void
AP_InertialSensor_NRF24L01::setInstance(AP_InertialSensor_NRF24L01 * inst)
{
	mInstance = inst;
}


// write to EEPROM
void AP_InertialSensor_NRF24L01::save_rf_id(const void *ptr, uint16_t ofs, uint8_t size)
{
	_storage.write_block(ofs, ptr, size);
	//hal.storage->write_block(ofs, ptr, size);
}

// write to EEPROM
void AP_InertialSensor_NRF24L01::read_rf_id(void *ptr, uint16_t ofs, uint8_t size)
{
	_storage.read_block(ptr, ofs, size);
	//hal.storage->read_block(ptr, ofs, size);
}

bool AP_InertialSensor_NRF24L01::_check_id(uint8_t* id) 
{
	bool ret = false;
	uint8_t i = 0;

	//printf("id: ");
	for (i=0;i<5;i++) {
		//printf("%x ", id[i]);
		if ((id[i]!=0) && (id[i]!=0xff)) {
			ret = true;
		}
	}
	//printf("\n");
	return ret;

}

bool AP_InertialSensor_NRF24L01::_init_sensor(void)
{
	//uint8_t addr[5] = {0x34,0x43,0x10,0x10,0x01};
	uint8_t buf[5];
	/*uint32_t ep_test = 0;

	//memcpy(_uniqueId, addr, 5);

	ep_test = _storage.read_uint32(RF_DATA_OFFSET);
	printf("ep_test: 0x%x\n", ep_test);

	_storage.write_uint32(RF_DATA_OFFSET, ++ep_test);

	printf("after inc, ep_test: 0x%x\n", _storage.read_uint32(RF_DATA_OFFSET));*/

	//hal.storage->write_block(256, addr, 5);

	//hal.storage->read_block(buf, 256, 5);

	//dprintBuf((char*)buf,5,"hal.storage");
	
	read_rf_id(buf, RF_ID_OFFSET, 5);

	dprintBuf((char*)buf, 5, "id");

	if(_check_id(buf)==true) {
		memcpy(_uniqueId, buf, 5);
	} else {
		memcpy(_uniqueId, default_addr, 5);
		save_rf_id(_uniqueId,RF_ID_OFFSET, 5);
		read_rf_id(buf, RF_ID_OFFSET,5);
		dprintBuf((char*)buf,5,"rf id");
	}

	//printf("nrf24l01 _init_sensor enter\n");
	palSetLineMode(HAL_PIN_NRF24L01_CE, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPad(GPIOA, 8U);

	palSetLineMode(HAL_PIN_NRF24L01_INT, PAL_MODE_INPUT_PULLUP);
	
	if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
		//printf("nrf24l01 HAL_SEMAPHORE_BLOCK_FOREVER enter\n");
        return false;
    }

	_rudderPara.oprMode = OPERA_L;

	clear_IRQ_Flag( IRQ_ALL );
	
    _register_write( DYNPD, ( 1 << 0 )); 	//使能通道1动�?�数据长�?
    _register_write( FEATRUE, 0x07 );
    _register_read( DYNPD );
    _register_read( FEATRUE );


    _register_write( CONFIG, /*( 1<<MASK_RX_DR ) |*/		//接收中断
                                      ( 1 << EN_CRC ) |     //使能CRC 1个字�?
                                      ( 1 << PWR_UP ) );    //�?启设�?
    _register_write( EN_AA, ( 1 << ENAA_P0 ) );   		//通道0自动应答
    _register_write( EN_RXADDR, ( 1 << ERX_P0 ) );		//通道0接收
    _register_write( SETUP_AW, AW_5BYTES );     			//地址宽度 5个字�?

	// 此处不设置自动重发，统计包丢失率
    _register_write( SETUP_RETR, ARD_4000US |
                        ( REPEAT_CNT & 0x0F ) );         	//重复等待时间 250us
    _register_write( RF_CH, 60 );             			//初始化�?�道
    _register_write( RF_SETUP, 0x26 );

    //NRF24L01_Set_TxAddr( &addr[0], 5 );                      //设置TX地址
    //NRF24L01_Set_RxAddr( 0, &addr[0], 5 );                   //设置RX地址

	_set_tx_addr( _uniqueId, 5 );                      //设置TX地址
    _set_rx_addr( 0, _uniqueId, 5 );                   //设置RX地址

	_read_buf( TX_ADDR, buf, 5 );

	if (!memcmp(_uniqueId, buf, 5)) {
		//printf("nrf24l01 _init_sensor exit\n");
		
		_set_mode(MODE_RX);

		AP::arming().arm(AP_Arming::Method::MAVLINK, false);
	
		printf("verified write ram ok\n");

		_dev->get_semaphore()->give();
		return TRUE;
	}

	//printf("nrf24l01 verified write addr error\n");
	// 忘记释放信号量，导致无法进入定时回调函数，�?�误�?早上。_accumulate
	_dev->get_semaphore()->give();
	//return true;
	return false;
}

void AP_InertialSensor_NRF24L01::start()
{
	printf("nrf24l01 task start\n");

	//_gyro_instance = _imu.register_gyro(800, _dev->get_bus_id_devtype(DEVTYPE_NRF24L01));
    //_accel_instance = _imu.register_accel(800, _dev->get_bus_id_devtype(DEVTYPE_NRF24L01));
	
	_dev->register_periodic_callback(1000000UL/DATA_CHECK_HZ, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_NRF24L01::_accumulate, void));
	//hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_InertialSensor_NRF24L01::_accumulate()));
}

// get nrf24l01 data
void AP_InertialSensor_NRF24L01::_accumulate()
{
	// 
	uint8_t rf_buf[64];
	uint8_t ret = 0;
	/*
	static uint64_t sample_us = 0;

	//sprintf(rf_buf, "nrf24l01 sys: %d\n", AP_HAL::micros());
	AP_Logger *logger = AP_Logger::get_singleton();

	if (logger!=nullptr) {
		uint64_t now = AP_HAL::micros64();
        struct log_GYRO pkt = {
            LOG_PACKET_HEADER_INIT((uint8_t)(LOG_GYR1_MSG)),
            time_us   : now,
            sample_us : sample_us?sample_us:now,
            GyrX      : 11.2233,
            GyrY      : 56.8899,
            GyrZ      : 33.8844
        };
        logger->WriteBlock(&pkt, sizeof(pkt));
	
	} else {

		printf("get logger error\n");

	}
	vehicle_was_disarmed
	*/


	//printf("nrf24l01 _accumulate\n");
	if (need_handle == true) {
		if (rx_packet(rf_buf) > 0) {
			//printf("received data\n");
			ret = rcvDataHandle(rf_buf);
			if (ret > 0) {
				responseProc();
			}
		}
		need_handle = false;
	}
	
	
}

bool AP_InertialSensor_NRF24L01::update()
{
	//uint8_t rf_buf[64];

	//update_gyro(_gyro_instance);
    //update_accel(_accel_instance);
	/*
	if (rx_packet(rf_buf) > 0) {
		printf("received data\n");
		rcvDataHandle(rf_buf);
	}*/
	//palSetLineMode(LED_GREEN, PAL_MODE_OUTPUT_PUSHPULL);
	//palSetPad(GPIOA, 5);
	uint8_t status = palReadLine(HAL_PIN_NRF24L01_INT);

	if (status == 0) {
		need_handle = true;
	}
	
	return true;
}

uint8_t AP_InertialSensor_NRF24L01::_register_read(uint8_t reg)
{
    uint8_t val = 0;
    _dev->read_registers(reg, &val, 1);
    return val;
}

void AP_InertialSensor_NRF24L01::_register_write(uint8_t reg, uint8_t val, bool checked)
{
    _dev->write_register(NRF_WRITE_REG|reg, val, checked);
}

void AP_InertialSensor_NRF24L01::_cmd_write(uint8_t cmd)
{
	_dev->transfer(&cmd, 1, nullptr, 0);
}


void AP_InertialSensor_NRF24L01::_set_tx_addr(uint8_t *pAddr, uint8_t len)
{
	len = ( len > 5 ) ? 5 : len;					//地址不能大于5个字�?
    _write_buf( TX_ADDR, pAddr, len );	//写地�?
}

void AP_InertialSensor_NRF24L01::_set_rx_addr(uint8_t pipeNum, uint8_t* pAddr, uint8_t len)
{
	len = ( len > 5 ) ? 5 : len;
    pipeNum = ( pipeNum > 5 ) ? 5 : pipeNum;		//通道不大�?5 地址长度不大�?5个字�?

    _write_buf( RX_ADDR_P0 + pipeNum, pAddr, len );	//写入地址

}

void AP_InertialSensor_NRF24L01::_write_buf(uint8_t RegAddr, uint8_t *pBuf, uint8_t len)
{
	uint8_t i;

	uint8_t buf[64];
	
	buf[0] = RegAddr | NRF_WRITE_REG;
	for (i=0;i<len;i++) {
		buf[i+1] = pBuf[i];
	}

	_dev->transfer(buf, len+1, buf, 0);
	
}

void AP_InertialSensor_NRF24L01::_read_buf( uint8_t RegAddr, uint8_t *pBuf, uint8_t len )
{

	uint8_t reg =  NRF_READ_REG | RegAddr;

	_dev->transfer(&reg, 1, pBuf, len);
	
}


uint8_t AP_InertialSensor_NRF24L01::_read_status_register()
{
    uint8_t status;
	
    status = _register_read(NRF_READ_REG + STATUS);
	
    return status;
}


/**
  * @brief :NRF24L01清中�?
  * @param :
           @IRQ_Source:中断�?
  * @note  :�?
  * @retval:清除后状态寄存器的�??
  */
uint8_t AP_InertialSensor_NRF24L01::clear_IRQ_Flag( uint8_t IRQ_Source )
{
    uint8_t btmp = 0;

    IRQ_Source &= ( 1 << RX_DR ) | ( 1 << TX_DS ) | ( 1 << MAX_RT );	//中断标志处理
    btmp = _read_status_register();			//读状态寄存器

	_register_write( NRF_WRITE_REG + STATUS , IRQ_Source | btmp, FALSE);
	
    return _read_status_register();			//返回状�?�寄存器状�??
}

uint8_t AP_InertialSensor_NRF24L01::rx_packet( uint8_t *rx_buf)
{
	uint8_t status;
//	uint8_t i = 0;

	status = _register_read(STATUS);
	_register_write(STATUS,status);

	if (status & RX_OK) {
		uint8_t rcv_len = _register_read(R_RX_PL_WID);
		_read_buf(RD_RX_PLOAD, rx_buf, rcv_len);		
		_register_write(FLUSH_RX,0xff);
		return rcv_len;
	}

	return 0;
}

void AP_InertialSensor_NRF24L01::EnginesDutyCalc()
{
	uint16_t acc = 0;
	uint16_t ch1_mdt = 0;
	uint16_t ch2_mdt = 0;	

	// ��һ������ģʽ�£�����Ҫ����λ
	ch1_mdt = INIT_PULSE_WIDTH;
	ch2_mdt = INIT_PULSE_WIDTH;

	if (0==_rudderPara.isReverse) {
		if (0==_rudderPara.isUp) {
			acc = ch1_mdt + 5 * _rudderPara.powUp;
			if (acc > 2000) {
				engDuty[CHAN_1] = 2000;
			} else {
				engDuty[CHAN_1] = acc;
			}
		} else {

			acc = ch1_mdt - 5 * _rudderPara.powUp;
			if (acc < 1000) {
				engDuty[CHAN_1] = 1000;
			} else {
				engDuty[CHAN_1] = acc;
			}
		}

		if (0==_rudderPara.isLeft) {
			acc = ch2_mdt + 5*_rudderPara.powLeft;
			if (acc > 2000) {
				engDuty[CHAN_3] = 2000;
			} else {
				engDuty[CHAN_3] = acc;
			}
		} else {
			acc = ch2_mdt - 5 * _rudderPara.powLeft;
			if (acc < 1000) {
				engDuty[CHAN_3]  = 1000;
			} else {
				engDuty[CHAN_3] = acc;
			}
			
		}
		
	} else {

		if (0==_rudderPara.isUp) {
			
			acc = ch2_mdt + 5 * _rudderPara.powUp;
			if (acc > 2000) {
				//g_EngDuty[CHAN_2] = 2000;
				engDuty[CHAN_1] = 2000;
			} else {
				//g_EngDuty[CHAN_2] = acc;
				engDuty[CHAN_1] = acc;
			}
		} else {

			acc = ch2_mdt - 10 * _rudderPara.powUp;
			if (acc < 1000) {
				//g_EngDuty[CHAN_2] = 1000;
				engDuty[CHAN_1] = 1000;
			} else {
				//g_EngDuty[CHAN_2] = acc;
				engDuty[CHAN_1] = acc;
			}
		}

		if (0==_rudderPara.isLeft) {
			//acc = ch1_mdt + 5*g_rudderPara.powLeft;
			acc = ch1_mdt - 5 * _rudderPara.powLeft;
			//if (acc > 2000) {
			if (acc < 1000) {
				//g_EngDuty[CHAN_1] = 2000;
				engDuty[CHAN_3] = 1000;
			} else {
				//g_EngDuty[CHAN_1] = acc;
				engDuty[CHAN_3] = acc;
			}
		} else {
			//acc = ch1_mdt - 5 * g_rudderPara.powLeft;
			acc = ch1_mdt + 5*_rudderPara.powLeft;
			//if (acc < 1000) {
			if (acc > 2000) {
				//g_EngDuty[CHAN_1]  = 1000;
				engDuty[CHAN_3]  = 2000;
			} else {
				//g_EngDuty[CHAN_1] = acc;
				engDuty[CHAN_3] = acc;
			}
			
		}

	}
	

}

void AP_InertialSensor_NRF24L01::ImpChanDuty()
{
	//uint8_t i = 0;
	printf("set ch1[%d], ch3[%d]\n", engDuty[CHAN_1], engDuty[CHAN_3]);

	//do_set_servo(CHAN_1, 2000);
	//do_set_servo(CHAN_3, 2000);

	//do_set_servo(CHAN_2, engDuty[CHAN_1]);
	//do_set_servo(CHAN_4, engDuty[CHAN_3]);

	//do_set_servo(CHAN_5, engDuty[CHAN_1]);
	//do_set_servo(CHAN_6, engDuty[CHAN_3]);


	//do_set_servo(CHAN_7, engDuty[CHAN_1]);
	//do_set_servo(CHAN_8, engDuty[CHAN_3]);

/*
	for (i=0;i<8;i++) {	
		do_set_servo(CHAN_1 + i, engDuty[CHAN_1]);
		do_set_servo(CHAN_8 + i, engDuty[CHAN_3]);
	}*/
}

bool AP_InertialSensor_NRF24L01::get_nrf24l01_input(float &throttle, float &steering) 
{
	if (_rudderPara.isLeft) {
		steering = -1 * _rudderPara.powLeft * 45;
	} else {
		steering = _rudderPara.powLeft * 45;
	}

	if (_rudderPara.isUp) {
		throttle = -1 * _rudderPara.powUp;
	} else {
		throttle =  _rudderPara.powUp;
	}

	//printf("%s steering[%d], throttle[%d]\n", steering, throttle);
	
	return true;
}

bool AP_InertialSensor_NRF24L01::get_nrf24l01_mode(OperaType &mode)
{
	mode = (OperaType)_rudderPara.oprMode;
	return true;
}


uint8_t AP_InertialSensor_NRF24L01::rcvDataHandle(uint8_t *buf) 
{

//	uint8_t i = 0;
	uint8_t index = sizeof("gas:")-1;
/*
	printf("datas: ");
	for(i=0;i<32;i++) {
		printf("%x ", buf[i]);
	}
	printf("\n");
	*/
	
	//dprint("%s\r\n",buf);
	//dprintBuf((char*)buf,32, "rfBuf");
	
	if (!memcmp(buf,"gas:",index++)) {
		_rudderPara.isLeft  = buf[index++];
		_rudderPara.powLeft = buf[index++];
		_rudderPara.isUp = buf[index++];
		_rudderPara.powUp = buf[index++];	
		//g_rudderPara.isReverse = buf[index++];
		// 翻转判读由gyprosensor决定
		_rudderPara.dummy = buf[index++];
		_rudderPara.oprMode = buf[index++];
		_rudderPara.ch5set = buf[index++];
		_rudderPara.ch6set = buf[index++];
		_rudderPara.ch1_mft = (int8_t)buf[index++];
		_rudderPara.ch2_mft = (int8_t)buf[index++];
		_rudderPara.isResponse = buf[index++];
		// 接收到一次数据，则清零超时计数器
		// g_hsTimeout = 0;
		// pow Ϊ0 ~ 100
		/*printf("isLeft:%d, powLeft:%d, isUp:%d, powUp:%d, isRvs:%d\r\n", 
			_rudderPara.isLeft, _rudderPara.powLeft,_rudderPara.isUp,_rudderPara.powUp,_rudderPara.isReverse);*/

		//printf("mode: %d\n", _rudderPara.oprMode);

		//EnginesDutyCalc();
		//ImpChanDuty();
		return 1;
	}
	return 0;

//	if (!rover.set_home(cmd.content.location, false)) {
            // ignored...
//    }

}

void AP_InertialSensor_NRF24L01::printf(const char *fmt, ...)
{

	//hal.uartE->begin(115200);

    va_list ap;
    va_start(ap, fmt);
    hal.uartC->vprintf(fmt, ap);
    va_end(ap);
}

void AP_InertialSensor_NRF24L01::_set_mode( nRf24l01ModeType Mode )
{
    uint8_t controlreg = 0;

	//RF24L01_Init_0(g_uniqueId);
	
	controlreg = _register_read( CONFIG );
	
    if( Mode == MODE_TX )       
	{
		controlreg &= ~( 1<< PRIM_RX );
	}
    else if (Mode == MODE_RX )
	{
		controlreg |= ( 1<< PRIM_RX ); 
	}

	//NRF24L01_Set_TxAddr( g_uniqueId, 5 );                      //设置TX地址
    //NRF24L01_Set_RxAddr( 0, g_uniqueId, 5 );                   //设置RX地址
    _register_write( CONFIG, controlreg );
}

uint8_t AP_InertialSensor_NRF24L01::BuildRspPkt(uint8_t *buf) 
{
	uint8_t index = 0;

	// set voltage for test
	pwr_voltage[0] = 1986;
	pwr_voltage[1] = 2020;
	
	memcpy(buf, RSP_HEAD, sizeof(RSP_HEAD)-1);
	index += sizeof(RSP_HEAD);
	memcpy(buf+index, (uint8_t*)&pwr_voltage[0], 4);
	index += 4;
	//增加�?路电�?
	memcpy(buf+index, (uint8_t*)&pwr_voltage[1], 4);
	index += 4;
	memcpy(buf+index,RSP_END,sizeof(RSP_END)-1);
	index += sizeof(RSP_END);
	return index;
}


void AP_InertialSensor_NRF24L01::responseProc() 
{

	uint8_t rspBuf[32] = {0};
	uint8_t cnt = 0;
	uint8_t len = 0;
	
	if (_rudderPara.isResponse > 0) {

		//printf("response\r\n");
		
		_set_mode(MODE_TX);
		//delay_ms(5);
		hal.scheduler->delay(5);
		len = BuildRspPkt(rspBuf);
		//dprint("vol: %d\r\n", g_pwrVoltage);
		//dprintBuf((char*)rspBuf,32,"rspbuf");
		
		while(cnt < 10) {
			//if(NRF24L01_TxPacket(rspBuf,len)==TX_OK)
			if(txPacket(rspBuf,len) & TX_OK){
				break;
			}
			hal.scheduler->delay(10);
			cnt ++;
		}
		hal.scheduler->delay(5);
		_set_mode(MODE_RX);

	
	}
}

/**
  * @brief :NRF24L01发�?�一次数�?
  * @param :
  *			@txbuf:待发送数据首地址
  *			@Length:发�?�数据长�?
  * @note  :�?
  * @retval:
  *			MAX_TX：达到最大重发次�?
  *			TX_OK：发送完�?
  *			0xFF:其他原因
  */ 
uint8_t AP_InertialSensor_NRF24L01::txPacket( uint8_t *txbuf, uint8_t Length )
{
	uint8_t l_Status = 0;
	uint16_t l_MsTimes = 0;

	_cmd_write(FLUSH_TX);

	palClearPad(GPIOA, 8U);
	_write_buf(WR_TX_PLOAD, txbuf, Length);
	palSetPad(GPIOA, 8U);
	
	while( 0 != palReadLine(HAL_PIN_NRF24L01_INT))
	{
		hal.scheduler->delay(1);
		if( 50 == l_MsTimes++ )						//500ms还没有发送成功，重新初始化设�?
		{
			break;
		}
	}
	l_Status = _register_read(STATUS);						//读状态寄存器
	_register_write(STATUS, l_Status);						//清除TX_DS或MAX_RT中断标志
	
	if( l_Status & MAX_TX )	//达到�?大重发次�?
	{
		_register_write( FLUSH_TX,0xff );	//清除TX FIFO寄存�?
		//return MAX_TX; 
	}
	return l_Status;

	/*
	if( l_Status & TX_OK )	//发�?�完�?
	{
		return TX_OK;
	}
	
	return 0xFF;	//其他原因发�?�失�?
	*/
}


void AP_InertialSensor_NRF24L01::dprintBuf(const char* buf, uint8_t len, char* name) 
{
	int i = 0;
	printf("%s[%d]: ", name , len);
	for (i=0;i<len;i++) {
		printf("%x ", buf[i]);
	}
	printf("\r\n");
}



#define PAIR_TRY_TIMES  10
#define PAIR_WAIT_TIMES 10
uint8_t AP_InertialSensor_NRF24L01::pairProc() 
{

	uint8_t addr[5] = {PAIR_ADDR};
	uint8_t rcvBuf[32];
	uint8_t ret = 0;
	uint8_t cnt = 0;
//	u8 i = 0;

	printf("pair buoy\r\n");
	//dprintBuf((char*)addr, 5, "PairAddr");
	
	_set_tx_addr(addr,5);
	_set_rx_addr(0,addr, 5);

	_set_mode(MODE_TX);
	hal.scheduler->delay(1);

	memcpy(rcvBuf,PAIR_REQ_HEAD,sizeof(PAIR_REQ_HEAD)-1);

	while(cnt++<PAIR_TRY_TIMES)
	{
		printf("err: %d\r\n", cnt);
		ret = txPacket(rcvBuf,32);
		if (TX_OK & ret) {
			break;
		}
		hal.scheduler->delay(1);
	}

	if (cnt >= PAIR_TRY_TIMES) {
		return ret;
	}

	_set_mode(MODE_RX);
	hal.scheduler->delay(5);

	for (cnt = 0;cnt<PAIR_WAIT_TIMES;cnt++) {

		ret = 0;
		if (palReadLine(HAL_PIN_NRF24L01_INT) == 0) {
			ret = rx_packet(rcvBuf);
			if (ret > 0) {
				break;
			}
		}
		hal.scheduler->delay(30);
	}
	
	if (ret > 0) {
		dprintBuf((char*)rcvBuf,32,"rcv");
		if (!memcmp(rcvBuf,UNIQUE_ID_HEAD,sizeof(UNIQUE_ID_HEAD)-1)) {
			memcpy(_uniqueId, rcvBuf+sizeof(UNIQUE_ID_HEAD)-1, 5);

			dprintBuf((char*)_uniqueId, 5, "uniqueId");
			
			_set_mode(MODE_TX);

			memcpy(rcvBuf, PAIR_OK_HEAD, sizeof(PAIR_OK_HEAD)-1);

			cnt = 0;
			while (cnt++<10) {
				ret = txPacket(rcvBuf,32);
				
				if (TX_OK & ret) {
					printf("pair ok\r\n");
					memcpy(rcvBuf, UNIQUE_ID_HEAD, sizeof(UNIQUE_ID_HEAD)-1);
					memcpy(rcvBuf+sizeof(UNIQUE_ID_HEAD)-1, _uniqueId, 5);
					save_rf_id(_uniqueId, RF_ID_OFFSET, 5);

					_set_tx_addr(_uniqueId,5);
					_set_rx_addr(0,_uniqueId,5);
				}
			}
		}
	}else{
		return 0;
	}
	
	return 0;
}

bool AP_InertialSensor_NRF24L01::do_set_servo(uint8_t _channel, uint16_t pwm)
{
    SRV_Channel *c = SRV_Channels::srv_channel(_channel-1);
    if (c == nullptr) {
		printf("SRV_Channel is null\n");
        return false;
    }
	/*
    switch(c->get_function())
    {
    case SRV_Channel::k_none:
    case SRV_Channel::k_manual:
    case SRV_Channel::k_rcin1 ... SRV_Channel::k_rcin16: // rc pass-thru
        break;
    default:
        gcs().send_text(MAV_SEVERITY_INFO, "ServoRelayEvent: Channel %d is already in use", _channel);
		printf("ServoRelayEvent: Channel %d is already in use\n", _channel);
        return false;
    }*/
	/*
    if (type == EVENT_TYPE_SERVO && 
        channel == _channel) {
        // cancel previous repeat
        repeat = 0;
    }*/
    //printf("set output pwm\n");
    c->set_output_pwm(pwm);
    //c->ignore_small_rcin_changes();
    return true;
}


