/*
 * File      : ms5611_sensor.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-09-30     zoujiachi    first version.
 */
 
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <math.h>
#include "sensor_manager.h"
#include "delay.h"
#include "conversion.h"

#define POW2(_x)		((_x) * (_x))

/* SPI protocol address bits */
#define DIR_READ                (1<<7)
#define DIR_WRITE               (0<<7)

#define ADDR_RESET_CMD				0x1E	/* write to this address to reset chip */
#define ADDR_CMD_CONVERT_D1			0x48	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D2			0x58	/* write to this address to start temperature conversion */
#define ADDR_ADC					0x00	/* address of 3 bytes / 32bit pressure data */
#define ADDR_PROM_SETUP				0xA0	/* address of 8x 2 bytes factory and calibration data */
#define ADDR_PROM_C1				0xA2	/* address of 6x 2 bytes calibration data */
#define ADDR_PROM_C2				0xA4
#define ADDR_PROM_C3				0xA6
#define ADDR_PROM_C4				0xA8
#define ADDR_PROM_C5				0xAA
#define ADDR_PROM_C6				0xAC
#define ADDR_PROM_CRC				0xAE

#define INTERVAL_CONV_TIME			10		/* the max conv time is 9ms */

static rt_device_t spi_device;
static struct rt_device baro_device;

uint32_t _raw_temperature , _raw_pressure;

int32_t _dT;

MS5611_PROM_Def _prom;
//MS5611_REPORT_Def report;

rt_err_t ms5611_write_cmd(rt_uint8_t cmd)
{
	rt_uint8_t send_buffer;
	rt_size_t w_byte;
	
	send_buffer = DIR_WRITE | cmd;
	w_byte = rt_device_write(spi_device , 0 , &send_buffer , sizeof(send_buffer));
	
	return w_byte == sizeof(send_buffer) ? RT_EOK : RT_ERROR;
}

rt_err_t ms5611_read_adc(uint32_t* buff)
{
	rt_uint8_t send_val;
	rt_err_t res;

	send_val = ADDR_ADC;
	
	res = rt_spi_send_then_recv((struct rt_spi_device *)spi_device , (void*)&send_val , 1 , (void*)buff, 3);
	//big-endian to little-endian
	Msb2Lsb((uint8_t*)buff , 3);
	
	return res;
}

rt_err_t ms5611_read_prom_reg(rt_uint8_t cmd , uint16_t* buff)
{
	rt_uint8_t send_val;
	rt_err_t res;

	send_val = DIR_READ | cmd;
	
	res = rt_spi_send_then_recv((struct rt_spi_device *)spi_device , (void*)&send_val , 1 , (void*)buff, 2);
	//big-endian to little-endian
	Msb2Lsb((uint8_t*)buff , 2);

	return res;
}

rt_bool_t crc_check(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read crc */
	crc_read = n_prom[7];

	/* remove CRC byte */
	n_prom[7] = (0xFF00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return true if CRCs match */
	return (0x000F & crc_read) == (n_rem ^ 0x00);
}

rt_err_t ms5611_load_prom(void)
{
	for(uint8_t i = 0 ; i<8 ; i++)
	{
		ms5611_read_prom_reg(ADDR_PROM_SETUP+(i<<1) , ((uint16_t*)&_prom)+i);
	}
	
	return crc_check((uint16_t*)&_prom) ? RT_EOK : RT_ERROR;
}

rt_err_t baro_collect_data(void *args)
{
	MS5611_REPORT_Def* report = (MS5611_REPORT_Def*)args;
	
	report->raw_pressure = _raw_pressure;
	report->raw_temperature = _raw_temperature;
	
	_dT = _raw_temperature - ((int32_t)_prom.c5<<8);
	int32_t _temp = 2000 + (int32_t)(((int64_t)_dT * _prom.c6) >> 23);
	
	int64_t OFF = ((int64_t)_prom.c2 << 16) + (((int64_t)_prom.c4 * _dT) >> 7);
	int64_t SENS = ((int64_t)_prom.c1 << 15) + (((int64_t)_prom.c3 * _dT) >> 8);
	
	/* temperature compensation */
	if(_temp < 2000){
		int32_t T2 = POW2(_dT) >> 31;

		int64_t f = POW2((int64_t)_temp - 2000);
		int64_t OFF2 = 5 * f >> 1;
		int64_t SENS2 = 5 * f >> 2;

		if(_temp < -1500){
			int64_t f2 = POW2(_temp + 1500);
			OFF2 += 7 * f2;
			SENS2 += 11 * f2 >> 1;
		}

		_temp -= T2;
		OFF  -= OFF2;
		SENS -= SENS2;
	}
	
	int32_t _pressure = (((_raw_pressure * SENS) >> 21) - OFF) >> 15;
	
	report->temperature = _temp / 100.0f;
	report->pressure = _pressure / 100.0f;
	
	/* tropospheric properties (0-11km) for standard atmosphere */
	const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin, [K] = [°C] + 273.15 */
	const double a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
	const double g  = 9.80665;	/* gravity constant in m/s/s */
	const double R  = 287.05;	/* ideal gas constant in J/kg/K */
	
	/* current pressure at MSL in kPa */
	double p1 = 101325.0 / 1000.0;

	/* measured pressure in kPa */
	double p = _pressure / 1000.0;

	/*
	 * Solve:
	 *
	 *     /        -(aR / g)     \
	 *    | (p / p1)          . T1 | - T1
	 *     \                      /
	 * h = -------------------------------  + h1
	 *                   a
	 */
	//report->altitude = (((exp((-(a * R) / g) * log((p / p1)))) * T1) - T1) / a;
	report->altitude = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;
	
	report->time_stamp = time_nowMs();

	return RT_EOK;
}

rt_err_t baro_init(rt_device_t dev)
{	
	rt_device_open(spi_device , RT_DEVICE_OFLAG_RDWR);

	/* reset first */
	ms5611_write_cmd(ADDR_RESET_CMD);
	
	/* device need 2.8ms reload time */
	time_waitMs(5);
	
	/* load prom */
	rt_err_t res = ms5611_load_prom();
	
	_raw_temperature = _raw_pressure = 0;
	
	return res;
}

rt_size_t baro_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
	rt_err_t res;
	
	if(pos == RAW_TEMPERATURE_POS)	//read temperature raw data
	{
		res = ms5611_read_adc(&_raw_temperature);
	}
	else if(pos == RAW_PRESSURE_POS)	//read pressure raw data
	{
		res = ms5611_read_adc(&_raw_pressure);
	}
	else if(pos == COLLECT_DATA_POS)
	{
		res = baro_collect_data(buffer);
	}
	
	return (res == RT_EOK) ? size : 0;
}

rt_err_t baro_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	rt_err_t res = RT_EOK;
	static uint32_t last_conv_time = 0xFFFFFFFF;
	
	switch(cmd)
	{
		case SENSOR_CONVERSION:
		{
			if(*(u8*)args != ADDR_CMD_CONVERT_D1 && *(u8*)args != ADDR_CMD_CONVERT_D2)
			{
				return RT_ERROR;
			}
			
			ms5611_write_cmd(*(u8*)args);
			/* record last convertion time */
			last_conv_time = time_nowMs();
		}break;
		case SENSOR_IS_CONV_FIN:
		{
			if(time_nowMs() - last_conv_time >= INTERVAL_CONV_TIME)
			{
				res = RT_EOK;
			}else
			{
				res = RT_EBUSY;
			}
		}break;
		default:
			return RT_ERROR;
	}
	
	return res;
}

rt_err_t rt_ms5611_init(char* spi_device_name)
{	
	rt_err_t res = RT_EOK;;
	
	/* set device type */
    baro_device.type    = RT_Device_Class_SPIDevice;
    baro_device.init    = baro_init;
    baro_device.open    = RT_NULL;
    baro_device.close   = RT_NULL;
    baro_device.read    = baro_read;
    baro_device.write   = RT_NULL;
    baro_device.control = baro_control;
    
    /* register to device manager */
    res |= rt_device_register(&baro_device , "ms5611", RT_DEVICE_FLAG_RDWR);
	
	spi_device = rt_device_find(spi_device_name);
	
	if(spi_device == RT_NULL)
    {
        rt_kprintf("spi device %s not found!\r\n", spi_device_name);
        return RT_EEMPTY;
    }
	
	/* config spi */
	{
		struct rt_spi_configuration cfg;
		cfg.data_width = 8;
		cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible Modes 3 */	
		cfg.max_hz = 3000000;
		
		struct rt_spi_device* spi_device_t = (struct rt_spi_device*)spi_device;
		
		spi_device_t->config.data_width = cfg.data_width;
		spi_device_t->config.mode       = cfg.mode & RT_SPI_MODE_MASK ;
		spi_device_t->config.max_hz     = cfg.max_hz;
		res |= rt_spi_configure(spi_device_t, &cfg);
	}
	
	return res;
}

