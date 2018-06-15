/*
 * File      : Sensor_Manager.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-11-21     zoujiachi    first version.
 */
 
#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <string.h>

#define MAX_INTERVAL_NUM		32
#define ACC_UPDATE_INTERVAL		1
#define MAG_UPDATE_INTERVAL		1
#define GYR_UPDATE_INTERVAL		1
#define BARO_UPDATE_INTERVAL	1
#define GPS_UPDATE_INTERVAL		1

struct event
{
	uint8_t 		event_type;		//1:acc 2:mag 3:gyr 4:baro 5:gps
	void			(*callback)(uint8_t event_type, void* data);
	uint32_t		event_id;
	struct event*	prev;
	struct event*	next;
};
typedef struct event Event;

typedef struct
{
	uint32_t		time_interval;
	struct rt_timer	timer;
	struct event*	next;	
}Interval;

char TimerName[MAX_INTERVAL_NUM][4];

uint32_t IntervalMask = 0;
uint16_t EventCnt = 0;
Interval IntervalBuff[MAX_INTERVAL_NUM];
Sensor_Report sensor_report;

static rt_device_t acc_mag_device_t , gyr_device_t , baro_device_t, gps_device;
static struct vehicle_gps_position_s gps_position;
static struct satellite_info_s satellite_info;
static Baro_Machine_State baro_state;
static MS5611_REPORT_Def report_baro;

void _init_intervalBuff(void)
{
	uint8_t i;
	
	for(i = 0 ; i<MAX_INTERVAL_NUM ; i++){
		IntervalBuff[i].time_interval = 0;
		IntervalBuff[i].next = NULL;
	}
}

int8_t _find_free_intervalmask(void)
{
	uint8_t i;
	
	for(i = 0 ; i<MAX_INTERVAL_NUM ; i++){
		if(!(IntervalMask & (1<<i)))
			return i;
	}
	
	return -1;
}

void _set_intervalmask(uint8_t pos)
{
	IntervalMask |= (1<<pos);
}

void _reset_intervalmask(uint8_t pos)
{
	IntervalMask &= ~(1<<pos);
}

int8_t _find_timeinterval(uint32_t time_interval)
{
	uint8_t i;
	
	for(i = 0 ; i<MAX_INTERVAL_NUM ; i++){
		if(IntervalMask & (1<<i)){
			if(IntervalBuff[i].time_interval == time_interval)
				return i;
		}
	}
	
	return -1;
}

uint8_t _need_update(uint32_t time_stamp, uint32_t update_interval)
{
	if(time_stamp == 0 || (time_nowMs()-time_stamp) >= update_interval){
		return 1;
	}else{
		return 0;
	}
}

static void _timer_timeout(void* parameter)
{
	uint8_t i;
	uint16_t pos = 0xFFFF;
	
	for(i = 0 ; i<MAX_INTERVAL_NUM ; i++){
		if( strcmp((char*)parameter, TimerName[i]) == 0 ){
			pos = i;
			break;
		}
	}
	
	if(pos == 0xFFFF){
		printf("error, can not find specific timer name\r\n");
		return ;
	}
	
	for(Event* e = IntervalBuff[pos].next ; e != NULL ; e = e->next){
		switch(e->event_type)
		{
			case SENSOR_EVENT_ACC:
			{
				if(_need_update(sensor_report.acc_report.timestamp, ACC_UPDATE_INTERVAL)){
				
				}
			}break;
			case SENSOR_EVENT_MAG:
			{
			
			}break;
			case SENSOR_EVENT_GYR:
			{
			
			}break;
			case SENSOR_EVENT_BARO:
			{
			
			}break;
			case SENSOR_EVENT_GPS:
			{
			
			}break;
		}
	}
}

uint32_t 	// event_id  0:fail other:success
register_sensor_event(uint8_t event_type, uint32_t time_interval, void (*callback)(uint8_t event_type, void* data))
{	
	int16_t pos;
	Event* new_event;
	
	pos = _find_timeinterval(time_interval);
	
	if(pos >= 0){	//find same time interval
		new_event = (Event*)malloc(sizeof(Event));
		if(new_event == NULL){
			return 0;
		}
		
		Event* e = IntervalBuff[pos].next;
		
		//find tail
		while(e->next != NULL){
			e = e->next;
		}
		
		if(EventCnt == 0)
			EventCnt = 1;
		
		new_event->event_type = event_type;
		new_event->callback = callback;
		new_event->event_id = pos | ((EventCnt++)<<16);
		new_event->prev = e;
		new_event->next = NULL;
		
		e->next = new_event;

	}else{	//create new time interval
		int16_t new_pos = _find_free_intervalmask();
		
		if(new_pos < 0){
			printf("the interval is full! \r\n");
			return 0;
		}
		
		new_event = (Event*)malloc(sizeof(Event));
		if(new_event == NULL){
			return 0;
		}
		
		if(EventCnt == 0)
			EventCnt = 1;
		
		new_event->event_type = event_type;
		new_event->callback = callback;
		new_event->event_id = new_pos | ((EventCnt++)<<16);
		new_event->prev = NULL;
		new_event->next = NULL;
		
		//create new interval
		IntervalBuff[new_pos].time_interval = time_interval;
		IntervalBuff[new_pos].next = new_event;
		_set_intervalmask(new_pos);
		
		/* register timer event */
		rt_timer_init(&IntervalBuff[new_pos].timer, TimerName[new_pos],
						_timer_timeout,
						TimerName[new_pos],
						IntervalBuff[new_pos].time_interval,
						RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
		
		/* start timer */
		rt_timer_start(&IntervalBuff[new_pos].timer);
	}
}

rt_err_t deregister_sensor_event(uint32_t event_id)
{
	uint16_t pos = event_id & 0xFFFF;	//the lower 16bits store the pos of intervalbuff
	Event* e;
	
	for(e = IntervalBuff[pos].next ; e != NULL ; e = e->next){
		if(e->event_id == event_id){
			if(e->prev == NULL){	//the first node, then delete the interval
				IntervalBuff[pos].time_interval = 0;
				IntervalBuff[pos].next = NULL;
				_reset_intervalmask(pos);
			}else{
				e->prev->next = e->next;
				if(e->next != NULL){
					e->next->prev = e->prev;
				}
			}
			
			//release mememory
			free(e);
			
			return RT_EOK;
		}
	}
	
	return RT_ERROR;
}

rt_err_t SensorManager_Init(void)
{
	rt_err_t res;
	
	IntervalMask = 0;
	EventCnt = 0;
	
	//init TimerName
	for(uint8_t i = 0 ; i<MAX_INTERVAL_NUM ; i++)
	{
		sprintf((char*)TimerName[i], "T%d", i);
		
	}
	
	//init sensor_report
	sensor_report.acc_report.timestamp = 0;
	sensor_report.mag_report.timestamp = 0;
	sensor_report.gyr_report.timestamp = 0;
	sensor_report.baro_report.temperature = 0;
	sensor_report.gps_report.timestamp = 0;
	
	/* init acc_mag device */
	res = rt_lsm303d_init("spi_d1");
	
	acc_mag_device_t = rt_device_find(ACC_MAG_DEVICE_NAME);
	
	if(acc_mag_device_t == RT_NULL)
	{
		return RT_EEMPTY;
	}
	
	rt_device_open(acc_mag_device_t , RT_DEVICE_OFLAG_RDWR);
	
	/* init gyr device */
	res = rt_l3gd20h_init("spi_d2");
	
	gyr_device_t = rt_device_find(GYR_DEVICE_NAME);
	
	if(gyr_device_t == RT_NULL)
	{
		return RT_EEMPTY;
	}
	
	rt_device_open(gyr_device_t , RT_DEVICE_OFLAG_RDWR);
	
	/* init barometer device */
	baro_state = S_CONV_1;
	
	rt_ms5611_init("spi_d3");
	
	baro_device_t = rt_device_find(BARO_DEVICE_NAME);
	
	if(baro_device_t == RT_NULL)
	{
		return RT_EEMPTY;
	}
	
	rt_device_open(baro_device_t , RT_DEVICE_OFLAG_RDWR);
	
	/* init gps device */
	rt_gps_init("uart4" , &gps_position , &satellite_info);
	
	return res;
}
