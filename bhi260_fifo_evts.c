/*
 * bhi260_fifo_evts.c
 *
 *  Created on: Sep 10, 2021
 *      Author: btrud
 */

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <queue.h>
#include "bhi260_fifo_evts.h"
#include "bhi260_defs.h"
#include "bhi260_spi.h"
#include "bhi260_helper.h"
#include "board.h"
#include "debug.h"

/******************************************************************************
 * Local Constants
 ******************************************************************************/
// Meta Event queues defines
#define META_EVENTS_INITIALIZED_QUEUE_LEN		4
#define META_EVENTS_SMPL_RATE_CHNG_QUEUE_LEN	8
#define META_EVENTS_PWR_MODE_CHNG_QUEUE_LEN		8
#define META_EVENTS_SENS_STATUS_QUEUE_LEN		8
// Sensor Data Events queue length
#define SENSOR_DATA_EVENTS_QUEUE_LEN			32
// Sensor Data Events queue length
#define STATUS_EVENTS_QUEUE_LEN					8
// Temporal FIFO buffer size
#define FIFO_DATA_BLOCK_SIZE					512
// Maximum virtual sensors supported
#define MAX_SENSORS_SUPPORTED					94		// Supports up to sensor ID=93

// Debug options
#define ENABLE_DEBUG        true
#define DEBUG_TAG           "BHI260_FIFO"

#if ENABLE_DEBUG == true
#define DEBUG_PRINTF(format, ...) Debug_Print(DEBUG_TAG, true, format, ##__VA_ARGS__)
#define DEBUG_PRINT(format, ...) Debug_Print(DEBUG_TAG, false, format, ##__VA_ARGS__)
#define PRINTF(format, ...) Debug_Print(0, true, format, ##__VA_ARGS__)
#define PRINT(format, ...) Debug_Print(0, false, format, ##__VA_ARGS__)
#else
#define DEBUG_PRINTF(format, ...)
#define DEBUG_PRINT(format, ...)
#define PRINTF(format, ...)
#define PRINT(format, ...)
#endif

// Enables the printing of the raw content on the FIFO
#define ENABLE_FIFO_RAW_PRINT				false

// Enables the printing of every new data received on Status FIFO
#define ENABLE_STATUS_FIFO_PRINT			false

/******************************************************************************
 * Local Types
 ******************************************************************************/
/**
 * @brief Local main states
 * 
 */
typedef enum
{
	STATE_INIT = 0,
	STATE_WAIT,
	STATE_REQUEST_FIFO_PENDING,
	STATE_WAKEUP_READ,
	STATE_NONWAKEUP_READ,
	STATE_STATUS_READ,
	STATE_EXIT
}state_t;

/******************************************************************************
 * Local Variables
 ******************************************************************************/
/**
 * @brief Local data container
 * 
 */
static struct local_data
{
	state_t state;

	bool wakeupPend;
	bool nonWakeupPend;
	bool statusPend;
	
	// Temporal buffer for storing the FIFO content
	uint8_t fifo_buf[FIFO_DATA_BLOCK_SIZE];
	uint16_t fifo_buf_size;

	// Meta event queues
	queue_t meta_evts_initialized_queue;
	meta_event_t meta_evts_initialized_buf[META_EVENTS_INITIALIZED_QUEUE_LEN];
	queue_t meta_evts_smpl_rate_chng_queue;
	meta_event_t meta_evts_smpl_rate_chng_buf[META_EVENTS_SMPL_RATE_CHNG_QUEUE_LEN];
	queue_t meta_evts_pwr_mode_chng_queue;
	meta_event_t meta_evts_pwr_mode_chng_buf[META_EVENTS_PWR_MODE_CHNG_QUEUE_LEN];
	queue_t meta_evts_sens_status_queue;
	meta_event_t meta_evts_sens_status_buf[META_EVENTS_SENS_STATUS_QUEUE_LEN];

	// Meta events queue variables
	queue_t sensor_data_evts_queue;
	sensor_data_event_t sensor_data_evts_buf[SENSOR_DATA_EVENTS_QUEUE_LEN];

	// Meta events queue variables
	queue_t status_evts_queue;
	status_event_t status_evts_buf[STATUS_EVENTS_QUEUE_LEN];
}this;

/**
 * @brief Constant array that returns the type of the FIFO event
 * 
 */
static const event_type_t event_type_list[256] = 
{
	FIFO_EVT_TYPE_PADDING, 
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_VIRTUAL_SENSOR,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_TIMESTAMP_SMALL_DELTA,
	FIFO_EVT_TYPE_TIMESTAMP_LARGE_DELTA,
	FIFO_EVT_TYPE_TIMESTAMP_FULL,
	FIFO_EVT_TYPE_META_EVENT,
	FIFO_EVT_TYPE_INVALID,
	FIFO_EVT_TYPE_DEBUG_DATA,
	FIFO_EVT_TYPE_TIMESTAMP_SMALL_DELTA,
	FIFO_EVT_TYPE_TIMESTAMP_LARGE_DELTA,
	FIFO_EVT_TYPE_TIMESTAMP_FULL,
	FIFO_EVT_TYPE_META_EVENT,
	FIFO_EVT_TYPE_FILLER
};

/**
 * @brief Constant array that returns the type sensor data ID
 * 
 */
// TODO: pending to add the virtual sensors with ID > 93. Quaternions
// and 3D vectors are all covered
static const event_type_t sensor_data_type_list[MAX_SENSORS_SUPPORTED] =
{
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_QUATERNION,
	BHI260_SENS_DATA_TYPE_QUATERNION,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_QUATERNION,
	BHI260_SENS_DATA_TYPE_QUATERNION,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_QUATERNION,
	BHI260_SENS_DATA_TYPE_QUATERNION,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_EULER,
	BHI260_SENS_DATA_TYPE_EULER,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_INVALID,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
};

/******************************************************************************
 * Local Functions
 ******************************************************************************/
/**
 * @brief Parses a new meta event and adds it to the meta event queue
 * 
 * @param buf buffer containing the raw data
 * @param timestamp timestamp of the event
 */
static void ParseMetaEvent(uint8_t *buf, uint64_t timestamp)
{
	meta_event_t evt;

	evt.timestamp = timestamp;
	evt.id = *(buf+1);
	evt.values[0] = *(buf+2);
	evt.values[1] = *(buf+3);

	// Push the event to the corresponding queue
	switch (evt.id)
	{
		case BHY2_META_EVENT_INITIALIZED:
			QUEUE_Push(&this.meta_evts_initialized_queue,&evt);
			break;
		
		case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
			QUEUE_Push(&this.meta_evts_smpl_rate_chng_queue,&evt);
			break;
		
		case BHY2_META_EVENT_POWER_MODE_CHANGED:
			QUEUE_Push(&this.meta_evts_pwr_mode_chng_queue,&evt);
			break;
		
		case BHY2_META_EVENT_SENSOR_STATUS:
			QUEUE_Push(&this.meta_evts_sens_status_queue,&evt);
			break;
		
		default:
			break;
	}
}

/**
 * @brief Get the Sensor Data Type from the given sensor data event
 * 
 * @param evt sensor data event to evaluate
 * @return sensor_data_type_t 
 */
sensor_data_type_t GetSensorDataType(sensor_data_event_t evt)
{
	if(evt.id > sizeof(sensor_data_type_list)) return BHI260_SENS_DATA_TYPE_INVALID;

	return(sensor_data_type_list[evt.id]);
}

/**
 * @brief Parses a new sensor data event and adds it to the sensor data event queue
 * 
 * @param block pointer to the block containing the FIFO block raw data
 * @param timestamp timestamp of the event
 * 
 * @return the ammount of bytes parsed from the buffer
 */
static uint8_t ParseSensorData(uint8_t *buf, uint64_t timestamp)
{
	sensor_data_event_t evt;
	uint8_t offset = 0;

	evt.timestamp = timestamp;
	evt.id = *buf;
	offset++;
	// Get the sensor data type
	evt.data_type = GetSensorDataType(evt);

	switch (evt.data_type)
	{
	case BHI260_SENS_DATA_TYPE_QUATERNION:
		memcpy(evt.data,buf+1,10);
		offset += 10;
		break;

	case BHI260_SENS_DATA_TYPE_3DVECTOR:
		memcpy(evt.data,buf+1,6);
		offset += 6;
		break;
	
	default:
		return 1;
		break;
	}

	// Push the event into the corresponding queue
	QUEUE_Push(&this.sensor_data_evts_queue,&evt);

	return offset;
}

/**
 * @brief arses a new status event and adds it to the status event queue
 * 
 * @param buf pointer to the buffer containing the data
 * @param fifo_payload_size the size of the payload in the Status FIFO
 */
static void ParseStatus(uint8_t *buf, uint16_t fifo_payload_size)
{
	status_event_t status;

	status.cmd = (uint16_t)*buf | (uint16_t)*(buf+1) << 8;
	status.length = fifo_payload_size;

	memcpy(status.payload,(buf+4),status.length);

#if ENABLE_STATUS_FIFO_PRINT == true
	DEBUG_PRINTF("Status and Debug FIFO received:");
	DEBUG_PRINTF("-------------------------------");
	PRINTF("\tCmd id: %d",status.cmd);
	PRINTF("\tPayload Length: %d ",status.length);
	PRINTF("\tPayload: ");
	BHI260_HELPER_print_buffer(status.payload,status.length);
#endif

	QUEUE_Push(&this.status_evts_queue,&status);
}

/**
 * @brief Reads one Status Fifo message from the Status FIFO
 * 
 * @param buf pointer to the buffer containing the data
 * @param buf_size data size in the buffer
 * 
 * @return BHI260_RET_OK if fifo block retrieved ok 
 * @return BHI260_RET_FAILED if failed 
 * @return BHI260_RET_WORKING if function busy 
 * @return BHI260_RET_IDLE if no more blocks in fifo
 */
static bhi260_ret_t ReadStatusFifo(uint8_t *buf, uint16_t buf_size)
{
	bhi260_ret_t ret = BHI260_RET_WORKING;
	static reg_read_or_write_t rReg;
	static uint8_t b[8];
	static uint16_t status_payload_len;

	// Local states
	static enum
	{
		READFIFO_STATUS_READ_START = 0,
		READFIFO_STATUS_CONTENT_LENGTH,
		READFIFO_STATUS_BLOCK_PAYLOAD,
	}rFifoState = READFIFO_STATUS_READ_START;

	switch (rFifoState)
	{
		case READFIFO_STATUS_READ_START:
			status_payload_len = 0;

			// Assert the CS line
			BHI260_SPI_CSassert();

			if(RegRead(&rReg,BHY2_REG_CHAN_STATUS,b,4,false))
			{
				rFifoState = READFIFO_STATUS_CONTENT_LENGTH;
			}
			else
			{
				BHI260_SPI_CSrelease();
				ret = BHI260_RET_FAILED;
				rFifoState = READFIFO_STATUS_READ_START;
			}
			break;
		
		case READFIFO_STATUS_CONTENT_LENGTH:
			if(RegProcess_Done(&rReg))
			{
				memcpy(buf,&b[1],4);
				status_payload_len = ((uint16_t)b[3] | (uint16_t)b[4]<<8);

				if(!status_payload_len)
				{
					BHI260_SPI_CSrelease();
					ret = BHI260_RET_IDLE;
					rFifoState = READFIFO_STATUS_READ_START;
				}
				else
				{
					if(RegRead(&rReg,BHY2_REG_CHAN_STATUS,buf+4,status_payload_len,false))
					{
						rFifoState = READFIFO_STATUS_BLOCK_PAYLOAD;
					}
					else
					{
						BHI260_SPI_CSrelease();
						ret = BHI260_RET_FAILED;
						rFifoState = READFIFO_STATUS_READ_START;
					}
				}
			}		
			break;
		
		case READFIFO_STATUS_BLOCK_PAYLOAD:
			if(RegProcess_Done(&rReg))
			{
				BHI260_SPI_CSrelease();

				// Parse the data read from Status FIFO
				ParseStatus(buf,status_payload_len);
				
				ret = BHI260_RET_OK;
				rFifoState = READFIFO_STATUS_READ_START;
			}
			break;

		default:
			ret = BHI260_RET_FAILED;
			break;
	}

	return ret;
}

/**
 * @brief Get the Evt Type of the given event ID
 * 
 * @param id identifier of the Event
 * @return event_type_t 
 */
static event_type_t GetEvtType(uint8_t id)
{
	return event_type_list[id];
}

/**
 * @brief Parses 1 block of data stored in the buffer read from data fifos
 * 
 * @param buf pointer to the buffer containing 1 fifo block of data (512Bytes max)
 * @param size amount of bytes on the buffer
 */
static void ParseFifoBlock(uint8_t *buf, uint16_t size)
{
	uint16_t idx = 0;
	uint64_t timestamp = 0;

	if(GetEvtType(*(buf+idx)) == FIFO_EVT_TYPE_META_EVENT)
	{
		idx++;
		if(idx >= size)
		{
			DEBUG_PRINTF("ParseFifoBlock(): Fewer data than expected (idx=%d/size=%d)",
				idx, size);
			return;
		}

		if(MetaEvent_isSpacerOrOverflow(*(buf+idx)))
		{
			// Ignore the Spacer or Overflow MetaEvt content
			idx += 3;
			if(idx >= size)
			{
				DEBUG_PRINTF("ParseFifoBlock(): Fewer data than expected (idx=%d/size=%d)",
					idx, size);
				return;
			}

			if(GetEvtType(*(buf+idx)) == FIFO_EVT_TYPE_TIMESTAMP_FULL)
			{
				if(idx+6 >= size)
				{
					DEBUG_PRINTF("ParseFifoBlock(): Fewer data than expected (idx=%d/size=%d)",
						idx, size);
					return;
				}

				timestamp += *(buf+idx+5); timestamp <<= 8;
				timestamp += *(buf+idx+4); timestamp <<= 8;
				timestamp += *(buf+idx+3); timestamp <<= 8;
				timestamp += *(buf+idx+2); timestamp <<= 8;
				timestamp += *(buf+idx+1); timestamp <<= 8;
			
				// Prepare the index for the next process
				idx += 6;
				if(idx >= size)
				{
					DEBUG_PRINTF("ParseFifoBlock(): Fewer data than expected (idx=%d/size=%d)",
						idx, size);
					return;
				}
			}
			else
			{
				DEBUG_PRINTF("ParseFifoBlock(): Expected Full Timestamp Event after Spacer or Overflow");
				return;
			}
		}
		else
		{
			DEBUG_PRINTF("ParseFifoBlock(): expected Spacer or Overflow MetaEvt");
			return;
		}
	}
	else
	{
		DEBUG_PRINTF("ParseFifoBlock(): expected MetaEvt as first element in fifo");
		return;
	}

	// Start parsing all the data contained in the block
	while(idx < size)
	{
		switch(GetEvtType(*(buf+idx)))
		{
			case FIFO_EVT_TYPE_META_EVENT:
				ParseMetaEvent(buf+idx, timestamp);
				idx += 4;
				break;
			
			case FIFO_EVT_TYPE_VIRTUAL_SENSOR:
				idx += ParseSensorData(buf+idx, timestamp);
				break;

			case FIFO_EVT_TYPE_TIMESTAMP_LARGE_DELTA:
				idx += 3;
				break;

			default: idx++; break;
		}
	}
}

/**
 * @brief Parses the data stored in the buffer read from data fifos
 * 
 * @param buf pointer to the buffer containing the fifo data
 * @param size amount of bytes on the buffer
 * @param init initializes the fifo parser machine
 * 
 * @return true process finished
 * @return false in progress
 *
static bool ParseFifoData(uint8_t *buf, uint16_t size, bool init)
{
	static enum
	{
		pfSTATE_INIT = 0,
		pfSTATE_SALL_DELTA_TIMESTAMP,
		pfSTATE_TIMESTAMP,
		
	}
}*/

/**
 * @brief Reads a bulk of data from the FIFO and stores it a temporal buffer
 * 
 * @param fifoID determines which fifo will be read
 * @param buf buffer where to store the data read from the FIFO
 * @param buf_size size of the buffer
 * @return BHI260_RET_OK if fifo block retrieved ok 
 * @return BHI260_RET_FAILED if failed 
 * @return BHI260_RET_WORKING if function busy 
 * @return BHI260_RET_IDLE if no more blocks in fifo
 */
static bhi260_ret_t ReadDataFifo(fifoID_t fifoID, uint8_t *buf, uint16_t buf_size)
{
	bhi260_ret_t ret = BHI260_RET_WORKING;
	static reg_read_or_write_t rReg;
	static uint8_t reg_addr;
	static uint16_t fifoLen;

	// Local states
	static enum
	{
		READFIFO_STATE_INIT = 0,
		READFIFO_READ_START,
		READFIFO_CONTENT_LENGTH,
		READFIFO_PAYLOAD,
	}rFifoState = READFIFO_STATE_INIT;

	switch (rFifoState)
	{
	case READFIFO_STATE_INIT:
		fifoLen = 0;

		switch(fifoID)
		{
			case FIFO_WAKEUP:
				reg_addr = BHY2_REG_CHAN_FIFO_W;
				rFifoState = READFIFO_READ_START;
				break;
			
			case FIFO_NONWAKEUP:
				reg_addr = BHY2_REG_CHAN_FIFO_NW;
				rFifoState = READFIFO_READ_START;
				break;
			
			default: return BHI260_RET_FAILED;
		}
		
		break;
	
	case READFIFO_READ_START:
		// Assert the CS line
		BHI260_SPI_CSassert();

		// Read 4 Bytes, the first 2Bytes will contain the FIFO length and the second 2Bytes the 
		// Small Delta Timestamp
		if(RegRead(&rReg,reg_addr,buf,4,false))
		{
			rFifoState = READFIFO_CONTENT_LENGTH;
		}
		else
		{
			BHI260_SPI_CSrelease();
			ret = BHI260_RET_FAILED;
			rFifoState = READFIFO_STATE_INIT;
		}
		break;
	
	case READFIFO_CONTENT_LENGTH:
		if(RegProcess_Done(&rReg))
		{
			fifoLen = (uint16_t)buf[1] | (buf[2] << 8);

			// Discount the Smal Delta Timestamp (2 Bytes)
			if(fifoLen >=2) fifoLen -= 2;		
			
			// Fifo empty
			if(fifoLen == 0)
			{
				BHI260_SPI_CSrelease();
				ret = BHI260_RET_IDLE;
				rFifoState = READFIFO_STATE_INIT;
			}
			else
			{
				if(fifoLen > buf_size) fifoLen = buf_size;

				this.fifo_buf_size = fifoLen;

				if(RegRead(&rReg,reg_addr,buf,fifoLen,false))
				{
					rFifoState = READFIFO_PAYLOAD;
				}
				else
				{
					BHI260_SPI_CSrelease();
					ret = BHI260_RET_FAILED;
					rFifoState = READFIFO_STATE_INIT;
				}
			}
		}
		break;
	
	case READFIFO_PAYLOAD:
		if(RegProcess_Done(&rReg))
		{
			BHI260_SPI_CSrelease();	
			
#if ENABLE_FIFO_RAW_PRINT == true
			DEBUG_PRINT("[%d]: ",fifoLen);
			for(int i = 0; i < fifoLen; i++) PRINT("%02x ", *(buf + i));
			PRINTF("");
#endif
			ParseFifoBlock(this.fifo_buf,this.fifo_buf_size);
			
			ret = BHI260_RET_OK;
			rFifoState = READFIFO_STATE_INIT;
		}
		break;
	
	default:
		ret = BHI260_RET_FAILED;
		break;
	}

	return ret;
}

/**
 * @brief Reads all the pending block of the desired FIFO. All the blocks
 * are parsed and pushed to the corresponding queue
 * 
 * @return process status <bhi260_ret_t>
 */
static bhi260_ret_t FifoGet(fifoID_t fifoID)
{
	bhi260_ret_t ret;

	if(fifoID >= FIFO_MAX) return BHI260_RET_FAILED;

	//ret = ReadFifoBlock(fifoID,&block);
	if(fifoID == FIFO_STATUS)
	{
		ret = ReadStatusFifo(this.fifo_buf, sizeof(this.fifo_buf));
	}
	else
	{
		ret = ReadDataFifo(fifoID, this.fifo_buf, sizeof(this.fifo_buf));
	}	

	return ret;
}

/******************************************************************************
 * Callback Functions
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/
void BHI260_FIFO_Init()
{
	this.state = STATE_INIT;

	// Create the queues for the initialized meta events
	QUEUE_Create(&this.meta_evts_initialized_queue,
		this.meta_evts_initialized_buf,
		META_EVENTS_INITIALIZED_QUEUE_LEN,
		sizeof(meta_event_t));

	// Create the queues for the sample rate changed meta events
	QUEUE_Create(&this.meta_evts_smpl_rate_chng_queue,
		this.meta_evts_smpl_rate_chng_buf,
		META_EVENTS_SMPL_RATE_CHNG_QUEUE_LEN,
		sizeof(meta_event_t));

	// Create the queues for the power mode changed meta events
	QUEUE_Create(&this.meta_evts_pwr_mode_chng_queue,
		this.meta_evts_pwr_mode_chng_buf,
		META_EVENTS_PWR_MODE_CHNG_QUEUE_LEN,
		sizeof(meta_event_t));

	// Create the queues for the power mode changed meta events
	QUEUE_Create(&this.meta_evts_sens_status_queue,
		this.meta_evts_sens_status_buf,
		META_EVENTS_SENS_STATUS_QUEUE_LEN,
		sizeof(meta_event_t));

	// Create the queue for the sensor data events
	QUEUE_Create(&this.sensor_data_evts_queue,
		this.sensor_data_evts_buf,
		SENSOR_DATA_EVENTS_QUEUE_LEN,
		sizeof(sensor_data_event_t));

	// Create the queue for the status events
	QUEUE_Create(&this.status_evts_queue,
		this.status_evts_buf,
		STATUS_EVENTS_QUEUE_LEN,
		sizeof(status_event_t));
}

void BHI260_FIFO_Tasks()
{
	static uint8_t RXbyte[3];
	// Register operations
	static reg_read_or_write_t rReg;

	switch(this.state)
	{
		case STATE_INIT:			
			if(BOARD_isBHIinterrupt())
			{
				this.wakeupPend = false;
				this.nonWakeupPend = false;
				this.statusPend = false;
				this.state = STATE_WAIT;
			}
			break;
		
		case STATE_WAIT:
			if(BHI260_SPI_Take())
			{
				RegRead(&rReg,BHY2_REG_INT_STATUS,RXbyte,1,true);
				this.state = STATE_REQUEST_FIFO_PENDING;
			}
			break;
		
		case STATE_REQUEST_FIFO_PENDING:
			if(RegProcess_Done(&rReg))
			{
				this.wakeupPend = !BHY2_INT_STATUS_Is_WakeupNodata(RXbyte[1]);
				this.nonWakeupPend = !BHY2_INT_STATUS_Is_NonWakeupNodata(RXbyte[1]);
				this.statusPend = BHY2_INT_STATUS_Is_Status(RXbyte[1]);
				
				// Ask, just in case
				if(!this.wakeupPend && !this.nonWakeupPend && !this.statusPend)
				{
					this.state = STATE_EXIT;
				}
				else
				{
					this.state = STATE_WAKEUP_READ;
				}				
			}
			break;
		
		case STATE_WAKEUP_READ:
			if(this.wakeupPend)
			{
				switch(FifoGet(FIFO_WAKEUP))
				{
					case BHI260_RET_FAILED:
					case BHI260_RET_IDLE:
						this.state = STATE_NONWAKEUP_READ;
						break;
					
					case BHI260_RET_OK:
					case BHI260_RET_WORKING: break;

					default:
						this.state = STATE_EXIT;
						break;
				}
			}
			else
			{
				this.state = STATE_NONWAKEUP_READ;
			}			
			break;
		
		case STATE_NONWAKEUP_READ:
			if(this.nonWakeupPend)
			{
				switch(FifoGet(FIFO_NONWAKEUP))
				{
					case BHI260_RET_FAILED:
					case BHI260_RET_IDLE:
						this.state = STATE_STATUS_READ;
						break;
					
					case BHI260_RET_OK:
					case BHI260_RET_WORKING: break;

					default:
						this.state = STATE_EXIT;
						break;
				}
			}
			else
			{
				this.state = STATE_STATUS_READ;
			}						
			break;
		
		case STATE_STATUS_READ:
			if(this.statusPend)
			{
				switch(FifoGet(FIFO_STATUS))
				{
					case BHI260_RET_FAILED:
					case BHI260_RET_IDLE:
						this.state = STATE_EXIT;
						break;
					
					case BHI260_RET_OK:
					case BHI260_RET_WORKING: break;

					default:
						this.state = STATE_EXIT;
						break;
				}
			}
			else
			{
				this.state = STATE_EXIT;
			}
			break;
		
		case STATE_EXIT:
			if(BHI260_SPI_Give())
			{
				this.state = STATE_INIT;
			}
			break;
		
		default: break;
	}
}

bool BHI260_FIFO_Busy()
{
	return (this.state != STATE_INIT && this.state != STATE_WAIT);
}

bool BHI260_FIFO_MetaEvent_Initialized_Pull(meta_event_t *dst)
{
	return (QUEUE_Pull(&this.meta_evts_initialized_queue,dst));
}

bool BHI260_FIFO_MetaEvent_PwrModeChanged_Pull(meta_event_t *dst)
{
	return (QUEUE_Pull(&this.meta_evts_smpl_rate_chng_queue,dst));
}

bool BHI260_FIFO_MetaEvent_SmplRateChanged_Pull(meta_event_t *dst)
{
	return (QUEUE_Pull(&this.meta_evts_pwr_mode_chng_queue,dst));
}

bool BHI260_FIFO_MetaEvent_SensorStatus_Pull(meta_event_t *dst)
{
	return (QUEUE_Pull(&this.meta_evts_sens_status_queue,dst));
}

bool BHI260_FIFO_Status_Pull(status_event_t *dst)
{
	return (QUEUE_Pull(&this.status_evts_queue,dst));
}

bool BHI260_FIFO_SensorData_Pull(sensor_data_event_t *dst)
{
	return (QUEUE_Pull(&this.sensor_data_evts_queue,dst));
}
