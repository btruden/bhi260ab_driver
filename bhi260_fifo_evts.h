/*
 * bhi260_fifo_evts.h
 *
 *  Created on: Sep 10, 2021
 *      Author: btrud
 */

#ifndef INC_IMU_BHI260_FIFO_EVTS_H_
#define INC_IMU_BHI260_FIFO_EVTS_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "bhi260_common.h"

/******************************************************************************
 * Public Constants
 ******************************************************************************/
#define CMD_STATUS_REPLY_PAYLOAD_SIZE_MAX	64


/******************************************************************************
 * Public Types
 ******************************************************************************/
/**
 * @brief FIFO ID
 * 
 */
typedef enum
{
	FIFO_WAKEUP,
	FIFO_NONWAKEUP,
	FIFO_STATUS,

	FIFO_MAX
}fifoID_t;

/**
 * @brief Meta Event structure
 * 
 */
typedef struct meta_event
{
	uint8_t id;
	uint8_t values[2];
	uint64_t timestamp;
}meta_event_t;

/**
 * @brief Sensor data event structure
 * 
 */
typedef struct sensor_data_event
{
	uint8_t id;
	uint8_t data[32];
	sensor_data_type_t data_type;
	uint64_t timestamp;
}sensor_data_event_t;

/**
 * @brief Status event structure
 * 
 */
typedef struct status_event
{
	bhi260_hostCmd_id_t cmd;
	uint16_t length;
	uint8_t payload[CMD_STATUS_REPLY_PAYLOAD_SIZE_MAX];
}status_event_t;

/******************************************************************************
 * Public Functions
 ******************************************************************************/
/**
 * @brief Initializes the FIFO events machine
 * 
 */
void BHI260_FIFO_Init();

/**
 * @brief Machine that reads any existing data on Wakeup, Non-wakup, and Status 
 * FIFOs.
 * 
 * If there's some information to be retrieved, it'll push it to the
 * corresponding queue
 * 
 */
void BHI260_FIFO_Tasks();

/**
 * @brief indicates if the FIFO events machine is busy, and so using the SPI
 * host interface
 * 
 * @return true 
 * @return false 
 */
bool BHI260_FIFO_Busy();

/**
 * @brief Pulls any existing Initialized Meta Event from the queue
 * 
 * @param dst destination meta event structure where to store the
 * pulled element
 * @return true succeeded
 * @return false queue empty
 */
bool BHI260_FIFO_MetaEvent_Initialized_Pull(meta_event_t *dst);

/**
 * @brief Pulls any existing Power Mode Changed Meta Event from the queue
 * 
 * @param dst destination meta event structure where to store the
 * pulled element
 * @return true succeeded
 * @return false queue empty
 */
bool BHI260_FIFO_MetaEvent_PwrModeChanged_Pull(meta_event_t *dst);

/**
 * @brief Pulls any existing Sample Rate Changed Meta Event from the queue
 * 
 * @param dst destination meta event structure where to store the
 * pulled element
 * @return true succeeded
 * @return false queue empty
 */
bool BHI260_FIFO_MetaEvent_SmplRateChanged_Pull(meta_event_t *dst);

/**
 * @brief Pulls any existing Sensor Status Meta Event from the queue
 * 
 * @param dst destination meta event structure where to store the
 * pulled element
 * @return true succeeded
 * @return false queue empty
 */
bool BHI260_FIFO_MetaEvent_SensorStatus_Pull(meta_event_t *dst);

/**
 * @brief Pulls any existing Status Event from the Status Event queue
 * 
 * @param dst destination status event structure where to store the
 * pulled element
 * @return true succeeded
 * @return false queue empty
 */
bool BHI260_FIFO_Status_Pull(status_event_t *dst);

/**
 * @brief Pulls any existing Sensor Data Event from the Sensor Data Event queue
 * 
 * @param dst destination sensor data event structure where to store the
 * pulled element
 * @return true succeeded
 * @return false queue empty
 */
bool BHI260_FIFO_SensorData_Pull(sensor_data_event_t *dst);

#endif /* INC_IMU_BHI260_FIFO_EVTS_H_ */
