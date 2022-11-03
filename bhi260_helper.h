/*
 * bhi260_helper.h
 *
 *  Created on: Sep 15, 2021
 *      Author: btrud
 */

#ifndef INC_IMU_BHI260_HELPER_H_
#define INC_IMU_BHI260_HELPER_H_

#include <stdbool.h>
#include <stdint.h>
#include "bhi260_fifo_evts.h"


/******************************************************************************
 * Public Constants
 ******************************************************************************/

/******************************************************************************
 * Public Types
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/
/**
 * @brief Gets the quaternion from the given sensor data event pack
 * 
 * @param evt sensor data event from where to take the quaternion
 * @param q_dst pointer to the destination quaternion structure
 * @return true operation successful
 * @return false sensor data event doesn't correspond to a quaternion type
 */
bool BHI260_GetQuaternion(sensor_data_event_t evt, quaternion_t *q_dst);

/**
 * @brief Gets the 3D vector information from the given sensor data event pack
 * 
 * @param evt sensor data event from where to take the quaternion
 * @param v_dst pointer to the destination 3Dvector structure
 * @return true operation successful
 * @return false sensor data event doesn't correspond to a quaternion type
 */
bool BHI260_GetVector(sensor_data_event_t evt, vector3D_t *v_dst);

/**
 * @brief Gets the sensor name string with the given ID
 * 
 * @param sensor_id sensor id
 * @return char* sensor name string
 */
char* BHI260_HELPER_get_sensor_name(uint8_t sensor_id);

/**
 * @brief Prints the raw buffer
 * 
 * @param buf buffer to be printed
 * @param buf_len buffer len to be printed
 */
void BHI260_HELPER_print_buffer(uint8_t *buf, uint16_t buf_len);

#endif /* INC_IMU_BHI260_HELPER_H_ */
