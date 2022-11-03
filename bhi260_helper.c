/*
 * bhi260_helper.c
 *
 *  Created on: Sep 15, 2021
 *      Author: btrud
 */
#include "bhi260_helper.h"
#include "bhi260_defs.h"
#include "debug.h"

/******************************************************************************
 * Includes
 ******************************************************************************/

/******************************************************************************
 * Local Constants
 ******************************************************************************/
#define QUATERNION_SCALE    16384
#define GRAVITY_SCALE       4096

// Debug options
#define ENABLE_DEBUG        true
#define DEBUG_TAG           "BHI260_HELPER"

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

/******************************************************************************
 * Local Types
 ******************************************************************************/

/******************************************************************************
 * Local Variables
 ******************************************************************************/

/******************************************************************************
 * Local Functions
 ******************************************************************************/

/**
 * @brief converts the given array which containes twos compliment into a
 * signed int variable. The array must contain the data in LSB first format.
 * 
 * @param buf array containing te two's compliment value
 * @return int16_t 
 */
int16_t TwoComplimnet_ToInt16(uint8_t *buf)
{
    uint16_t val = (uint16_t)(*buf) | ((uint16_t)(*(buf+1))<<8);

    // if negative
    if(val & 0x8000)
        return (~((int16_t)val - 1))*(-1);
    else
        return (int16_t)val;
}

/******************************************************************************
 * Callback Functions
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/
bool BHI260_GetQuaternion(sensor_data_event_t evt, quaternion_t *q_dst)
{
    int16_t aux;

    if(evt.data_type != BHI260_SENS_DATA_TYPE_QUATERNION) return false;

    aux = TwoComplimnet_ToInt16(&evt.data[0]);
    q_dst->x = (double)aux / QUATERNION_SCALE;
    
    aux = TwoComplimnet_ToInt16(&evt.data[2]);
    q_dst->y = (double)aux / QUATERNION_SCALE;

    aux = TwoComplimnet_ToInt16(&evt.data[4]);
    q_dst->z = (double)aux / QUATERNION_SCALE;

    aux = TwoComplimnet_ToInt16(&evt.data[6]);
    q_dst->w = (double)aux / QUATERNION_SCALE;

    aux = TwoComplimnet_ToInt16(&evt.data[8]);
    q_dst->accuracy = (double)aux / QUATERNION_SCALE;

    return true;
}

bool BHI260_GetVector(sensor_data_event_t evt, vector3D_t *v_dst)
{
    int16_t aux;

    if(evt.data_type != BHI260_SENS_DATA_TYPE_3DVECTOR) return false;

    aux = TwoComplimnet_ToInt16(&evt.data[0]);
    v_dst->x = (double)aux / GRAVITY_SCALE;
    
    aux = TwoComplimnet_ToInt16(&evt.data[2]);
    v_dst->y = (double)aux / GRAVITY_SCALE;

    aux = TwoComplimnet_ToInt16(&evt.data[4]);
    v_dst->z = (double)aux / GRAVITY_SCALE;

    return true;
}

char* BHI260_HELPER_get_sensor_name(uint8_t sensor_id)
{
    char *ret;

    switch (sensor_id)
    {
        case BHY2_SENSOR_ID_ACC_PASS:
            ret = "Accelerometer passthrough";
            break;
        case BHY2_SENSOR_ID_ACC_RAW:
            ret = "Accelerometer uncalibrated";
            break;
        case BHY2_SENSOR_ID_ACC:
            ret = "Accelerometer corrected";
            break;
        case BHY2_SENSOR_ID_ACC_BIAS:
            ret = "Accelerometer offset";
            break;
        case BHY2_SENSOR_ID_ACC_WU:
            ret = "Accelerometer corrected wake up";
            break;
        case BHY2_SENSOR_ID_ACC_RAW_WU:
            ret = "Accelerometer uncalibrated wake up";
            break;
        case BHY2_SENSOR_ID_GYRO_PASS:
            ret = "Gyroscope passthrough";
            break;
        case BHY2_SENSOR_ID_GYRO_RAW:
            ret = "Gyroscope uncalibrated";
            break;
        case BHY2_SENSOR_ID_GYRO:
            ret = "Gyroscope corrected";
            break;
        case BHY2_SENSOR_ID_GYRO_BIAS:
            ret = "Gyroscope offset";
            break;
        case BHY2_SENSOR_ID_GYRO_WU:
            ret = "Gyroscope wake up";
            break;
        case BHY2_SENSOR_ID_GYRO_RAW_WU:
            ret = "Gyroscope uncalibrated wake up";
            break;
        case BHY2_SENSOR_ID_MAG_PASS:
            ret = "Magnetometer passthrough";
            break;
        case BHY2_SENSOR_ID_MAG_RAW:
            ret = "Magnetometer uncalibrated";
            break;
        case BHY2_SENSOR_ID_MAG:
            ret = "Magnetometer corrected";
            break;
        case BHY2_SENSOR_ID_MAG_BIAS:
            ret = "Magnetometer offset";
            break;
        case BHY2_SENSOR_ID_MAG_WU:
            ret = "Magnetometer wake up";
            break;
        case BHY2_SENSOR_ID_MAG_RAW_WU:
            ret = "Magnetometer uncalibrated wake up";
            break;
        case BHY2_SENSOR_ID_GRA:
            ret = "Gravity vector";
            break;
        case BHY2_SENSOR_ID_GRA_WU:
            ret = "Gravity vector wake up";
            break;
        case BHY2_SENSOR_ID_LACC:
            ret = "Linear acceleration";
            break;
        case BHY2_SENSOR_ID_LACC_WU:
            ret = "Linear acceleration wake up";
            break;
        case BHY2_SENSOR_ID_RV:
            ret = "Rotation vector";
            break;
        case BHY2_SENSOR_ID_RV_WU:
            ret = "Rotation vector wake up";
            break;
        case BHY2_SENSOR_ID_GAMERV:
            ret = "Game rotation vector";
            break;
        case BHY2_SENSOR_ID_GAMERV_WU:
            ret = "Game rotation vector wake up";
            break;
        case BHY2_SENSOR_ID_GEORV:
            ret = "Geo-magnetic rotation vector";
            break;
        case BHY2_SENSOR_ID_GEORV_WU:
            ret = "Geo-magnetic rotation vector wake up";
            break;
        case BHY2_SENSOR_ID_ORI:
            ret = "Orientation";
            break;
        case BHY2_SENSOR_ID_ORI_WU:
            ret = "Orientation wake up";
            break;
        case BHY2_SENSOR_ID_TILT_DETECTOR:
            ret = "Tilt detector";
            break;
        case BHY2_SENSOR_ID_STD:
            ret = "Step detector";
            break;
        case BHY2_SENSOR_ID_STC:
            ret = "Step counter";
            break;
        case BHY2_SENSOR_ID_STC_WU:
            ret = "Step counter wake up";
            break;
        case BHY2_SENSOR_ID_SIG:
            ret = "Significant motion";
            break;
        case BHY2_SENSOR_ID_WAKE_GESTURE:
            ret = "Wake gesture";
            break;
        case BHY2_SENSOR_ID_GLANCE_GESTURE:
            ret = "Glance gesture";
            break;
        case BHY2_SENSOR_ID_PICKUP_GESTURE:
            ret = "Pickup gesture";
            break;
        case BHY2_SENSOR_ID_AR:
            ret = "Activity recognition";
            break;
        case BHY2_SENSOR_ID_WRIST_TILT_GESTURE:
            ret = "Wrist tilt gesture";
            break;
        case BHY2_SENSOR_ID_DEVICE_ORI:
            ret = "Device orientation";
            break;
        case BHY2_SENSOR_ID_DEVICE_ORI_WU:
            ret = "Device orientation wake up";
            break;
        case BHY2_SENSOR_ID_STATIONARY_DET:
            ret = "Stationary detect";
            break;
        case BHY2_SENSOR_ID_MOTION_DET:
            ret = "Motion detect";
            break;
        case BHY2_SENSOR_ID_ACC_BIAS_WU:
            ret = "Accelerometer offset wake up";
            break;
        case BHY2_SENSOR_ID_GYRO_BIAS_WU:
            ret = "Gyroscope offset wake up";
            break;
        case BHY2_SENSOR_ID_MAG_BIAS_WU:
            ret = "Magnetometer offset wake up";
            break;
        case BHY2_SENSOR_ID_STD_WU:
            ret = "Step detector wake up";
            break;
        case BHY2_SENSOR_ID_TEMP:
            ret = "Temperature";
            break;
        case BHY2_SENSOR_ID_BARO:
            ret = "Barometer";
            break;
        case BHY2_SENSOR_ID_HUM:
            ret = "Humidity";
            break;
        case BHY2_SENSOR_ID_GAS:
            ret = "Gas";
            break;
        case BHY2_SENSOR_ID_TEMP_WU:
            ret = "Temperature wake up";
            break;
        case BHY2_SENSOR_ID_BARO_WU:
            ret = "Barometer wake up";
            break;
        case BHY2_SENSOR_ID_HUM_WU:
            ret = "Humidity wake up";
            break;
        case BHY2_SENSOR_ID_GAS_WU:
            ret = "Gas wake up";
            break;
        case BHY2_SENSOR_ID_STC_HW:
            ret = "Hardware Step counter";
            break;
        case BHY2_SENSOR_ID_STD_HW:
            ret = "Hardware Step detector";
            break;
        case BHY2_SENSOR_ID_SIG_HW:
            ret = "Hardware Significant motion";
            break;
        case BHY2_SENSOR_ID_STC_HW_WU:
            ret = "Hardware Step counter wake up";
            break;
        case BHY2_SENSOR_ID_STD_HW_WU:
            ret = "Hardware Step detector wake up";
            break;
        case BHY2_SENSOR_ID_SIG_HW_WU:
            ret = "Hardware Significant motion wake up";
            break;
        case BHY2_SENSOR_ID_ANY_MOTION:
            ret = "Any motion";
            break;
        case BHY2_SENSOR_ID_ANY_MOTION_WU:
            ret = "Any motion wake up";
            break;
        case BHY2_SENSOR_ID_EXCAMERA:
            ret = "External camera trigger";
            break;
        case BHY2_SENSOR_ID_GPS:
            ret = "GPS";
            break;
        case BHY2_SENSOR_ID_LIGHT:
            ret = "Light";
            break;
        case BHY2_SENSOR_ID_PROX:
            ret = "Proximity";
            break;
        case BHY2_SENSOR_ID_LIGHT_WU:
            ret = "Light wake up";
            break;
        case BHY2_SENSOR_ID_PROX_WU:
            ret = "Proximity wake up";
            break;
        default:
            if ((sensor_id >= BHY2_SENSOR_ID_CUSTOM_START) && (sensor_id <= BHY2_SENSOR_ID_CUSTOM_END))
            {
                ret = "Custom sensor ID ";
            }
            else
            {
                ret = "Undefined sensor ID ";
            }
    }

    return ret;
}

void BHI260_HELPER_print_buffer(uint8_t *buf, uint16_t buf_len)
{
    char line[512];	
	char number[8];
	int idx = 0;
	int cnt = buf_len / 64;
	if(cnt%64) cnt++;
	
	PRINTF("buffer [len=%d]: ",buf_len);
    PRINTF("----------------");
	while(idx <= cnt)
	{
		line[0] = '\0';
		for(int i = 0; (i < 64) && ((i+(idx*64)) < buf_len); i++)
		{
			snprintf(number,sizeof(number),"%02x ",*(buf+(idx*64)+i));
			strcat(line,number);
		}
		PRINTF("%s",line);
		idx++;
	}
	PRINTF("");
}