/*
 * bhi260ab.c
 *
 *  Created on: Aug 27, 2021
 *      Author: btrud
 */

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "bhi260ab.h"
#include "bhi260_defs.h"
#include "bhi260_firmware.h"
#include "bhi260_fifo_evts.h"
#include "bhi260_spi.h"
#include "bhi260_helper.h"
#include "tick.h"
#include "debug.h"

/******************************************************************************
 * Local Constants
 ******************************************************************************/
// Timeouts in ms
#define RESET_DELAY						100
#define BOOT_HOSTINT_TIMEOUT			1000
#define BOOT_FROM_RAM_TIMEOUT			200	
#define INITIALIZED_METAEVENT_TIMEOUT	100
#define VIRT_SENSORS_PRESENT_TIMEOUT	100	
#define ACCEL_SELFTEST_TIMEOUT			1000	
#define GYRO_SELFTEST_TIMEOUT			1000
#define MAG_SELFTEST_TIMEOUT			1000
#define VIRTUAL_SENSOR_CFG_TIMEOUT		500
#define ORIENTATION_MATRIX_READ_TIMEOUT	500

// Debug options
#define ENABLE_DEBUG        true
#define DEBUG_TAG           "BHI260"

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

// Defines if the axes remapping will be performed from this micro (host) or if
// the reampping is already embedded on the BHI firmware
#define AXES_REMAPING_FROM_HOST		false

/******************************************************************************
 * Local Types
 ******************************************************************************/
/**
 * Local states
 */
typedef enum
{
	STATE_INIT,
	STATE_SYSTEM_SETUP,
	STATE_SENSOR_CONFIG,
	STATE_RUNNING,
	STATE_IDLE,
	STATE_FAILURE
}states_t;

/**
 * @brief Virtual sensor type
 * 
 */
typedef struct virtual_sensor
{
    uint8_t id;
    uint8_t sample_rate;
    bool enabled;
    sensor_data_type_t data_type;
    union
    {
		quaternion_t q;
		vector3D_t v;
		euler_t e;
    }data;
    sensor_accuracy_status_t accu_status;
}virtual_sensor_t;

#if AXES_REMAPING_FROM_HOST == true
/**
 * @brief Orientation matrix type
 * 
 */
typedef union orientation_matrix
{
	struct{
		int Xx, Xy, Xz;
		int Yx, Yy, Yz;
		int Zx, Zy, Zz;
	};
	int buf[9];
} orientation_matrix_t;
#endif

/******************************************************************************
 * Local Variables
 ******************************************************************************/

/**
 * @brief List of sample rate values for all the virtual sensors enabled
 * 
 */
static const float sample_rate_config_list[VS_MAX] = 
{
	50.00,
	50.00,
	50.00,
	50.00,
	50.00
};

/**
 * Local data continer
 */
static struct local_data
{
	states_t state;			// Local state variable
	uint32_t timer;			// Local timer
	bool pause_fifo;			// Flag for holding the FIFO and use the host SPI
	// Array with all the VS used in the application
	virtual_sensor_t vs_list[VS_MAX];
}this;

#if AXES_REMAPING_FROM_HOST == true
/**
 * @brief Orientation Matrix for the BHI260 chip
 * 
 */
static const orientation_matrix_t bhi260ab_orientation_matrix =
{
	.Xx = 1,
	.Xy = 0,
	.Xz = 0,
	.Yx = 0,
	.Yy = 1,
	.Yz = 0,
	.Zx = 0,
	.Zy = 0,
	.Zz = 1
};

/**
 * @brief Orientation Matrix for the BMM150 chip
 * 
 */
static const orientation_matrix_t bmm150_orientation_matrix =
{
	.Xx = 1,
	.Xy = 0,	
	.Xz = 0,
	.Yx = 0,
	.Yy = -1,
	.Yz = 0,
	.Zx = 0,
	.Zy = 0,
	.Zz = -1
};
#endif

/******************************************************************************
 * Local Functions
 ******************************************************************************/
// standard timer functions
static void SetTimer(uint32_t ms) {this.timer = ms;}
static bool Timeout() {return this.timer == 0;}

/**
 * BHI260 startup machine
 *
 * @return bhi260_ret_t status
 */
static bhi260_ret_t Startup()
{
	bhi260_ret_t ret = BHI260_RET_WORKING;
	static bhi260_ret_t ret_exit = BHI260_RET_FAILED;
	static uint8_t TXbyte[3];
	static uint8_t RXbyte[3];

	// Register operations
	static reg_read_or_write_t wReg, rReg;

	// Machine states
	static enum
	{
		STARTUP_STATE_INIT,
		STARTUP_STATE_WAIT_RESET,
		STARTUP_STATE_POLL_BOOT_STATE,
		STARTUP_STATE_FUSER2_PROD_ID,
		STARTUP_STATE_FUSER2_REV_ID,
		STARTUP_STATE_FUSER2_ROM_VERSION,
		STARTUP_STATE_UPLOAD_TO_PROG_RAM,
		STARTUP_STATE_FIRMWARE_VALIDATED,
		STARTUP_STATE_BOOT_FROM_RAM,
		STARTUP_STATE_WAIT_BOOT,
		STARTUP_STATE_FIRMWARE_ERROR_CHECK,
		STARTUP_STATE_EXIT,
	}startupState = STARTUP_STATE_INIT;

	switch(startupState)
	{
		case STARTUP_STATE_INIT:
			if(BHI260_SPI_Take())
			{
				TXbyte[1] = BHY2_REQUEST_RESET;
				if(RegWrite(&wReg,BHY2_REG_RESET_REQ,TXbyte,1,true))
				{
					SetTimer(RESET_DELAY);
					startupState = STARTUP_STATE_WAIT_RESET;
					DEBUG_PRINTF("Reseting device...");
				}
				else
				{
					DEBUG_PRINTF("Failed to send the reset command");
					ret_exit = BHI260_RET_FAILED;
					startupState = STARTUP_STATE_EXIT;
				}
			}			
			break;
		
		case STARTUP_STATE_WAIT_RESET:
			if(Timeout())
			{
				DEBUG_PRINTF("Reset delay elapsed");
				RegRead(&rReg,BHY2_REG_BOOT_STATUS,RXbyte,1,true);
				SetTimer(BOOT_HOSTINT_TIMEOUT);
				startupState = STARTUP_STATE_POLL_BOOT_STATE;
			}
			break;
		
		case STARTUP_STATE_POLL_BOOT_STATE:
			if(Timeout())
			{
				DEBUG_PRINTF("Host Interface Interface Ready flag timeout");
				DEBUG_PRINTF("Boot status reg: 0x%02x",RXbyte[1]);
				startupState = STARTUP_STATE_EXIT;
				ret_exit = BHI260_RET_FAILED;
			}
			else
			{
				// Check if the transaction has finished
				if(RegProcess_Done(&rReg))
				{
					// Check the boot status for the Host Interface bit
					if(RXbyte[1] & BHY2_BST_HOST_INTERFACE_READY)
					{
						DEBUG_PRINTF("Host interface boot status ready");
						DEBUG_PRINTF("Boot Status: 0x%02x",RXbyte[1]);

						// Read the Fuser2 product identifier
						RegRead(&rReg,BHY2_REG_PRODUCT_ID,RXbyte,1,true);
						startupState = STARTUP_STATE_FUSER2_PROD_ID;
					}
					else
					{
						RegRead(&rReg,BHY2_REG_BOOT_STATUS,RXbyte,1,true);
					}
				}
			}
			break;
		
		case STARTUP_STATE_FUSER2_PROD_ID:
			if(RegProcess_Done(&rReg))
			{
				DEBUG_PRINTF("Fuser2 prod ID: 0x%02x (%s)",
					RXbyte[1],
					(RXbyte[1] == BHY2_PRODUCT_ID)? "BHI260AB" : "not recognized");

				// Read the Fuser2 product identifier
				RegRead(&rReg,BHY2_REG_REVISION_ID,RXbyte,1,true);
				startupState = STARTUP_STATE_FUSER2_REV_ID;
			}
			break;
		
		case STARTUP_STATE_FUSER2_REV_ID:
			if(RegProcess_Done(&rReg))
			{
				DEBUG_PRINTF("Fuser2 rev ID: 0x%02x", RXbyte[1]);
				
				// Read the Fuser2 product identifier
				RegRead(&rReg,BHY2_REG_ROM_VERSION_0,RXbyte,2,true);
				startupState = STARTUP_STATE_FUSER2_ROM_VERSION;
			}
			break;
		
		case STARTUP_STATE_FUSER2_ROM_VERSION:
			if(RegProcess_Done(&rReg))
			{
				DEBUG_PRINTF("ROM version: 0x%04x", (uint16_t)((RXbyte[2] << 8) | RXbyte[1]));
				DEBUG_PRINTF("Uploading firmware");
				// Read the Fuser2 product identifier				
				startupState = STARTUP_STATE_UPLOAD_TO_PROG_RAM;
			}
			break;
		
		case STARTUP_STATE_UPLOAD_TO_PROG_RAM:
			switch(SendCmd(BHY2_CMD_UPLOAD_TO_PROGRAM_RAM,(uint8_t *)bhy2_firmware_image,sizeof(bhy2_firmware_image)/4,sizeof(uint32_t)))
			{
				case BHI260_RET_OK:
					DEBUG_PRINTF("Firmware uploaded");
					RegRead(&rReg,BHY2_REG_BOOT_STATUS,RXbyte,1,true);
					startupState = STARTUP_STATE_FIRMWARE_VALIDATED;
					break;
				
				case BHI260_RET_WORKING: break;

				default:
				case BHI260_RET_FAILED:
					DEBUG_PRINTF("Failed to upload firmware");
					startupState = STARTUP_STATE_EXIT;
					ret_exit = BHI260_RET_FAILED;
					break;
			}
			break;
		
		case STARTUP_STATE_FIRMWARE_VALIDATED:
			if(RegProcess_Done(&rReg))
			{
				if(RXbyte[1] & BHY2_BST_HOST_FW_VERIFY_DONE)
				{
					DEBUG_PRINTF("Boot Status: 0x%02x",RXbyte[1]);
					DEBUG_PRINTF("Firmware uploaded correctly");
					DEBUG_PRINTF("Booting firmware from RAM");

					startupState = STARTUP_STATE_BOOT_FROM_RAM;

				}
				else if(RXbyte[1] & BHY2_BST_HOST_FW_VERIFY_ERROR)
				{
					DEBUG_PRINTF("Boot Status: 0x%02x",RXbyte[1]);
					DEBUG_PRINTF("Firmware validation error");
					startupState = STARTUP_STATE_EXIT;
					ret_exit = BHI260_RET_FAILED;
				}
				else
				{
					RegRead(&rReg,BHY2_REG_BOOT_STATUS,RXbyte,1,true);
				}
			}
			break;
		
		case STARTUP_STATE_BOOT_FROM_RAM:
			switch(SendCmd(BHY2_CMD_BOOT_PROGRAM_RAM,NULL,0,0))
			{
				case BHI260_RET_OK:
					DEBUG_PRINTF("Command sent OK");
					SetTimer(BOOT_FROM_RAM_TIMEOUT);
					RegRead(&rReg,BHY2_REG_BOOT_STATUS,RXbyte,1,true);
					startupState = STARTUP_STATE_WAIT_BOOT;
					break;
				
				case BHI260_RET_WORKING: break;

				default:
				case BHI260_RET_FAILED:
					DEBUG_PRINTF("Failed to send the Boot From RAM command");
					startupState = STARTUP_STATE_EXIT;
					ret_exit = BHI260_RET_FAILED;
					break;
			}
			break;
		
		case STARTUP_STATE_WAIT_BOOT:
			if(Timeout())
			{
				DEBUG_PRINTF("Boot from RAM timeout");
				startupState = STARTUP_STATE_EXIT;
				ret_exit = BHI260_RET_FAILED;
			}
			else
			{
				if(RegProcess_Done(&rReg))
				{
					if(RXbyte[1] & BHY2_BST_HOST_INTERFACE_READY)
					{
						DEBUG_PRINTF("Boot Status: 0x%02x",RXbyte[1]);
						DEBUG_PRINTF("Boot from RAM ok!!!");

						RegRead(&rReg,BHY2_REG_ERROR_VALUE,RXbyte,1,true);
						startupState = STARTUP_STATE_FIRMWARE_ERROR_CHECK;
						
					}
					else
					{
						RegRead(&rReg,BHY2_REG_BOOT_STATUS,RXbyte,1,true);
					}
				}
			}
			break;
		
		case STARTUP_STATE_FIRMWARE_ERROR_CHECK:
			if(RegProcess_Done(&rReg))
			{
				DEBUG_PRINTF("Boot Status: 0x%02x",RXbyte[1]);
				
				if(RXbyte[1] == 0)
				{					
					DEBUG_PRINTF("No errors");			
					startupState = STARTUP_STATE_EXIT;
					ret_exit = BHI260_RET_OK;		
				}
				else
				{
					DEBUG_PRINTF("Firmware errors occured");
					startupState = STARTUP_STATE_EXIT;
					ret_exit = BHI260_RET_FAILED;
				}
			}
			break;
		
		case STARTUP_STATE_EXIT:
			if(BHI260_SPI_Give())
			{
				DEBUG_PRINTF("SPI was given, returning to init state");
				ret = ret_exit;
				startupState = STARTUP_STATE_INIT;
			}
			break;

		default: break;
	}

	return ret;
}

/**
 * @brief Performs a Config Sensor command with the given parameters
 * 
 * @param sensorID virtual sensor ID to configure
 * @param sample_rate sample rate in Hz. If 0 sensor is disabled
 * @param latency latency for writing the data into the FIFO
 * @return bhi260_ret_t 
 */
static bhi260_ret_t SensorConfig(uint8_t sensorID, float sample_rate, uint32_t latency)
{
	bhi260_ret_t ret = BHI260_RET_WORKING;
	static uint8_t buf[8];

	static enum
	{
		SENSCONFIG_STATE_INIT,
		SENSCONFIG_STATE_CMD
	}sensCfgSate = SENSCONFIG_STATE_INIT;

	switch(sensCfgSate)
	{
		case SENSCONFIG_STATE_INIT:
			// Copy the parameters into the local buffer
			buf[0] = sensorID;
			memcpy(&buf[1],&sample_rate,4);
			buf[5] = latency & 0x000000ff;
			buf[6] = (latency & 0x0000ff00)>>8;
			buf[7] = (latency & 0x00ff0000)>>16;
			sensCfgSate = SENSCONFIG_STATE_CMD;
			break;
		
		case SENSCONFIG_STATE_CMD:
			ret = SendCmd(BHY2_CMD_CONFIG_SENSOR,buf,8,1);
			
			if(ret != BHI260_RET_WORKING)
			{
				sensCfgSate = SENSCONFIG_STATE_INIT;
			}
			break;

		default:
			ret = BHI260_RET_FAILED;
			sensCfgSate = SENSCONFIG_STATE_INIT;
			break;
	}
	return ret;
}

#if AXES_REMAPING_FROM_HOST == true
/**
 * @brief Set the Orientation Matrix into the BHI260 parameters
 * 
 * @param m 
 * @return bhi260_ret_t 
 */
static bhi260_ret_t SetOrientationMatrix(const orientation_matrix_t *m, uint8_t phy_sens_id)
{
	bhi260_ret_t ret = BHI260_RET_WORKING;

	static uint8_t buf[16]; // double of the required size, just in case
	uint8_t value;
	int i;

	static enum
	{
		omSTATE_INIT = 0,
		omSTATE_SEND_CMD
	}omState = omSTATE_INIT;

	switch(omState)
	{
		case omSTATE_INIT:
			memset(buf,0,sizeof(buf));			
			for(i = 0; i < 9; i++)
			{
				switch(m->buf[i])
				{
					case 0: value = 0x0; break;
					case 1: value = 0x1; break;
					case -1: value = 0xf; break;
					default: return BHI260_RET_FAILED; break;
				}
				
				// Is even?
				if(i%2)
				{
					buf[i/2] |= (value<<4);
				}
				else
				{
					buf[i/2] |= value;
				}
			}
			omState = omSTATE_SEND_CMD;
			break;
		
		case omSTATE_SEND_CMD:
			ret = WriteParameter(BHY2_PARAM_PHYSICAL_SENSOR_BASE+phy_sens_id,8,buf);
			switch(ret)
			{
				case BHI260_RET_WORKING: break;

				default: 
					omState = omSTATE_INIT;
					break;
			}
			break;
		
		default: ret = BHI260_RET_FAILED; break;
	}

	return ret;
}
#endif

/**
 * @brief Checks that the BSX firmware is running and sets
 * system parameters
 * 
 * @return bhi260_ret_t 
 */
static bhi260_ret_t SystemSetup()
{
	bhi260_ret_t ret = BHI260_RET_WORKING;
	static bhi260_ret_t ret_exit = BHI260_RET_FAILED;
	static uint32_t next_timeout = 0;
	static meta_event_t metaEvt;
	static status_event_t status;
	static uint8_t initEvtCnt;
	static uint8_t buf[8];
	static char sensor_name[128];
	int i, k;
	static int idx = 0;
	static int next_state = 0;

	// Local states
	static enum
	{
		SENSORSTATE_INIT,
		SENSORSTATE_WAIT_INIT_METAEVENT,
		SENSORSTATE_VIRTUAL_SENSORS_PRESENT_CMD,
		SENSORSTATE_VIRTUAL_SENSORS_PRESENT_REPLY,
		SENSORSTATE_ACCEL_SELFTEST,
		SENSORSTATE_ACCEL_SELFTEST_RESULTS,
		SENSORSTATE_GYRO_SELFTEST,
		SENSORSTATE_GYRO_SELFTEST_RESULTS,
		SENSORSTATE_MAG_SELFTEST,
		SENSORSTATE_MAG_SELFTEST_RESULTS,
#if AXES_REMAPING_FROM_HOST == true
		SENSORSTATE_ACCEL_AXES_REMAPPING,
		SENSORSTATE_GYRO_AXES_REMAPPING,
		SENSORSTATE_MAG_AXES_REMAPPING,
#endif
		SENSORSTATE_ACCEL_AXES_CHECK_CMD,
		SENSORSTATE_ACCEL_AXES_CHECK_EVENT,
		SENSORSTATE_GYRO_AXES_CHECK_CMD,
		SENSORSTATE_GYRO_AXES_CHECK_EVENT,
		SENSORSTATE_MAG_AXES_CHECK_CMD,
		SENSORSTATE_MAG_AXES_CHECK_EVENT,
		SENSORSTATE_CONFIG_VS,
		SENSORTATE_CONFIG_VS_STATUS_CHECK,
		SENSORSTATE_WAIT_SPI_TAKE,
		SENSORSTATE_WAIT_SPI_GIVE,
		SENSORSTATE_EXIT,
	}sensorSrtate = SENSORSTATE_INIT;

	switch (sensorSrtate)
	{
		case SENSORSTATE_INIT:
			initEvtCnt = 0;
			SetTimer(INITIALIZED_METAEVENT_TIMEOUT);
			sensorSrtate = SENSORSTATE_WAIT_INIT_METAEVENT;			
			break;
		
		
		case SENSORSTATE_WAIT_INIT_METAEVENT:
			if(Timeout())
			{
				ret = BHI260_RET_FAILED;
				sensorSrtate = SENSORSTATE_INIT;
			}
			else
			{
				// Attempt to take an element from Meta Event queue
				if(BHI260_FIFO_MetaEvent_Initialized_Pull(&metaEvt))
				{
					// If we received the two initialization events in both FIFOs
					if(++initEvtCnt == 2)
					{
						next_state = SENSORSTATE_VIRTUAL_SENSORS_PRESENT_CMD;
						sensorSrtate = SENSORSTATE_WAIT_SPI_TAKE;
					}				
				}			
			}
			break;

		case SENSORSTATE_VIRTUAL_SENSORS_PRESENT_CMD:
			switch(ReadParameter(BHY2_PARAM_SYS_VIRT_SENSOR_PRESENT))
			{
				case BHI260_RET_WORKING: break;

				case BHI260_RET_OK:
					next_timeout = VIRT_SENSORS_PRESENT_TIMEOUT;
					next_state = SENSORSTATE_VIRTUAL_SENSORS_PRESENT_REPLY;
					sensorSrtate = SENSORSTATE_WAIT_SPI_GIVE;
					break;
				
				default:
				case BHI260_RET_IDLE:
				case BHI260_RET_FAILED:
					DEBUG_PRINTF("Failed to execute the ReadParameter() function");
					ret_exit = BHI260_RET_FAILED;
					sensorSrtate = SENSORSTATE_EXIT;
					break;
			}
			break;
		
		case SENSORSTATE_VIRTUAL_SENSORS_PRESENT_REPLY:
			if(Timeout())
			{
				DEBUG_PRINTF("Virtual Sensors Present timeout");
				ret_exit = BHI260_RET_FAILED;
				sensorSrtate = SENSORSTATE_EXIT;
			}
			else
			{
				if(BHI260_FIFO_Status_Pull(&status))
				{					
					DEBUG_PRINTF("Virtual sensors present: ");

					for(i = 0; i < status.length; i++)
					{
						for(k = 0; k < 8; k++)
						{
							if(status.payload[i] & (1<<k))
							{
								strcpy(sensor_name, BHI260_HELPER_get_sensor_name((i*8)+k));
								PRINTF("\t* %s",sensor_name);
							}								
						}
					}
					PRINTF("");

					// Prepare the data buf for the Accel self test command
					buf[0] = BHY2_PHYS_SENS_ID_ACCEL;

					sensorSrtate = SENSORSTATE_WAIT_SPI_TAKE;
					next_state = SENSORSTATE_ACCEL_SELFTEST;
				}
			}
			break;
		
		case SENSORSTATE_ACCEL_SELFTEST:
			switch (SendCmd(BHY2_CMD_REQ_SELF_TEST,buf,4,1))
			{
				case BHI260_RET_WORKING: break;
				
				case BHI260_RET_OK:
					next_timeout = ACCEL_SELFTEST_TIMEOUT;
					next_state = SENSORSTATE_ACCEL_SELFTEST_RESULTS;
					sensorSrtate = SENSORSTATE_WAIT_SPI_GIVE;
					break;				
				
				default:
				case BHI260_RET_IDLE:
				case BHI260_RET_FAILED:
					DEBUG_PRINTF("Failed to execute the ReadParameter() function");
					ret_exit = BHI260_RET_FAILED;
					sensorSrtate = SENSORSTATE_EXIT;
					break;
			}
			break;
		
		case SENSORSTATE_ACCEL_SELFTEST_RESULTS:
			if(Timeout())
			{
				DEBUG_PRINTF("Accel selftest timeout");
				ret = BHI260_RET_FAILED;
				sensorSrtate = SENSORSTATE_INIT;
			}
			else
			{
				if(BHI260_FIFO_Status_Pull(&status))
				{
					// The reply is for this command?
					if(status.cmd == BHY2_CMD_REQ_SELF_TEST_REPLY_ID)
					{
						// The reply is for the selected physical sensor?
						if(status.payload[0] == BHY2_PHYS_SENS_ID_ACCEL)
						{
							// 0 means that the self test result is ok
							if(status.payload[1] == BHY2_SELF_TEST_OK)
							{
								// Prepare the data buf for the gyro self test command
								DEBUG_PRINTF("Accel self test passed");
								buf[0] = BHY2_PHYS_SENS_ID_GYRO;
								sensorSrtate = SENSORSTATE_WAIT_SPI_TAKE;
								next_state = SENSORSTATE_GYRO_SELFTEST;
							}
							else
							{
								DEBUG_PRINTF("Accel selftest failed");
								ret = BHI260_RET_FAILED;
								sensorSrtate = SENSORSTATE_INIT;
							}
						}
					}
				}
			}			
			break;
		
		case SENSORSTATE_GYRO_SELFTEST:
			switch (SendCmd(BHY2_CMD_REQ_SELF_TEST,buf,4,1))
			{
				case BHI260_RET_WORKING: break;
				
				case BHI260_RET_OK:
					next_timeout = GYRO_SELFTEST_TIMEOUT;
					next_state = SENSORSTATE_GYRO_SELFTEST_RESULTS;
					sensorSrtate = SENSORSTATE_WAIT_SPI_GIVE;
					break;				
				
				default:
				case BHI260_RET_IDLE:
				case BHI260_RET_FAILED:
					DEBUG_PRINTF("Failed to execute the ReadParameter() function");
					ret_exit = BHI260_RET_FAILED;
					sensorSrtate = SENSORSTATE_EXIT;
					break;
			}
			break;
		
		case SENSORSTATE_GYRO_SELFTEST_RESULTS:
			if(Timeout())
			{
				DEBUG_PRINTF("Gyro selftest timeout");
				ret = BHI260_RET_FAILED;
				sensorSrtate = SENSORSTATE_INIT;
			}
			else
			{
				if(BHI260_FIFO_Status_Pull(&status))
				{
					// The reply is for this command?
					if(status.cmd == BHY2_CMD_REQ_SELF_TEST_REPLY_ID)
					{
						// The reply is for the selected physical sensor?
						if(status.payload[0] == BHY2_PHYS_SENS_ID_GYRO)
						{
							// 0 means that the self test result is ok
							if(status.payload[1] == BHY2_SELF_TEST_OK)
							{
								// Prepare the data buf for the gyro self test command
								DEBUG_PRINTF("Gyro self test passed");
								buf[0] = BHY2_PHYS_SENS_ID_MAG;
								sensorSrtate = SENSORSTATE_WAIT_SPI_TAKE;
								next_state = SENSORSTATE_MAG_SELFTEST;
							}
							else
							{
								DEBUG_PRINTF("Gyro selftest failed");
								ret = BHI260_RET_FAILED;
								sensorSrtate = SENSORSTATE_INIT;
							}
						}
					}
				}
			}			
			break;

		case SENSORSTATE_MAG_SELFTEST:
			switch (SendCmd(BHY2_CMD_REQ_SELF_TEST,buf,4,1))
			{
				case BHI260_RET_WORKING: break;
				
				case BHI260_RET_OK:
					next_timeout = MAG_SELFTEST_TIMEOUT;
					next_state = SENSORSTATE_MAG_SELFTEST_RESULTS;
					sensorSrtate = SENSORSTATE_WAIT_SPI_GIVE;
					break;				
				
				default:
				case BHI260_RET_IDLE:
				case BHI260_RET_FAILED:
					DEBUG_PRINTF("Failed to execute the ReadParameter() function");
					ret_exit = BHI260_RET_FAILED;
					sensorSrtate = SENSORSTATE_EXIT;
					break;
			}
			break;
		
		case SENSORSTATE_MAG_SELFTEST_RESULTS:
			if(Timeout())
			{
				DEBUG_PRINTF("Magnetometer selftest timeout");
				ret = BHI260_RET_FAILED;
				sensorSrtate = SENSORSTATE_INIT;
			}
			else
			{
				if(BHI260_FIFO_Status_Pull(&status))
				{
					// The reply is for this command?
					if(status.cmd == BHY2_CMD_REQ_SELF_TEST_REPLY_ID)
					{
						// The reply is for the selected physical sensor?
						if(status.payload[0] == BHY2_PHYS_SENS_ID_MAG)
						{
							// 0 means that the self test result is ok
							if(status.payload[1] == BHY2_SELF_TEST_OK)
							{
								DEBUG_PRINTF("Magnetometer self test passed");
								sensorSrtate = SENSORSTATE_WAIT_SPI_TAKE;
								next_state = SENSORSTATE_ACCEL_AXES_CHECK_CMD;
							}
							else
							{
								DEBUG_PRINTF("Gyro selftest failed");
								ret = BHI260_RET_FAILED;
								sensorSrtate = SENSORSTATE_INIT;
							}
						}
					}
				}
			}			
			break;

#if AXES_REMAPING_FROM_HOST == true

		case SENSORSTATE_ACCEL_AXES_REMAPPING:
			switch(SetOrientationMatrix(&bhi260ab_orientation_matrix,BHY2_PHYS_SENS_ID_ACCEL))
			{
				case BHI260_RET_WORKING: break;

				case BHI260_RET_OK:
					DEBUG_PRINTF("Accelerometer orientation matrix set");
					sensorSrtate = SENSORSTATE_GYRO_AXES_REMAPPING;
					break;
				
				default:
				case BHI260_RET_IDLE:
				case BHI260_RET_FAILED:
					DEBUG_PRINTF("Unable to send cmd for setting orientation matrix for accel");
					ret_exit = BHI260_RET_FAILED;
					sensorSrtate = SENSORSTATE_EXIT;
					break;
			}
			break;
		
		case SENSORSTATE_GYRO_AXES_REMAPPING:
			switch(SetOrientationMatrix(&bhi260ab_orientation_matrix,BHY2_PHYS_SENS_ID_GYRO))
			{
				case BHI260_RET_WORKING: break;

				case BHI260_RET_OK:
					DEBUG_PRINTF("Gyro orientation matrix set");
					sensorSrtate = SENSORSTATE_MAG_AXES_REMAPPING;
					break;
				
				default:
				case BHI260_RET_IDLE:
				case BHI260_RET_FAILED:
					DEBUG_PRINTF("Unable to send cmd for setting orientation matrix for gyro");
					ret_exit = BHI260_RET_FAILED;
					sensorSrtate = SENSORSTATE_EXIT;
					break;
			}
			break;
		
		case SENSORSTATE_MAG_AXES_REMAPPING:
			switch(SetOrientationMatrix(&bmm150_orientation_matrix,BHY2_PHYS_SENS_ID_MAG))
			{
				case BHI260_RET_WORKING: break;

				case BHI260_RET_OK:
					DEBUG_PRINTF("Mag orientation matrix set");					
					sensorSrtate = SENSORSTATE_ACCEL_AXES_CHECK_CMD;
					break;
				
				default:
				case BHI260_RET_IDLE:
				case BHI260_RET_FAILED:
					DEBUG_PRINTF("Unable to send cmd for setting orientation matrix for mag");
					ret_exit = BHI260_RET_FAILED;
					sensorSrtate = SENSORSTATE_EXIT;
					break;
			}
			break;

#endif

		case SENSORSTATE_ACCEL_AXES_CHECK_CMD:
			switch(ReadParameter(BHY2_PARAM_PHYSICAL_SENSOR_BASE+BHY2_PHYS_SENS_ID_ACCEL))
			{
				case BHI260_RET_WORKING: break;

				case BHI260_RET_OK:
					DEBUG_PRINTF("Cmd for reading accel orientation matrix sent ok");
					next_timeout = ORIENTATION_MATRIX_READ_TIMEOUT;
					next_state = SENSORSTATE_ACCEL_AXES_CHECK_EVENT;
					sensorSrtate = SENSORSTATE_WAIT_SPI_GIVE;
					break;

				default:
				case BHI260_RET_IDLE:
				case BHI260_RET_FAILED:
					DEBUG_PRINTF("Unable to send cmd for reading orientation matrix of accel");
					ret_exit = BHI260_RET_FAILED;
					sensorSrtate = SENSORSTATE_EXIT;
					break;
			}
			break;

		case SENSORSTATE_ACCEL_AXES_CHECK_EVENT:
			if(Timeout())
			{
				DEBUG_PRINTF("Orientation matrix read event timeout");
				ret = BHI260_RET_FAILED;
				sensorSrtate = SENSORSTATE_INIT;
			}
			else
			{
				if(BHI260_FIFO_Status_Pull(&status))
				{
					BHI260_HELPER_print_buffer(status.payload,status.length);

					// Prepare for the next command
					idx = 0;
					strcpy(sensor_name,BHI260_HELPER_get_sensor_name(this.vs_list[idx].id));

					sensorSrtate = SENSORSTATE_WAIT_SPI_TAKE;
					next_state = SENSORSTATE_GYRO_AXES_CHECK_CMD;				
				}
			}
			break;

		case SENSORSTATE_GYRO_AXES_CHECK_CMD:
			switch(ReadParameter(BHY2_PARAM_PHYSICAL_SENSOR_BASE+BHY2_PHYS_SENS_ID_GYRO))
			{
				case BHI260_RET_WORKING: break;

				case BHI260_RET_OK:
					DEBUG_PRINTF("Cmd for reading gyro orientation matrix sent ok");
					next_timeout = ORIENTATION_MATRIX_READ_TIMEOUT;
					next_state = SENSORSTATE_GYRO_AXES_CHECK_EVENT;
					sensorSrtate = SENSORSTATE_WAIT_SPI_GIVE;
					break;

				default:
				case BHI260_RET_IDLE:
				case BHI260_RET_FAILED:
					DEBUG_PRINTF("Unable to send cmd for reading orientation matrix of gyro");
					ret_exit = BHI260_RET_FAILED;
					sensorSrtate = SENSORSTATE_EXIT;
					break;
			}
			break;	
		
		case SENSORSTATE_GYRO_AXES_CHECK_EVENT:
			if(Timeout())
			{
				DEBUG_PRINTF("Orientation matrix read event timeout");
				ret = BHI260_RET_FAILED;
				sensorSrtate = SENSORSTATE_INIT;
			}
			else
			{
				if(BHI260_FIFO_Status_Pull(&status))
				{
					BHI260_HELPER_print_buffer(status.payload,status.length);

					// Prepare for the next command
					idx = 0;
					strcpy(sensor_name,BHI260_HELPER_get_sensor_name(this.vs_list[idx].id));

					sensorSrtate = SENSORSTATE_WAIT_SPI_TAKE;
					next_state = SENSORSTATE_MAG_AXES_CHECK_CMD;				
				}
			}
			break;

		case SENSORSTATE_MAG_AXES_CHECK_CMD:
			switch(ReadParameter(BHY2_PARAM_PHYSICAL_SENSOR_BASE+BHY2_PHYS_SENS_ID_MAG))
			{
				case BHI260_RET_WORKING: break;

				case BHI260_RET_OK:
					DEBUG_PRINTF("Cmd for reading magnet orientation matrix sent ok");
					next_timeout = ORIENTATION_MATRIX_READ_TIMEOUT;
					next_state = SENSORSTATE_MAG_AXES_CHECK_EVENT;
					sensorSrtate = SENSORSTATE_WAIT_SPI_GIVE;
					break;

				default:
				case BHI260_RET_IDLE:
				case BHI260_RET_FAILED:
					DEBUG_PRINTF("Unable to send cmd for reading orientation matrix of magnet");
					ret_exit = BHI260_RET_FAILED;
					sensorSrtate = SENSORSTATE_EXIT;
					break;
			}
			break;

		case SENSORSTATE_MAG_AXES_CHECK_EVENT:
			if(Timeout())
			{
				DEBUG_PRINTF("Orientation matrix read event timeout");
				ret = BHI260_RET_FAILED;
				sensorSrtate = SENSORSTATE_INIT;
			}
			else
			{
				if(BHI260_FIFO_Status_Pull(&status))
				{
					BHI260_HELPER_print_buffer(status.payload,status.length);

					// Prepare for the next command
					idx = 0;
					strcpy(sensor_name,BHI260_HELPER_get_sensor_name(this.vs_list[idx].id));

					sensorSrtate = SENSORSTATE_WAIT_SPI_TAKE;
					next_state = SENSORSTATE_CONFIG_VS;				
				}
			}
			break;
		
		case SENSORSTATE_CONFIG_VS:
			switch (SensorConfig(this.vs_list[idx].id,sample_rate_config_list[idx],0))
			{
				case BHI260_RET_WORKING: break;
				
				case BHI260_RET_OK:
					DEBUG_PRINTF("%s setup cmd sent ok (ODR: %.2f)", sensor_name, sample_rate_config_list[idx]);
					next_timeout = VIRTUAL_SENSOR_CFG_TIMEOUT;
					next_state = SENSORTATE_CONFIG_VS_STATUS_CHECK;
					sensorSrtate = SENSORSTATE_WAIT_SPI_GIVE;
					break;				
				
				default:
				case BHI260_RET_IDLE:
				case BHI260_RET_FAILED:
					DEBUG_PRINTF("%s setup cmd failed to execute", sensor_name);
					ret_exit = BHI260_RET_FAILED;
					sensorSrtate = SENSORSTATE_EXIT;
					break;
			}
			break;

		case SENSORTATE_CONFIG_VS_STATUS_CHECK:
			if(Timeout())
			{
				DEBUG_PRINTF("%s configuration timeout", sensor_name);
				ret = BHI260_RET_FAILED;
				sensorSrtate = SENSORSTATE_INIT;
			}
			else
			{
				if(BHI260_FIFO_MetaEvent_PwrModeChanged_Pull(&metaEvt))
				{
					if(metaEvt.values[0] == this.vs_list[idx].id)
					{
						this.vs_list[idx].enabled = true;
					}
				}

				if(BHI260_FIFO_MetaEvent_SmplRateChanged_Pull(&metaEvt))
				{
					if(metaEvt.values[0] == this.vs_list[idx].id)
					{
						this.vs_list[idx].sample_rate = metaEvt.values[1];
					}
				}

				if(this.vs_list[idx].enabled && this.vs_list[idx].sample_rate)
				{
					DEBUG_PRINTF("%s configured successfuly",
							sensor_name);
					
					// Increment the index and check if all VS were configured
					idx++;
					if(idx >= VS_MAX)
					{
						DEBUG_PRINTF("All virtual sensors configured");
						ret = BHI260_RET_OK;
						sensorSrtate = SENSORSTATE_INIT;
					}
					else
					{
						strcpy(sensor_name,BHI260_HELPER_get_sensor_name(this.vs_list[idx].id));
						sensorSrtate = SENSORSTATE_WAIT_SPI_TAKE;
						next_state = SENSORSTATE_CONFIG_VS;
					}					
				}				
			}
			break;

		case SENSORSTATE_WAIT_SPI_TAKE:
			if(BHI260_SPI_Take())
			{
				DEBUG_PRINTF("SPI taken");
				sensorSrtate = next_state;
			}
			break;

		case SENSORSTATE_WAIT_SPI_GIVE:
			if(BHI260_SPI_Give())
			{
				sensorSrtate = next_state;
				SetTimer(next_timeout);
			}
			break;

		case SENSORSTATE_EXIT:
			if(BHI260_SPI_Give())
			{
				ret = ret_exit;
				sensorSrtate = SENSORSTATE_INIT;
			}
			break;

		default:
			ret = BHI260_RET_FAILED;
			break;
	}

	return ret;
}

/**
 * @brief Parses the given sensor data block
 * 
 */
static void Parse(sensor_data_event_t s)
{
	int i;

	for(i = 0; i < VS_MAX; i++)
	{
		if(s.id == this.vs_list[i].id)
		{
			switch(s.data_type)
			{
				case BHI260_SENS_DATA_TYPE_QUATERNION:
					BHI260_GetQuaternion(s,&this.vs_list[i].data.q);
					break;
				
				case BHI260_SENS_DATA_TYPE_3DVECTOR:
					BHI260_GetVector(s,&this.vs_list[i].data.v);
					break;
				
				default: break;
			}
			break;
		}
	}
}

/**
 * @brief Checks if there is SensorData in the fifo queue and calls the 
 * corresponding parser
 * 
 */
static void GetSamples()
{
	sensor_data_event_t s;

	if(BHI260_FIFO_SensorData_Pull(&s))
	{
		Parse(s);
	}
}

/**
 * @brief registers any change on the accuracy status of the enabled sensors
 * 
 */
static void GetSensorAccuracyChanges()
{
	meta_event_t evt;
	int i;
	if(BHI260_FIFO_MetaEvent_SensorStatus_Pull(&evt))
	{
		for(i = 0; i < VS_MAX; i++)
		{
			if(evt.values[0] == this.vs_list[i].id)
			{
				this.vs_list[i].accu_status = evt.values[1];
			}
		}
	}
}

/******************************************************************************
 * Callback Functions
 ******************************************************************************/
/**
 * Tick callback registered for this module
 */
static void BHI260_tick_callback()
{
	if(this.timer) this.timer--;
}

/******************************************************************************
 * Public Functions
 ******************************************************************************/
void BHI260AB_Init()
{
	this.state = STATE_INIT;
	TICK_CallbackRegister(BHI260_tick_callback);
	BHI260_FIFO_Init();
	this.pause_fifo = false;

	// Init all the VS in 0
	memset(this.vs_list, 0, sizeof(this.vs_list));

	// Init rotation vector
	this.vs_list[VS_GAME_ROTATION_VECTOR].id = BHY2_SENSOR_ID_GAMERV;
	this.vs_list[VS_GAME_ROTATION_VECTOR].data_type = BHI260_SENS_DATA_TYPE_QUATERNION;

	// Init gravity vector
	this.vs_list[VS_GRAVITY].id = BHY2_SENSOR_ID_GRA;
	this.vs_list[VS_GRAVITY].data_type = BHI260_SENS_DATA_TYPE_3DVECTOR;

	// Init accel corrected
	this.vs_list[VS_ACCEL_CORRECTED].id = BHY2_SENSOR_ID_ACC;
	this.vs_list[VS_ACCEL_CORRECTED].data_type = BHI260_SENS_DATA_TYPE_3DVECTOR;

	// Init gyro corrected
	this.vs_list[VS_GYRO_CORRECTED].id = BHY2_SENSOR_ID_GYRO;
	this.vs_list[VS_GYRO_CORRECTED].data_type = BHI260_SENS_DATA_TYPE_3DVECTOR;

	// Init magnetometer corrected
	this.vs_list[VS_MAG_CORRECTED].id = BHY2_SENSOR_ID_MAG;
	this.vs_list[VS_MAG_CORRECTED].data_type = BHI260_SENS_DATA_TYPE_3DVECTOR;
}

void BHI260AB_Tasks()
{
	BHI260_FIFO_Tasks();
	GetSensorAccuracyChanges();

	switch(this.state)
	{
		case STATE_INIT:
			switch(Startup())
			{
				case BHI260_RET_WORKING: break;

				case BHI260_RET_OK:
					DEBUG_PRINTF("Initialization finished OK");
					this.state = STATE_SYSTEM_SETUP;
					break;
				
				default:
				case BHI260_RET_FAILED:
				case BHI260_RET_IDLE:
					DEBUG_PRINTF("Initialization failed");
					this.state = STATE_FAILURE;
					break;
			}
			break;

		case STATE_SYSTEM_SETUP:
			switch(SystemSetup())
			{
				case BHI260_RET_WORKING: break;

				case BHI260_RET_OK:
					DEBUG_PRINTF("System ok and running");
					this.state = STATE_RUNNING;
					break;

				default:
				case BHI260_RET_FAILED:
				case BHI260_RET_IDLE:
					DEBUG_PRINTF("Error on system setup");
					this.state = STATE_FAILURE;
					break;
			}
			break;
		
		case STATE_RUNNING:
			GetSamples();
			break;

		case STATE_IDLE:
			break;

		case STATE_FAILURE:
			break;

		default:break;
	}
}

bool BHI260AB_isReady()
{
	return this.state == STATE_RUNNING;
}

quaternion_t BHI260AB_GetQuaternion(virtual_sensor_list_t idx)
{
	quaternion_t q;
	memset(&q,0,sizeof(q));
	
	if(idx < VS_MAX)
	{
		if(this.vs_list[idx].data_type == BHI260_SENS_DATA_TYPE_QUATERNION)
		{	
			memcpy(&q,&this.vs_list[idx].data.q,sizeof(q));
		}		
	}

	return q;
}

vector3D_t BHI260AB_Get3DVector(virtual_sensor_list_t idx)
{
	vector3D_t v;
	memset(&v,0,sizeof(v));

	if(idx < VS_MAX)
	{
		if(this.vs_list[idx].data_type == BHI260_SENS_DATA_TYPE_3DVECTOR)
		{	
			memcpy(&v,&this.vs_list[idx].data.v,sizeof(v));
		}		
	}

	return v;
}

sensor_accuracy_status_t BHI260AB_GetAccuracyStatus(virtual_sensor_list_t idx)
{
	sensor_accuracy_status_t status = ACCURACY_UNRELIABLE;

	if(idx < VS_MAX)
	{
		status = this.vs_list[idx].accu_status;
	}

	return status;
}
