/*
 * bhi260_spi.c
 *
 *  Created on: Sep 10, 2021
 *      Author: btrud
 */

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "bhi260_spi.h"
#include "bhi260_defs.h"
#include "board.h"

/******************************************************************************
 * Local Constants
 ******************************************************************************/
// SPI read or write
#define SPI_WRITE_MASK		0x00
#define SPI_READ_MASK		0x80

// Debug options
#define ENABLE_DEBUG        true
#define DEBUG_TAG           "BHI260_SPI"

#if ENABLE_DEBUG == true
#define DEBUG_PRINTF(format, ...) Debug_Print(DEBUG_TAG, true, format, ##__VA_ARGS__)
#define DEBUG_PRINT(format, ...) Debug_Print(DEBUG_TAG, false, format, ##__VA_ARGS__)
#define PRINTF(format, ...) Debug_Print(0, true, format, ##__VA_ARGS__)
#else
#define DEBUG_PRINTF(format, ...)
#define DEBUG_PRINT(format, ...)
#define PRINTF(format, ...)
#endif

/******************************************************************************
 * Local Types
 ******************************************************************************/

/******************************************************************************
 * Local Variables
 ******************************************************************************/

static struct 
{
	bool spi_free;
}this = {
	.spi_free = true
};

/******************************************************************************
 * Local Functions
 ******************************************************************************/
/**
 * Asserts the CS line of the SPI interface
 */
static void SpiCSassert()
{
	BOARD_BHI_CS_Enable();
}

/**
 * Releases the CS line of the SPI interface
 */
static void SpiCSrelease()
{
	BOARD_BHI_CS_Disable();
}

/******************************************************************************
 * Callback Functions
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/
bool RegWrite(reg_read_or_write_t *regWrite, uint8_t addr, uint8_t *txbuf, uint32_t data_size, bool CS_ctrl)
{
	if(regWrite == NULL) return false;
	if(txbuf == NULL) return false;

	// Prepare tx buffer
	*txbuf = addr | SPI_WRITE_MASK;

	// Charge the SPI transaction. If the CS control is enabled then charge the 
	// CS assert and release functions, if not set to NULL
	if(CS_ctrl)
	{
		return SPI_CreateTransactionAdd(&regWrite->t,
			txbuf,
			txbuf,
			data_size+1,
			SpiCSassert,
			SpiCSrelease,
			NULL);
	}
	else
	{
		return SPI_CreateTransactionAdd(&regWrite->t,
			txbuf,
			txbuf,
			data_size+1,
			NULL,
			NULL,
			NULL);
	}
}

bool RegRead(reg_read_or_write_t *regRead, uint8_t addr, uint8_t *rxbuf, uint32_t data_size, bool CS_ctrl)
{
	if(regRead == NULL) return false;
	if(rxbuf == NULL) return false;


	// Prepare tx buffer
	*rxbuf = addr | SPI_READ_MASK;

	// Charge the SPI transaction. If the CS control is enabled then charge the 
	// CS assert and release functions, if not set to NULL
	if(CS_ctrl)
	{
		return SPI_CreateTransactionAdd(&regRead->t,
			rxbuf,
			rxbuf,
			data_size+1,
			SpiCSassert,
			SpiCSrelease,
			NULL);
	}
	else
	{
		return SPI_CreateTransactionAdd(&regRead->t,
			rxbuf,
			rxbuf,
			data_size+1,
			NULL,
			NULL,
			NULL);
	}	
}

bool RegProcess_Done(reg_read_or_write_t *regProc)
{
	return (regProc->t.status == SPI_TRANSACTION_STATUS_FINISHED ||
			regProc->t.status == SPI_TRANSACTION_STATUS_FAILED);
}

bhi260_ret_t SendCmd(bhi260_hostCmd_id_t cmd_id, uint8_t *data, uint32_t length, size_t data_element_size)
{
	bhi260_ret_t ret = 	BHI260_RET_WORKING;

	// Local copies
	static uint8_t cmd_buf[5];
	static spi_transaction_t t;
	static uint16_t dataBlocks, dataReminder;
	static uint32_t dataIdx = 0;

	// Machine state variable
	static enum
	{
		CMDSTATE_INIT = 0,
		CMDSTATE_SEND_CMD,
		CMDSTATE_SEND_DATA_BLOCK,
		CMDSTATE_SEND_DATA_REMINDER
	}cmdState = CMDSTATE_INIT;

	switch(cmdState)
	{
		case CMDSTATE_INIT:
			// Calculate the send routines
			dataBlocks = (length*data_element_size)/SPI_MAX_TRANSACTION_SIZE;
			dataReminder = (length*data_element_size)%SPI_MAX_TRANSACTION_SIZE;
			dataIdx = 0;

			// Prepare the command buffer
			cmd_buf[0] = BHY2_REG_CHAN_CMD | SPI_WRITE_MASK;
			cmd_buf[1] = (uint8_t)(cmd_id & 0x00ff);
			cmd_buf[2] = (uint8_t)((cmd_id & 0xff00)>>8);
			cmd_buf[3] = (uint8_t)(length & 0x00ff);
			cmd_buf[4] = (uint8_t)((length & 0xff00)>>8);

			// Send the command buffer
			SpiCSassert();

			if(SPI_CreateTransactionAdd(&t,cmd_buf,cmd_buf,sizeof(cmd_buf),NULL,NULL,NULL))
			{
				cmdState = CMDSTATE_SEND_CMD;
			}
			else
			{
				SpiCSrelease();
				ret = BHI260_RET_FAILED;
			}
			break;
		
		case CMDSTATE_SEND_CMD:
			if(t.status == SPI_TRANSACTION_STATUS_FINISHED)
			{
				// Are there still blocks to send?
				if(dataBlocks > 0)
				{
					if(SPI_CreateTransactionAdd(&t,data+dataIdx,data+dataIdx,SPI_MAX_TRANSACTION_SIZE,NULL,NULL,NULL))
					{
						cmdState = CMDSTATE_SEND_DATA_BLOCK;
					}
					else
					{
						SpiCSrelease();
						ret = BHI260_RET_FAILED;
						cmdState = CMDSTATE_INIT;
					}
				}
				else if(dataReminder > 0)
				{
					if(SPI_CreateTransactionAdd(&t,data+dataIdx,data+dataIdx,dataReminder,NULL,NULL,NULL))
					{
						cmdState = CMDSTATE_SEND_DATA_REMINDER;
					}
					else
					{
						SpiCSrelease();
						ret = BHI260_RET_FAILED;
						cmdState = CMDSTATE_INIT;
					}
				}
				else
				{
					SpiCSrelease();
					ret = BHI260_RET_OK;
					cmdState = CMDSTATE_INIT;
				}
			}
			else if(t.status == SPI_TRANSACTION_STATUS_FAILED)
			{
				SpiCSrelease();
				ret = BHI260_RET_FAILED;
				cmdState = CMDSTATE_INIT;
			}
			break;
		
		case CMDSTATE_SEND_DATA_BLOCK:
			if(t.status == SPI_TRANSACTION_STATUS_FINISHED)
			{
				// Update indexes and counters
				dataIdx += SPI_MAX_TRANSACTION_SIZE;
				dataBlocks--;

				// Are there still blocks to send?
				if(dataBlocks > 0)
				{
					if(SPI_CreateTransactionAdd(&t,data+dataIdx,data+dataIdx,SPI_MAX_TRANSACTION_SIZE,NULL,NULL,NULL))
					{
						cmdState = CMDSTATE_SEND_DATA_BLOCK;
					}
					else
					{
						SpiCSrelease();
						ret = BHI260_RET_FAILED;
						cmdState = CMDSTATE_INIT;
					}
				}
				else if(dataReminder > 0)
				{
					if(SPI_CreateTransactionAdd(&t,data+dataIdx,data+dataIdx,dataReminder,NULL,NULL,NULL))
					{
						cmdState = CMDSTATE_SEND_DATA_REMINDER;
					}
					else
					{
						SpiCSrelease();
						ret = BHI260_RET_FAILED;
						cmdState = CMDSTATE_INIT;
					}
				}
				else
				{
					SpiCSrelease();
					ret = BHI260_RET_OK;
					cmdState = CMDSTATE_INIT;
				}
			}
			else if(t.status == SPI_TRANSACTION_STATUS_FAILED)
			{
				SpiCSrelease();
				ret = BHI260_RET_FAILED;
				cmdState = CMDSTATE_INIT;
			}
			break;
		
		case CMDSTATE_SEND_DATA_REMINDER:
			if(t.status == SPI_TRANSACTION_STATUS_FINISHED)
			{
				SpiCSrelease();
				ret = BHI260_RET_OK;
				cmdState = CMDSTATE_INIT;
			}
			else if(t.status == SPI_TRANSACTION_STATUS_FAILED)
			{
				SpiCSrelease();
				ret = BHI260_RET_FAILED;
				cmdState = CMDSTATE_INIT;
			}
			break;
		
		default:
			SpiCSrelease();
			ret = BHI260_RET_FAILED;
			cmdState = CMDSTATE_INIT;
			break;
	}

	return ret;
}

bhi260_ret_t ReadParameter(uint16_t param_addr)
{
	return SendCmd(BHY2_PARAM_READ_MASK+param_addr,NULL,0,1);	
}

bhi260_ret_t WriteParameter(uint16_t param_addr, uint16_t length, uint8_t *buf)
{
	return SendCmd(BHY2_PARAM_WRITE_MASK+param_addr,buf,length,1);
}

void BHI260_SPI_CSassert()
{
	SpiCSassert();
}

void BHI260_SPI_CSrelease()
{
	SpiCSrelease();
}

bool BHI260_SPI_Take()
{
	if(this.spi_free && SPI_isDone())
	{
		this.spi_free = false;
		return true;
	}
	else
	{
		return false;
	}
}

bool BHI260_SPI_Give()
{
	if(SPI_isDone())
	{
		this.spi_free = true;
		return true;
	}
	else
	{
		return false;
	}
}