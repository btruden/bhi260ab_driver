/*
 * bhi260_spi.h
 *
 *  Created on: Sep 10, 2021
 *      Author: btrud
 */

#ifndef INC_IMU_BHI260_SPI_H_
#define INC_IMU_BHI260_SPI_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "spi.h"
#include "bhi260_common.h"

/******************************************************************************
 * Public Constants
 ******************************************************************************/

/******************************************************************************
 * Public Types
 ******************************************************************************/
/**
 * Host Interface Register Write or Read structure
 */
typedef struct reg_read_or_write
{
	spi_transaction_t t;		// SPI transaction structure
	uint8_t addr;				// register address
}reg_read_or_write_t;

/******************************************************************************
 * Public Functions
 ******************************************************************************/
/**
 * Writes a bytes to a register of the Host Interface
 *
 * @param regWrite pointer to a reg_read_or_write_t structure
 * @param addr register address to write
 * @param txbuf pointer to the buffer containing the data to be written. txbuf[0] will be filled
 * with the register address and the WRITE flag.
 * @param data_size data size in Bytes to be written or read
 * @param CS_ctrl CS line control enable. If enabled the host interface will automatically assert
 * and release the CS line when performing the operation
 * @return true if process initialized correctly
 * @return false if process couldn't initialize correctly
 */
bool RegWrite(reg_read_or_write_t *regWrite, uint8_t addr, uint8_t *txbuf, uint32_t data_size, bool CS_ctrl);

/**
 * Reads bytes from a register of the Host Interface
 *
 * @param regRead pointer to a reg_read_or_write_t structure
 * @param addr register address to write
 * @param rxbuf pointer to the buffer to put the data to be read. rxbuf[0] will be filled
 * with the register address and the READ flag.
 * @param data_size data size in Bytes to be written or read
 * @param CS_ctrl CS line control enable. If enabled the host interface will automatically assert
 * and release the CS line when performing the operation
 * @return true if process initialized correctly
 * @return false if process couldn't initialize correctly
 */
bool RegRead(reg_read_or_write_t *regRead, uint8_t addr, uint8_t *rxbuf, uint32_t data_size, bool CS_ctrl);

/**
 * Returns when the read or write register process finishes
 *
 * @param regProc pointer to a reg_read_or_write_t structure
 * @return true when finished
 * @return false if not finished
 */
bool RegProcess_Done(reg_read_or_write_t *regProc);

/**
 * @brief State machine that peforms a complete command execution on the Host Command Interface
 * 
 * @param cmd_id command ID
 * @param data data body of the command. If the command doesn't require any data set to NULL.
 * @param length data length of the command. If the command doesn't require any data set to 0.
 * @param data_element_size Indicates the size of each element of the data. For example, when
 * uploading firmware the <length> parameter is the size of the firmware divided by 4 (double word)
 * and the <data_element_size> will be 4
 * @return bhi260_ret_t type
 */
bhi260_ret_t SendCmd(bhi260_hostCmd_id_t cmd_id, uint8_t *data, uint32_t length, size_t data_element_size);

/**
 * @brief Sends a read parameter command over the Host Interface. The user must wait for the
 * status queue for the response
 * 
 * @param param_addr parameter address to read
 * @return bhi260_ret_t 
 */
bhi260_ret_t ReadParameter(uint16_t param_addr);

/**
 * @brief Sends a write parameter command over the Host Interface
 * 
 * @param param_addr address of the parameter to write
 * @param length lenght of the data to write
 * @param buf buffer containing the data to write. Byte array.
 * @return bhi260_ret_t 
 */
bhi260_ret_t WriteParameter(uint16_t param_addr, uint16_t length, uint8_t *buf);

/**
 * @brief Asserts the Host Interface CS line
 * 
 */
void BHI260_SPI_CSassert();

/**
 * @brief Releases the Host Interface CS line
 * 
 */
void BHI260_SPI_CSrelease();

/**
 * @brief Returns if the SPI is in use. 
 * 
 * @return true 
 * @return false 
 */
bool BHI260_SPI_Bussy();

/**
 * @brief Takes the SPI. It must be given after used
 * 
 * @return true SPI successfuly taken
 * @return false SPI bussy
 */
bool BHI260_SPI_Take();

/**
 * @brief Gives the SPI
 * 
 * @return true SPI successfuly given
 * @return false SPI bussy
 */
bool BHI260_SPI_Give();

#endif /* INC_IMU_BHI260_SPI_H_ */
