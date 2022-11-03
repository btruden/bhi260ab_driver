/*
 * spi1.h
 *
 *  Created on: Aug 26, 2021
 *      Author: btrud
 */

#ifndef INC_IMU_SPI1_H_
#define INC_IMU_SPI1_H_

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

/******************************************************************************
 * Public Constants
 ******************************************************************************/

/******************************************************************************
 * Public Types
 ******************************************************************************/
/**
 * Initializes the module
 */
void SPI1_Init();

/**
 * Callback prototype for transmission complete
 */
typedef void (*spi1_tx_callback_t)(void);

/**
 * Callback prototype for reception complete
 */
typedef void (*spi1_rx_callback_t)(void);

/**
 * Callback prototype for transmission then reception complete
 */
typedef void (*spi1_tx_rx_callback_t)(void);

/**
 * SPI1 callback types
 */
typedef enum
{
	SPI1_CLCK_TX_DONE = 0,		// Transmission finished
	SPI1_CLCK_RX_DONE,			// Reception finished
	SPI1_CLCK_TX_THEN_RX_DONE	// Transmission/Reception finished
}spi_callback_type_t;

/******************************************************************************
 * Public Functions
 ******************************************************************************/
/**
  * @brief  Transmit an amount of data in non-blocking mode with DMA.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @retval true if succeeded.
  * @retval false if failed.
  */
bool SPI1_Transmit(uint8_t *pData, uint16_t Size);

/**
  * @brief  Receive an amount of data in non-blocking mode with DMA.
  * @note   In case of MASTER mode and SPI_DIRECTION_2LINES direction, hdmatx shall be defined.
  * @param  pData pointer to data buffer
  * @note   When the CRC feature is enabled the pData Length must be Size + 1.
  * @param  Size amount of data to be sent
  * @retval true if succeeded.
  * @retval false if failed.
  */
bool SPI1_Receive(uint8_t *pData, uint16_t Size);

/**
  * @brief  Transmit and Receive an amount of data in non-blocking mode with DMA.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @note   When the CRC feature is enabled the pRxData Length must be Size + 1
  * @param  Size amount of data to be sent
  * @retval true if succeeded.
  * @retval false if failed.
  */
bool SPI1_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);

/**
 * Registers a transmission finished callback
 */
void SPI1_RegisterCallback_Tx(spi1_tx_callback_t func);

/**
 * Registers a reception finished callback
 */
void SPI1_RegisterCallback_Rx(spi1_rx_callback_t func);

/**
 * Registers a transmission/reception finished callback
 */
void SPI1_RegisterCallback_TxRx(spi1_tx_rx_callback_t func);

#endif /* INC_IMU_SPI1_H_ */
