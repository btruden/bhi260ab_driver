/*
 * spi1.c
 *
 *  Created on: Aug 26, 2021
 *      Author: btrud
 */

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <imu/spi1.h>
#include "main.h"

/******************************************************************************
 * Local Constants
 ******************************************************************************/

/******************************************************************************
 * Local Types
 ******************************************************************************/

/******************************************************************************
 * Local Variables
 ******************************************************************************/
// External variables created by CubeMX
extern SPI_HandleTypeDef hspi1;

/**
 * Local data container
 */
static struct local_data
{
	// Callback pointers
	spi1_tx_callback_t tx_callback;
	spi1_rx_callback_t rx_callback;
	spi1_tx_rx_callback_t tx_rx_callback;
}this;

/******************************************************************************
 * Local Functions
 ******************************************************************************/

/******************************************************************************
 * Callback Functions
 ******************************************************************************/
/**
  * @brief  Tx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi_int_source)
{
	if(this.tx_callback != NULL)
	{
		this.tx_callback();
	}
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi_int_source)
{
	if(this.rx_callback != NULL)
	{
		this.rx_callback();
	}
}

/**
  * @brief  Tx and Rx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi_int_source)
{
	if(this.tx_rx_callback != NULL)
	{
		this.tx_rx_callback();
	}
}

/******************************************************************************
 * Public Functions
 ******************************************************************************/
void SPI1_Init()
{
	// Callback clean
	this.rx_callback = NULL;
	this.tx_callback = NULL;
	this.tx_rx_callback = NULL;
}

/**
  * @brief  Transmit an amount of data in non-blocking mode with DMA.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @retval true if succeeded.
  * @retval false if failed.
  */
bool SPI1_Transmit(uint8_t *pData, uint16_t Size)
{
	return (HAL_SPI_Transmit_DMA(&hspi1,pData,Size) == HAL_OK);
}

/**
  * @brief  Receive an amount of data in non-blocking mode with DMA.
  * @note   In case of MASTER mode and SPI_DIRECTION_2LINES direction, hdmatx shall be defined.
  * @param  pData pointer to data buffer
  * @note   When the CRC feature is enabled the pData Length must be Size + 1.
  * @param  Size amount of data to be sent
  * @retval true if succeeded.
  * @retval false if failed.
  */
bool SPI1_Receive(uint8_t *pData, uint16_t Size)
{
	return (HAL_SPI_Receive_DMA(&hspi1,pData,Size) == HAL_OK);
}

/**
  * @brief  Transmit and Receive an amount of data in non-blocking mode with DMA.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @note   When the CRC feature is enabled the pRxData Length must be Size + 1
  * @param  Size amount of data to be sent
  * @retval true if succeeded.
  * @retval false if failed.
  */
bool SPI1_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	return (HAL_SPI_TransmitReceive_DMA(&hspi1,pTxData,pRxData,Size) == HAL_OK);
}

/**
 * Registers a transmission finished callback
 */
void SPI1_RegisterCallback_Tx(spi1_tx_callback_t func)
{
	this.tx_callback = func;
}

/**
 * Registers a reception finished callback
 */
void SPI1_RegisterCallback_Rx(spi1_rx_callback_t func)
{
	this.rx_callback = func;
}

/**
 * Registers a transmission/reception finished callback
 */
void SPI1_RegisterCallback_TxRx(spi1_tx_rx_callback_t func)
{
	this.tx_rx_callback = func;
}
