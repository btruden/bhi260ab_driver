/*
 * bhi260_defs.h
 *
 *  Created on: Aug 31, 2021
 *      Author: btrud
 */

#ifndef INC_IMU_BHI260_DEFS_H_
#define INC_IMU_BHI260_DEFS_H_

/*! Register map */
#define BHY2_REG_CHAN_CMD                UINT8_C(0x00)
#define BHY2_REG_CHAN_FIFO_W             UINT8_C(0x01)
#define BHY2_REG_CHAN_FIFO_NW            UINT8_C(0x02)
#define BHY2_REG_CHAN_STATUS             UINT8_C(0x03)
#define BHY2_REG_CHIP_CTRL               UINT8_C(0x05)
#define BHY2_REG_HOST_INTERFACE_CTRL     UINT8_C(0x06)
#define BHY2_REG_HOST_INTERRUPT_CTRL     UINT8_C(0x07)
#define BHY2_REG_RESET_REQ               UINT8_C(0x14)
#define BHY2_REG_TIME_EV_REQ             UINT8_C(0x15)
#define BHY2_REG_HOST_CTRL               UINT8_C(0x16)
#define BHY2_REG_HOST_STATUS             UINT8_C(0x17)
#define BHY2_REG_CRC_0                   UINT8_C(0x18)          /* Totally 4 */
#define BHY2_REG_PRODUCT_ID              UINT8_C(0x1C)
#define BHY2_REG_REVISION_ID             UINT8_C(0x1D)
#define BHY2_REG_ROM_VERSION_0           UINT8_C(0x1E)          /* Totally 2 */
#define BHY2_REG_KERNEL_VERSION_0        UINT8_C(0x20)          /* Totally 2 */
#define BHY2_REG_USER_VERSION_0          UINT8_C(0x22)          /* Totally 2 */
#define BHY2_REG_FEATURE_STATUS          UINT8_C(0x24)
#define BHY2_REG_BOOT_STATUS             UINT8_C(0x25)
#define BHY2_REG_HOST_INTR_TIME_0        UINT8_C(0x26)          /* Totally 5 */
#define BHY2_REG_CHIP_ID                 UINT8_C(0x2B)
#define BHY2_REG_INT_STATUS              UINT8_C(0x2D)
#define BHY2_REG_ERROR_VALUE             UINT8_C(0x2E)
#define BHY2_REG_ERROR_AUX               UINT8_C(0x2F)
#define BHY2_REG_DEBUG_VALUE             UINT8_C(0x30)
#define BHY2_REG_DEBUG_STATE             UINT8_C(0x31)
#define BHY2_REG_GP_5                    UINT8_C(0x32)
#define BHY2_REG_GP_6                    UINT8_C(0x36)
#define BHY2_REG_GP_7                    UINT8_C(0x3A)

/*! Reset command */
#define BHY2_REQUEST_RESET               UINT8_C(0x01)

/*! Boot status */
#define BHY2_BST_FLASH_DETECTED       UINT8_C(0x01)
#define BHY2_BST_FLASH_VERIFY_DONE    UINT8_C(0x02)
#define BHY2_BST_FLASH_VERIFY_ERROR   UINT8_C(0x04)
#define BHY2_BST_NO_FLASH             UINT8_C(0x08)
#define BHY2_BST_HOST_INTERFACE_READY UINT8_C(0x10)
#define BHY2_BST_HOST_FW_VERIFY_DONE  UINT8_C(0x20)
#define BHY2_BST_HOST_FW_VERIFY_ERROR UINT8_C(0x40)
#define BHY2_BST_HOST_FW_IDLE         UINT8_C(0x80)
#define BHY2_BST_CHECK_RETRY          UINT8_C(100)

/*! Product ID */
#define BHY2_PRODUCT_ID               UINT8_C(0x89)

/*! Command packets */
#define BHY2_CMD_REQ_POST_MORTEM_DATA    UINT16_C(0x0001)
#define BHY2_CMD_UPLOAD_TO_PROGRAM_RAM   UINT16_C(0x0002)
#define BHY2_CMD_BOOT_PROGRAM_RAM        UINT16_C(0x0003)
#define BHY2_CMD_ERASE_FLASH             UINT16_C(0x0004)
#define BHY2_CMD_WRITE_FLASH             UINT16_C(0x0005)
#define BHY2_CMD_BOOT_FLASH              UINT16_C(0x0006)
#define BHY2_CMD_SET_INJECT_MODE         UINT16_C(0x0007)
#define BHY2_CMD_INJECT_DATA             UINT16_C(0x0008)
#define BHY2_CMD_FIFO_FLUSH              UINT16_C(0x0009)
#define BHY2_CMD_SW_PASSTHROUGH          UINT16_C(0x000A)
#define BHY2_CMD_REQ_SELF_TEST           UINT16_C(0x000B)
#define BHY2_CMD_REQ_FOC                 UINT16_C(0x000C)
#define BHY2_CMD_CONFIG_SENSOR           UINT16_C(0x000D)
#define BHY2_CMD_CHANGE_RANGE            UINT16_C(0x000E)
#define BHY2_CMD_FIFO_FORMAT_CTRL        UINT16_C(0x0015)

/*! Command replies ID */
#define BHY2_CMD_REQ_SELF_TEST_REPLY_ID  UINT16_C(0x0006)
#define BHY2_SELF_TEST_OK                UINT8_C(0x00)  

/*! Interrupt Status bits */
#define BHY2_HOST_INTA_ASSERTED_MASK    UINT8_C(0x01)
#define BHY2_HOST_INTA_ASSERTED_SHIFT   UINT8_C(0)
#define BHY2_INT_WAKEUP_FIFO_MASK       UINT8_C(0x06)
#define BHY2_INT_WAKEUP_FIFO_SHIFT      UINT8_C(1)
#define BHY2_INT_NONWAKEUP_FIFO_MASK    UINT8_C(0x18)
#define BHY2_INT_NONWAKEUP_FIFO_SHIFT   UINT8_C(3)
#define BHY2_INT_STATUS_MASK            UINT8_C(0x20)
#define BHY2_INT_STATUS_SHIFT           UINT8_C(5)
#define BHY2_INT_DEBUG_MASK             UINT8_C(0x40)
#define BHY2_INT_DEBUG_SHIFT            UINT8_C(6)
#define BHY2_INT_RST_OR_FAULT_MASK      UINT8_C(0x80)
#define BHY2_INT_RST_OR_FAULT_SHIFT     UINT8_C(7)

// FIFO interrupt status values
#define BHI2_INT_STATUS_FIFO_NO_DATA      UINT8_C(0x00)
#define BHI2_INT_STATUS_FIFO_IMMEDIATE    UINT8_C(0x01)
#define BHI2_INT_STATUS_FIFO_LATENCY      UINT8_C(0x02)
#define BHI2_INT_STATUS_FIFO_WATERMARK    UINT8_C(0x03)

// Interrupt Status macros
#define BHY2_INT_STATUS_Is_IntaAsserted(stat)    (stat & BHY2_HOST_INTA_ASSERTED_MASK)

#define BHY2_INT_STATUS_Is_WakeupNodata(stat)    (((stat & BHY2_INT_WAKEUP_FIFO_MASK) >> BHY2_INT_WAKEUP_FIFO_SHIFT) == BHI2_INT_STATUS_FIFO_NO_DATA);
#define BHY2_INT_STATUS_Is_WakeupImmedate(stat)  (((stat & BHY2_INT_WAKEUP_FIFO_MASK) >> BHY2_INT_WAKEUP_FIFO_SHIFT) == BHI2_INT_STATUS_FIFO_IMMEDIATE);
#define BHY2_INT_STATUS_Is_WakeupLatency(stat)   (((stat & BHY2_INT_WAKEUP_FIFO_MASK) >> BHY2_INT_WAKEUP_FIFO_SHIFT) == BHI2_INT_STATUS_FIFO_LATENCY);
#define BHY2_INT_STATUS_Is_WakeupWatermark(stat) (((stat & BHY2_INT_WAKEUP_FIFO_MASK) >> BHY2_INT_WAKEUP_FIFO_SHIFT) == BHI2_INT_STATUS_FIFO_WATERMARK);

#define BHY2_INT_STATUS_Is_NonWakeupNodata(stat)    (((stat & BHY2_INT_NONWAKEUP_FIFO_MASK) >> BHY2_INT_NONWAKEUP_FIFO_SHIFT) == BHI2_INT_STATUS_FIFO_NO_DATA)
#define BHY2_INT_STATUS_Is_NonWakeupImmedate(stat)  (((stat & BHY2_INT_NONWAKEUP_FIFO_MASK) >> BHY2_INT_NONWAKEUP_FIFO_SHIFT) == BHI2_INT_STATUS_FIFO_IMMEDIATE)
#define BHY2_INT_STATUS_Is_NonWakeupLatency(stat)   (((stat & BHY2_INT_NONWAKEUP_FIFO_MASK) >> BHY2_INT_NONWAKEUP_FIFO_SHIFT) == BHI2_INT_STATUS_FIFO_LATENCY)
#define BHY2_INT_STATUS_Is_NonWakeupWatermark(stat) (((stat & BHY2_INT_NONWAKEUP_FIFO_MASK) >> BHY2_INT_NONWAKEUP_FIFO_SHIFT) == BHI2_INT_STATUS_FIFO_WATERMARK)

#define BHY2_INT_STATUS_Is_Status(stat)         (stat & BHY2_INT_STATUS_MASK)
#define BHY2_INT_STATUS_Is_Debug(stat)          (stat & BHY2_INT_DEBUG_MASK)
#define BHY2_INT_STATUS_Is_RstOrFault(stat)     (stat & BHY2_INT_RST_OR_FAULT_MASK)

/* Sensor IDs */
#define BHY2_SENSOR_ID_CUSTOM_START             UINT8_C(160)
#define BHY2_SENSOR_ID_CUSTOM_END               UINT8_C(191)

#define BHY2_SENSOR_ID_MAX                      UINT8_C(200)
#define BHY2_SENSOR_ID_TBD                      UINT8_C(BHY2_SENSOR_ID_MAX - 1)
#define BHY2_SENSOR_ID_ACC_PASS                 UINT8_C(1)   /* Accelerometer passthrough */
#define BHY2_SENSOR_ID_ACC_RAW                  UINT8_C(3)   /* Accelerometer uncalibrated */
#define BHY2_SENSOR_ID_ACC                      UINT8_C(4)   /* Accelerometer corrected */
#define BHY2_SENSOR_ID_ACC_BIAS                 UINT8_C(5)   /* Accelerometer offset */
#define BHY2_SENSOR_ID_ACC_WU                   UINT8_C(6)   /* Accelerometer corrected wake up */
#define BHY2_SENSOR_ID_ACC_RAW_WU               UINT8_C(7)   /* Accelerometer uncalibrated wake up */
#define BHY2_SENSOR_ID_GYRO_PASS                UINT8_C(10)  /* Gyroscope passthrough */
#define BHY2_SENSOR_ID_GYRO_RAW                 UINT8_C(12)  /* Gyroscope uncalibrated */
#define BHY2_SENSOR_ID_GYRO                     UINT8_C(13)  /* Gyroscope corrected */
#define BHY2_SENSOR_ID_GYRO_BIAS                UINT8_C(14)  /* Gyroscope offset */
#define BHY2_SENSOR_ID_GYRO_WU                  UINT8_C(15)  /* Gyroscope wake up */
#define BHY2_SENSOR_ID_GYRO_RAW_WU              UINT8_C(16)  /* Gyroscope uncalibrated wake up */
#define BHY2_SENSOR_ID_MAG_PASS                 UINT8_C(19)  /* Magnetometer passthrough */
#define BHY2_SENSOR_ID_MAG_RAW                  UINT8_C(21)  /* Magnetometer uncalibrated */
#define BHY2_SENSOR_ID_MAG                      UINT8_C(22)  /* Magnetometer corrected */
#define BHY2_SENSOR_ID_MAG_BIAS                 UINT8_C(23)  /* Magnetometer offset */
#define BHY2_SENSOR_ID_MAG_WU                   UINT8_C(24)  /* Magnetometer wake up */
#define BHY2_SENSOR_ID_MAG_RAW_WU               UINT8_C(25)  /* Magnetometer uncalibrated wake up */
#define BHY2_SENSOR_ID_GRA                      UINT8_C(28)  /* Gravity vector */
#define BHY2_SENSOR_ID_GRA_WU                   UINT8_C(29)  /* Gravity vector wake up */
#define BHY2_SENSOR_ID_LACC                     UINT8_C(31)  /* Linear acceleration */
#define BHY2_SENSOR_ID_LACC_WU                  UINT8_C(32)  /* Linear acceleration wake up */
#define BHY2_SENSOR_ID_RV                       UINT8_C(34)  /* Rotation vector */
#define BHY2_SENSOR_ID_RV_WU                    UINT8_C(35)  /* Rotation vector wake up */
#define BHY2_SENSOR_ID_GAMERV                   UINT8_C(37)  /* Game rotation vector */
#define BHY2_SENSOR_ID_GAMERV_WU                UINT8_C(38)  /* Game rotation vector wake up */
#define BHY2_SENSOR_ID_GEORV                    UINT8_C(40)  /* Geo-magnetic rotation vector */
#define BHY2_SENSOR_ID_GEORV_WU                 UINT8_C(41)  /* Geo-magnetic rotation vector wake up */
#define BHY2_SENSOR_ID_ORI                      UINT8_C(43)  /* Orientation */
#define BHY2_SENSOR_ID_ORI_WU                   UINT8_C(44)  /* Orientation wake up */
#define BHY2_SENSOR_ID_TILT_DETECTOR            UINT8_C(48)  /* Tilt detector */
#define BHY2_SENSOR_ID_STD                      UINT8_C(50)  /* Step detector */
#define BHY2_SENSOR_ID_STC                      UINT8_C(52)  /* Step counter */
#define BHY2_SENSOR_ID_STC_WU                   UINT8_C(53)  /* Step counter wake up */
#define BHY2_SENSOR_ID_SIG                      UINT8_C(55)  /* Significant motion */
#define BHY2_SENSOR_ID_WAKE_GESTURE             UINT8_C(57)  /* Wake gesture */
#define BHY2_SENSOR_ID_GLANCE_GESTURE           UINT8_C(59)  /* Glance gesture */
#define BHY2_SENSOR_ID_PICKUP_GESTURE           UINT8_C(61)  /* Pickup gesture */
#define BHY2_SENSOR_ID_AR                       UINT8_C(63)  /* Activity recognition */
#define BHY2_SENSOR_ID_WRIST_TILT_GESTURE       UINT8_C(67)  /* Wrist tilt gesture */
#define BHY2_SENSOR_ID_DEVICE_ORI               UINT8_C(69)  /* Device orientation */
#define BHY2_SENSOR_ID_DEVICE_ORI_WU            UINT8_C(70)  /* Device orientation wake up */
#define BHY2_SENSOR_ID_STATIONARY_DET           UINT8_C(75)  /* Stationary detect */
#define BHY2_SENSOR_ID_MOTION_DET               UINT8_C(77)  /* Motion detect */
#define BHY2_SENSOR_ID_ACC_BIAS_WU              UINT8_C(91)  /* Accelerometer offset wake up */
#define BHY2_SENSOR_ID_GYRO_BIAS_WU             UINT8_C(92)  /* Gyroscope offset wake up */
#define BHY2_SENSOR_ID_MAG_BIAS_WU              UINT8_C(93)  /* Magnetometer offset wake up */
#define BHY2_SENSOR_ID_STD_WU                   UINT8_C(94)  /* Step detector wake up */
#define BHY2_SENSOR_ID_TEMP                     UINT8_C(128) /* Temperature */
#define BHY2_SENSOR_ID_BARO                     UINT8_C(129) /* Barometer */
#define BHY2_SENSOR_ID_HUM                      UINT8_C(130) /* Humidity */
#define BHY2_SENSOR_ID_GAS                      UINT8_C(131) /* Gas */
#define BHY2_SENSOR_ID_TEMP_WU                  UINT8_C(132) /* Temperature wake up */
#define BHY2_SENSOR_ID_BARO_WU                  UINT8_C(133) /* Barometer wake up */
#define BHY2_SENSOR_ID_HUM_WU                   UINT8_C(134) /* Humidity wake up */
#define BHY2_SENSOR_ID_GAS_WU                   UINT8_C(135) /* Gas wake up */
#define BHY2_SENSOR_ID_STC_HW                   UINT8_C(136) /* Hardware Step counter */
#define BHY2_SENSOR_ID_STD_HW                   UINT8_C(137) /* Hardware Step detector */
#define BHY2_SENSOR_ID_SIG_HW                   UINT8_C(138) /* Hardware Significant motion */
#define BHY2_SENSOR_ID_STC_HW_WU                UINT8_C(139) /* Hardware Step counter wake up */
#define BHY2_SENSOR_ID_STD_HW_WU                UINT8_C(140) /* Hardware Step detector wake up */
#define BHY2_SENSOR_ID_SIG_HW_WU                UINT8_C(141) /* Hardware Significant motion wake up */
#define BHY2_SENSOR_ID_ANY_MOTION               UINT8_C(142) /* Any motion */
#define BHY2_SENSOR_ID_ANY_MOTION_WU            UINT8_C(143) /* Any motion wake up */
#define BHY2_SENSOR_ID_EXCAMERA                 UINT8_C(144) /* External camera trigger */
#define BHY2_SENSOR_ID_GPS                      UINT8_C(145) /* GPS */
#define BHY2_SENSOR_ID_LIGHT                    UINT8_C(146) /* Light */
#define BHY2_SENSOR_ID_PROX                     UINT8_C(147) /* Proximity */
#define BHY2_SENSOR_ID_LIGHT_WU                 UINT8_C(148) /* Light wake up */
#define BHY2_SENSOR_ID_PROX_WU                  UINT8_C(149) /* Proximity wake up */

/*! System data IDs */
#define BHY2_IS_SYS_ID(sid) ((sid) >= 224)

#define BHY2_SYS_ID_PADDING                     UINT8_C(0)
#define BHY2_SYS_ID_TS_SMALL_DELTA              UINT8_C(251)
#define BHY2_SYS_ID_TS_LARGE_DELTA              UINT8_C(252)
#define BHY2_SYS_ID_TS_FULL                     UINT8_C(253)
#define BHY2_SYS_ID_META_EVENT                  UINT8_C(254)
#define BHY2_SYS_ID_TS_SMALL_DELTA_WU           UINT8_C(245)
#define BHY2_SYS_ID_TS_LARGE_DELTA_WU           UINT8_C(246)
#define BHY2_SYS_ID_TS_FULL_WU                  UINT8_C(247)
#define BHY2_SYS_ID_META_EVENT_WU               UINT8_C(248)
#define BHY2_SYS_ID_FILLER                      UINT8_C(255)
#define BHY2_SYS_ID_DEBUG_MSG                   UINT8_C(250)
#define BHY2_SYS_ID_BHY2_LOG_UPDATE_SUB         UINT8_C(243)
#define BHY2_SYS_ID_BHY2_LOG_DOSTEP             UINT8_C(244)

// Macro that returns if the data ID corresponds to a Meta Event
#define BHY2_SYS_ID_isMetaEvent(id)             (id == BHY2_SYS_ID_META_EVENT || id == BHY2_SYS_ID_META_EVENT_WU)

/*! Meta event definitions */
#define BHY2_META_EVENT_FLUSH_COMPLETE          (1)
#define BHY2_META_EVENT_SAMPLE_RATE_CHANGED     (2)
#define BHY2_META_EVENT_POWER_MODE_CHANGED      (3)
#define BHY2_META_EVENT_ALGORITHM_EVENTS        (5)
#define BHY2_META_EVENT_SENSOR_STATUS           (6)
#define BHY2_META_EVENT_BSX_DO_STEPS_MAIN       (7)
#define BHY2_META_EVENT_BSX_DO_STEPS_CALIB      (8)
#define BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL   (9)
#define BHY2_META_EVENT_RESERVED1               (10)
#define BHY2_META_EVENT_SENSOR_ERROR            (11)
#define BHY2_META_EVENT_FIFO_OVERFLOW           (12)
#define BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED   (13)
#define BHY2_META_EVENT_FIFO_WATERMARK          (14)
#define BHY2_META_EVENT_RESERVED2               (15)
#define BHY2_META_EVENT_INITIALIZED             (16)
#define BHY2_META_TRANSFER_CAUSE                (17)
#define BHY2_META_EVENT_SENSOR_FRAMEWORK        (18)
#define BHY2_META_EVENT_RESET                   (19)
#define BHY2_META_EVENT_SPACER                  (20)

#define MetaEvent_isSpacerOrOverflow(id)        (id == BHY2_META_EVENT_SPACER || id == BHY2_META_EVENT_FIFO_OVERFLOW)

/*! System parameters */
#define BHY2_PARAM_READ_MASK                    UINT16_C(0x1000)
#define BHY2_PARAM_WRITE_MASK                   UINT16_C(0x0000)
#define BHY2_PARAM_FIFO_CTRL                    UINT16_C(0x103)
#define BHY2_PARAM_SYS_VIRT_SENSOR_PRESENT      UINT16_C(0x11F)
#define BHY2_PARAM_SYS_PHYS_SENSOR_PRESENT      UINT16_C(0x120)
#define BHY2_PARAM_PHYSICAL_SENSOR_BASE         UINT16_C(0x120)
#define BHY2_PARAM_CALIB_STATE_BASE             UINT16_C(0x200)
#define BHY2_PARAM_SIC                          UINT16_C(0x27D)
#define BHY2_PARAM_BHY2_BSX_VERSION             UINT16_C(0x27E)
#define BHY2_PARAM_SET_SENSOR_CTRL              UINT16_C(0x0E00)
#define BHY2_PARAM_GET_SENSOR_CTRL              UINT16_C(0x1E00)
#define BHY2_PARAM_SENSOR_CTRL_FOC              UINT8_C(0x1)
#define BHY2_PARAM_SENSOR_CTRL_OIS              UINT8_C(0x2)
#define BHY2_PARAM_SENSOR_CTRL_FST              UINT8_C(0x3)
#define BHY2_PARAM_SENSOR_CTRL_READ             UINT8_C(0x80)
#define BHY2_PARAM_SENSOR_INFO_0                UINT16_C(0x300)
#define BHY2_PARAM_SENSOR_CONF_0                UINT16_C(0x500)

/* Physical sensor IDs */
#define BHY2_PHYS_SENS_ID_ACCEL                 UINT8_C(0x1) 
#define BHY2_PHYS_SENS_ID_GYRO                  UINT8_C(0x3)
#define BHY2_PHYS_SENS_ID_MAG                  	UINT8_C(0x5)

#endif /* INC_IMU_BHI260_DEFS_H_ */
