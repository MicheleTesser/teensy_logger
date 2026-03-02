/*
 * Teensy 4.1 SDMMC configuration for USDHC1 + FatFs.
 */

#ifndef _SDMMC_CONFIG_H_
#define _SDMMC_CONFIG_H_

#include "fsl_common.h"
#include "clock_config.h"
#include "fsl_sdmmc_host.h"

#ifdef SD_ENABLED
#include "fsl_sd.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_SDMMC_SD_HOST_BASEADDR USDHC1
#define BOARD_SDMMC_SD_HOST_IRQ      USDHC1_IRQn

/* Detect card using DAT3 since Teensy 4.1 does not expose EVKB GPIO CD logic. */
#define BOARD_SDMMC_SD_CD_TYPE                        kSD_DetectCardByHostDATA3
#define BOARD_SDMMC_SD_CARD_DETECT_DEBOUNCE_DELAY_MS (100U)

#define BOARD_SDMMC_SD_IO_VOLTAGE_CONTROL_TYPE kSD_IOVoltageCtrlByHost
#define BOARD_SDMMC_SD_HOST_SUPPORT_SDR104_FREQ (200000000U)

#define BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE            (32U)
#define BOARD_SDMMC_SD_HOST_IRQ_PRIORITY              (5U)
#define BOARD_SDMMC_HOST_DMA_DESCRIPTOR_BUFFER_SIZE   (32U)
#define BOARD_SDMMC_HOST_CACHE_CONTROL                kSDMMCHOST_CacheControlRWBuffer

#if defined(__cplusplus)
extern "C" {
#endif

uint32_t BOARD_USDHC1ClockConfiguration(void);

#ifdef SD_ENABLED
void BOARD_SD_Config(void *card, sd_cd_t cd, uint32_t hostIRQPriority, void *userData);
#endif

#if defined(__cplusplus)
}
#endif

#endif /* _SDMMC_CONFIG_H_ */
