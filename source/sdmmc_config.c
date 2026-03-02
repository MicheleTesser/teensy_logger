/*
 * Teensy 4.1 SDMMC configuration for USDHC1 + FatFs.
 */

#include "sdmmc_config.h"

#include "fsl_iomuxc.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
AT_NONCACHEABLE_SECTION_ALIGN(static uint32_t s_sdmmcHostDmaBuffer[BOARD_SDMMC_HOST_DMA_DESCRIPTOR_BUFFER_SIZE],
                              SDMMCHOST_DMA_DESCRIPTOR_BUFFER_ALIGN_SIZE);

#if defined(SDMMCHOST_ENABLE_CACHE_LINE_ALIGN_TRANSFER) && SDMMCHOST_ENABLE_CACHE_LINE_ALIGN_TRANSFER
SDK_ALIGN(static uint8_t s_sdmmcCacheLineAlignBuffer[BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE * 2U],
          BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE);
#endif

static sd_detect_card_t s_cd;
static sd_io_voltage_t s_ioVoltage = {
    .type = BOARD_SDMMC_SD_IO_VOLTAGE_CONTROL_TYPE,
    .func = NULL,
};
static sdmmchost_t s_host;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void BOARD_SDCardDAT3PullFunction(uint32_t status);
static void BOARD_SDCardDetectInit(sd_cd_t cd, void *userData);
static void BOARD_SD_IoStrength(uint32_t freq);

/*******************************************************************************
 * Code
 ******************************************************************************/
uint32_t BOARD_USDHC1ClockConfiguration(void)
{
    CLOCK_SetDiv(kCLOCK_Usdhc1Div, 1U);
    CLOCK_SetMux(kCLOCK_Usdhc1Mux, 0U);
    CLOCK_EnableClock(kCLOCK_Usdhc1);

    return CLOCK_GetClockRootFreq(kCLOCK_Usdhc1ClkRoot);
}

static void BOARD_SDCardDAT3PullFunction(uint32_t status)
{
    uint32_t padConfig;

    if (status == kSD_DAT3PullDown)
    {
        padConfig = IOMUXC_SW_PAD_CTL_PAD_SPEED(1) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK |
                    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                    IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(0) |
                    IOMUXC_SW_PAD_CTL_PAD_DSE(1);
    }
    else
    {
        padConfig = IOMUXC_SW_PAD_CTL_PAD_SPEED(1) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK |
                    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                    IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |
                    IOMUXC_SW_PAD_CTL_PAD_DSE(1);
    }

    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3, padConfig);
}

static void BOARD_SDCardDetectInit(sd_cd_t cd, void *userData)
{
    s_cd.cdDebounce_ms = BOARD_SDMMC_SD_CARD_DETECT_DEBOUNCE_DELAY_MS;
    s_cd.type          = BOARD_SDMMC_SD_CD_TYPE;
    s_cd.cardDetected  = NULL;
    s_cd.callback      = cd;
    s_cd.userData      = userData;
    s_cd.dat3PullFunc  = NULL;

    if (BOARD_SDMMC_SD_CD_TYPE == kSD_DetectCardByHostDATA3)
    {
        s_cd.dat3PullFunc = BOARD_SDCardDAT3PullFunction;
    }
}

static void BOARD_SD_IoStrength(uint32_t freq)
{
    uint32_t speed;
    uint32_t strength;

    if (freq <= 50000000U)
    {
        speed    = 0U;
        strength = 7U;
    }
    else if (freq <= 100000000U)
    {
        speed    = 2U;
        strength = 7U;
    }
    else
    {
        speed    = 3U;
        strength = 7U;
    }

    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_00_USDHC1_CMD,
                        IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |
                            IOMUXC_SW_PAD_CTL_PAD_DSE(strength));
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_01_USDHC1_CLK,
                        IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(0) |
                            IOMUXC_SW_PAD_CTL_PAD_DSE(strength));
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_02_USDHC1_DATA0,
                        IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |
                            IOMUXC_SW_PAD_CTL_PAD_DSE(strength));
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_03_USDHC1_DATA1,
                        IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |
                            IOMUXC_SW_PAD_CTL_PAD_DSE(strength));
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_04_USDHC1_DATA2,
                        IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |
                            IOMUXC_SW_PAD_CTL_PAD_DSE(strength));
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3,
                        IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                            IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |
                            IOMUXC_SW_PAD_CTL_PAD_DSE(strength));
}

#ifdef SD_ENABLED
void BOARD_SD_Config(void *card, sd_cd_t cd, uint32_t hostIRQPriority, void *userData)
{
    sd_card_t *sdCard = (sd_card_t *)card;

    assert(sdCard != NULL);

    s_host.dmaDesBuffer         = s_sdmmcHostDmaBuffer;
    s_host.dmaDesBufferWordsNum = BOARD_SDMMC_HOST_DMA_DESCRIPTOR_BUFFER_SIZE;
    s_host.enableCacheControl   = BOARD_SDMMC_HOST_CACHE_CONTROL;
#if defined(SDMMCHOST_ENABLE_CACHE_LINE_ALIGN_TRANSFER) && SDMMCHOST_ENABLE_CACHE_LINE_ALIGN_TRANSFER
    s_host.cacheAlignBuffer     = s_sdmmcCacheLineAlignBuffer;
    s_host.cacheAlignBufferSize = BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE * 2U;
#endif

    sdCard->host                                = &s_host;
    sdCard->host->hostController.base           = BOARD_SDMMC_SD_HOST_BASEADDR;
    sdCard->host->hostController.sourceClock_Hz = BOARD_USDHC1ClockConfiguration();

    BOARD_SDCardDetectInit(cd, userData);

    sdCard->usrParam.cd         = &s_cd;
    sdCard->usrParam.pwr        = NULL;
    sdCard->usrParam.ioStrength = BOARD_SD_IoStrength;
    sdCard->usrParam.ioVoltage  = &s_ioVoltage;
    sdCard->usrParam.maxFreq    = BOARD_SDMMC_SD_HOST_SUPPORT_SDR104_FREQ;

    NVIC_SetPriority(BOARD_SDMMC_SD_HOST_IRQ, hostIRQPriority);
}
#endif
