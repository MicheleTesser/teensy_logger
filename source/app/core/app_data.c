#include "app/app_shared.h"


usb_cdc_shell_state_t s_cdcState;
usb_cdc_acm_info_t s_cdcAcmInfo;

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t s_lineCoding[LINE_CODING_SIZE] = {
    (LINE_CODING_DTERATE >> 0U) & 0xFFU,
    (LINE_CODING_DTERATE >> 8U) & 0xFFU,
    (LINE_CODING_DTERATE >> 16U) & 0xFFU,
    (LINE_CODING_DTERATE >> 24U) & 0xFFU,
    LINE_CODING_CHARFORMAT,
    LINE_CODING_PARITYTYPE,
    LINE_CODING_DATABITS,
};

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t s_abstractState[COMM_FEATURE_DATA_SIZE] = {
    (STATUS_ABSTRACT_STATE >> 0U) & 0xFFU,
    (STATUS_ABSTRACT_STATE >> 8U) & 0xFFU,
};

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t s_countryCode[COMM_FEATURE_DATA_SIZE] = {
    (COUNTRY_SETTING >> 0U) & 0xFFU,
    (COUNTRY_SETTING >> 8U) & 0xFFU,
};

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t s_recvBuffer[DATA_BUFF_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t s_sendBuffer[DATA_BUFF_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t s_gsCanOutBuffer[GS_USB_MAX_PACKET_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) gs_host_frame_classic_t s_gsCanInFrames[GS_USB_MAX_FRAMES_PER_TRANSFER];

volatile uint32_t s_recvSize = 0U;
volatile uint8_t s_txBusy = 0U;
volatile uint8_t s_shellSessionReady = 0U;
volatile led_mode_t s_ledMode = kLedModeBlink;
volatile TickType_t s_ledActivityLastTick = 0U;
volatile bool s_ledActivitySeen = false;
volatile bool s_cdcOutPrimed = false;
volatile uint32_t s_gsCanOutSize = 0U;
volatile bool s_rtcInitialized = false;
volatile bool s_bootloaderRebootPending = false;
volatile TickType_t s_bootloaderRebootDeadline = 0U;
bool s_shellCliReady = false;
TickType_t s_canUtilLastTick = 0U;
bool s_canUtilInitialized = false;
uint32_t s_canUtilLastTxBits[2] = {0U, 0U};
uint32_t s_canUtilLastRxBits[2] = {0U, 0U};

FATFS s_fsObject;
bool s_fsConfigured = false;
bool s_fsMounted = false;
const TCHAR s_fsDrive[] = {SDDISK + '0', ':', '/', '\0'};
FIL s_canLogFile;
volatile bool s_canLogFileOpen = false;
char s_canLogActivePath[CAN_LOG_PATH_MAX] = {0};
uint32_t s_canLogNextIndex = 0U;
mf4_log_state_t s_mf4LogState;
sd_usage_cache_t s_sdUsageCache = {.valid = false, .lastResult = FR_NOT_READY};

const mf4_channel_desc_t s_mf4Channels[MF4_CHANNEL_COUNT] = {
    {.name = "timestamp_ns",
     .channelType = MF4_CHANNEL_TYPE_MASTER,
     .syncType = MF4_SYNC_TYPE_TIME,
     .dataType = MF4_DATA_TYPE_UNSIGNED_LE,
     .byteOffset = 0U,
     .bitCount = 64U},
    {.name = "data64",
     .channelType = MF4_CHANNEL_TYPE_VALUE,
     .syncType = MF4_SYNC_TYPE_NONE,
     .dataType = MF4_DATA_TYPE_UNSIGNED_LE,
     .byteOffset = 8U,
     .bitCount = 64U},
    {.name = "can_id",
     .channelType = MF4_CHANNEL_TYPE_VALUE,
     .syncType = MF4_SYNC_TYPE_NONE,
     .dataType = MF4_DATA_TYPE_UNSIGNED_LE,
     .byteOffset = 16U,
     .bitCount = 32U},
    {.name = "dlc",
     .channelType = MF4_CHANNEL_TYPE_VALUE,
     .syncType = MF4_SYNC_TYPE_NONE,
     .dataType = MF4_DATA_TYPE_UNSIGNED_LE,
     .byteOffset = 20U,
     .bitCount = 8U},
    {.name = "channel",
     .channelType = MF4_CHANNEL_TYPE_VALUE,
     .syncType = MF4_SYNC_TYPE_NONE,
     .dataType = MF4_DATA_TYPE_UNSIGNED_LE,
     .byteOffset = 21U,
     .bitCount = 8U},
    {.name = "flags",
     .channelType = MF4_CHANNEL_TYPE_VALUE,
     .syncType = MF4_SYNC_TYPE_NONE,
     .dataType = MF4_DATA_TYPE_UNSIGNED_LE,
     .byteOffset = 22U,
     .bitCount = 8U},
};

can_bus_context_t s_canBuses[] = {
    {.base = CAN1,
     .heartbeatMb = CAN_HEARTBEAT_MB,
     .shellTxMb = CAN_SHELL_TX_MB,
     .rxStdMb = CAN_RX_STD_MB,
     .rxExtMb = CAN_RX_EXT_MB,
     .heartbeatId = CAN1_HEARTBEAT_ID},
    {.base = CAN2,
     .heartbeatMb = CAN_HEARTBEAT_MB,
     .shellTxMb = CAN_SHELL_TX_MB,
     .rxStdMb = CAN_RX_STD_MB,
     .rxExtMb = CAN_RX_EXT_MB,
     .heartbeatId = CAN2_HEARTBEAT_ID},
};
volatile bool s_canReady = false;
volatile bool s_canHeartbeatEnabled = false;
volatile bool s_canDumpEnabled = false;
volatile uint32_t s_canBitRate = CAN_DEFAULT_BITRATE;
volatile uint32_t s_canModeFlags = 0U;
volatile bool s_canAutoStartEnabled = true;
volatile bool s_logEnabled = true;
volatile bool s_logAllowWhenUsbAttached = false;
volatile bool s_mscHostActive = false;

QueueHandle_t s_canLogQueue = NULL;
SemaphoreHandle_t s_shellTxMutex = NULL;

volatile bool s_gsCanConfigured = false;
volatile bool s_gsCanHostLe = true;
volatile bool s_gsCanInBusy = false;
volatile bool s_gsCanOutPrimed = false;
volatile bool s_gsCanEpReady = false;
volatile uint16_t s_gsCanTxHead = 0U;
volatile uint16_t s_gsCanTxTail = 0U;
volatile uint16_t s_gsCanTxCount = 0U;
volatile bool s_gsCanOverflowPending[GS_USB_CHANNEL_COUNT] = {false, false};
gs_host_frame_classic_t s_gsCanTxQueue[GS_USB_FRAME_QUEUE_LENGTH];
gs_can_deferred_request_t s_gsCanDeferredReq[GS_USB_CHANNEL_COUNT];

volatile uint16_t s_canRxFifoHead[GS_USB_CHANNEL_COUNT] = {0U, 0U};
volatile uint16_t s_canRxFifoTail[GS_USB_CHANNEL_COUNT] = {0U, 0U};
volatile bool s_canRxFifoOverflow[GS_USB_CHANNEL_COUNT] = {false, false};
can_isr_rx_entry_t s_canRxFifo[GS_USB_CHANNEL_COUNT][CAN_ISR_RX_FIFO_LENGTH];
volatile uint16_t s_canGsTxHead[GS_USB_CHANNEL_COUNT] = {0U, 0U};
volatile uint16_t s_canGsTxTail[GS_USB_CHANNEL_COUNT] = {0U, 0U};
volatile uint16_t s_canGsTxCount[GS_USB_CHANNEL_COUNT] = {0U, 0U};
volatile uint8_t s_canGsTxMbCursor[GS_USB_CHANNEL_COUNT] = {0U, 0U};
can_gs_tx_entry_t s_canGsTxQueue[GS_USB_CHANNEL_COUNT][CAN_GS_TX_QUEUE_LENGTH];

gs_host_config_t s_gsHostConfig;
gs_device_bittiming_t s_gsBitTiming;
gs_device_mode_t s_gsModeRequest;
uint32_t s_gsBerrRequest;
volatile uint32_t s_gsCanBtClockHz = 0U;
gs_device_config_t s_gsDeviceConfig;
gs_device_bt_const_t s_gsBtConst;
gs_device_bittiming_t s_gsBitTimingByChannel[GS_USB_CHANNEL_COUNT];
bool s_gsBitTimingValid[GS_USB_CHANNEL_COUNT] = {false, false};

usb_device_class_config_struct_t s_cdcAcmConfig[] = {{
                                                        USB_DeviceCdcVcomCallback,
                                                        0U,
                                                        &g_UsbDeviceCdcVcomConfig,
                                                    },
                                                    {
                                                        USB_DeviceMscCallback,
                                                        0U,
                                                        &g_UsbDeviceMscDiskConfig,
                                                    }};

usb_device_class_config_list_struct_t s_cdcAcmConfigList = {
    s_cdcAcmConfig,
    USB_DeviceCallback,
    2U,
};
