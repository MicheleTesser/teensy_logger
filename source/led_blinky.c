/*
 * FreeRTOS USB CDC shell for Teensy 4.1 (i.MX RT1062):
 * commands: help, led ..., fs ...
 */

#include <ctype.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "pin_mux.h"
#include "board.h"
#include "clock_config.h"
#include "system_MIMXRT1062.h"
#include "fsl_iomuxc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_class.h"
#include "usb_device_cdc_acm.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"
#include "usb_phy.h"

#include "ff.h"
#include "diskio.h"
#include "fsl_sd_disk.h"
#include "fsl_sdmmc_host.h"
#include "fsl_flexcan.h"
#include "sdmmc_config.h"

#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
#ifndef CONTROLLER_ID
#define CONTROLLER_ID kUSB_ControllerEhci0
#endif
#define DATA_BUFF_SIZE HS_CDC_VCOM_BULK_OUT_PACKET_SIZE
#else
#error "This application requires USB_DEVICE_CONFIG_EHCI=1."
#endif

#if defined(__GIC_PRIO_BITS)
#define USB_DEVICE_INTERRUPT_PRIORITY (25U)
#elif defined(__NVIC_PRIO_BITS) && (__NVIC_PRIO_BITS >= 3U)
#define USB_DEVICE_INTERRUPT_PRIORITY (6U)
#else
#define USB_DEVICE_INTERRUPT_PRIORITY (3U)
#endif

#define APP_TASK_STACK_WORDS 1280U
#define APP_TASK_PRIORITY    (tskIDLE_PRIORITY + 5U)
#define LED_TASK_STACK_WORDS 256U
#define LED_TASK_PRIORITY    (tskIDLE_PRIORITY + 2U)
#define CAN_TASK_STACK_WORDS 1280U
#define CAN_TASK_PRIORITY    (tskIDLE_PRIORITY + 1U)

#define LED_ACTIVITY_PULSE_MS   60U
#define CAN_HEARTBEAT_PERIOD_MS 100U
#define CAN_DUMP_POLL_PERIOD_MS 2U
#define CAN_DEFAULT_BITRATE     500000U
#define CAN_HEARTBEAT_MB        8U
#define CAN_SHELL_TX_MB         9U
#define CAN_RX_STD_MB           10U
#define CAN_RX_EXT_MB           11U
#define CAN1_HEARTBEAT_ID       0x321U
#define CAN2_HEARTBEAT_ID       0x322U

#define SHELL_PROMPT           "usb-shell> "
#define SHELL_LINE_BUFFER_SIZE 256U
#define SHELL_TX_LINE_SIZE     128U
#define SHELL_CLI_OUTPUT_SIZE  512U

#define FS_BENCH_FILE_PATH    "/bench.bin"
#define FS_BENCH_BUFFER_SIZE  4096U
#define FS_BENCH_DEFAULT_KIB  1024U
#define FS_BENCH_DEFAULT_RUNS 1U

#define CAN_UTIL_UPDATE_MS       200U

#define CAN_LOG_PATH                "/canlog.mf4"
#define CAN_LOG_TASK_STACK_WORDS    768U
#define CAN_LOG_TASK_PRIORITY       (tskIDLE_PRIORITY + 1U)
#define CAN_LOG_QUEUE_LENGTH        128U
#define CAN_LOG_FLUSH_PERIOD_MS     1000U
#define CAN_LOG_MOUNT_RETRY_MS      1000U
#define CAN_LOG_SYNC_FRAME_INTERVAL 64U
#define CAN_LOG_RECORD_SIZE_BYTES   24U

#define MF4_CHANNEL_COUNT           6U
#define MF4_ID_BLOCK_SIZE           64U
#define MF4_HD_BLOCK_SIZE           104U
#define MF4_FH_BLOCK_SIZE           56U
#define MF4_DG_BLOCK_SIZE           64U
#define MF4_CG_BLOCK_SIZE           104U
#define MF4_CN_BLOCK_SIZE           160U
#define MF4_BLOCK_COMMON_SIZE       24U
#define MF4_BLOCK_LINK_SIZE         8U
#define MF4_OFFSET_ID               0U
#define MF4_OFFSET_HD               (MF4_OFFSET_ID + MF4_ID_BLOCK_SIZE)
#define MF4_OFFSET_FH               (MF4_OFFSET_HD + MF4_HD_BLOCK_SIZE)
#define MF4_OFFSET_DG               (MF4_OFFSET_FH + MF4_FH_BLOCK_SIZE)
#define MF4_OFFSET_CG               (MF4_OFFSET_DG + MF4_DG_BLOCK_SIZE)
#define MF4_OFFSET_CN0              (MF4_OFFSET_CG + MF4_CG_BLOCK_SIZE)
#define MF4_CG_CYCLES_FIELD_OFFSET  (MF4_BLOCK_COMMON_SIZE + (6U * MF4_BLOCK_LINK_SIZE) + 8U)
#define MF4_DT_LEN_FIELD_OFFSET     8U
#define MF4_TX_BLOCK_ID             "##TX"
#define MF4_ID_BLOCK_MAGIC          "MDF     "
#define MF4_VERSION_TEXT            "4.10    "
#define MF4_PROGRAM_TEXT            "TEENLOG "
#define MF4_VERSION_NUM             410U
#define MF4_CN_FLAGS_DEFAULT        0x1CU

#define MF4_CHANNEL_TYPE_VALUE      0U
#define MF4_CHANNEL_TYPE_MASTER     2U
#define MF4_SYNC_TYPE_NONE          0U
#define MF4_SYNC_TYPE_TIME          1U
#define MF4_DATA_TYPE_UNSIGNED_LE   0U

#define RTC_COUNTER_SHIFT            15U
#define RTC_COUNTER_SYNC_RETRY       100U
#define RTC_ENABLE_TIMEOUT_LOOPS     1000000U
#define RTC_VALID_GPR_MAGIC          0x52544331UL /* RTC1 */
#define RTC_DEFAULT_FAT_YEAR         2026U
#define RTC_SECONDS_PER_MINUTE       60U
#define RTC_SECONDS_PER_HOUR         3600U
#define RTC_SECONDS_PER_DAY          86400U
#define RTC_DAYS_PER_NORMAL_YEAR     365U
#define RTC_DAYS_PER_LEAP_YEAR       366U

#define CAN_EFF_FLAG 0x80000000UL
#define CAN_RTR_FLAG 0x40000000UL
#define CAN_ERR_FLAG 0x20000000UL
#define CAN_SFF_MASK 0x000007FFUL
#define CAN_EFF_MASK 0x1FFFFFFFUL

#define GS_USB_BREQ_HOST_FORMAT   0U
#define GS_USB_BREQ_BITTIMING     1U
#define GS_USB_BREQ_MODE          2U
#define GS_USB_BREQ_BERR          3U
#define GS_USB_BREQ_BT_CONST      4U
#define GS_USB_BREQ_DEVICE_CONFIG 5U

#define GS_CAN_MODE_RESET 0U
#define GS_CAN_MODE_START 1U

#define GS_CAN_MODE_LISTEN_ONLY   (1UL << 0)
#define GS_CAN_MODE_LOOP_BACK     (1UL << 1)
#define GS_CAN_MODE_TRIPLE_SAMPLE (1UL << 2)
#define GS_CAN_MODE_ONE_SHOT      (1UL << 3)
#define GS_CAN_MODE_BERR_REPORT   (1UL << 12)
#define GS_CAN_MODE_SUPPORTED_MASK (GS_CAN_MODE_LISTEN_ONLY | GS_CAN_MODE_LOOP_BACK)

#define GS_CAN_FEATURE_LISTEN_ONLY   (1UL << 0)
#define GS_CAN_FEATURE_LOOP_BACK     (1UL << 1)
#define GS_CAN_FEATURE_TRIPLE_SAMPLE (1UL << 2)
#define GS_CAN_FEATURE_ONE_SHOT      (1UL << 3)
#define GS_CAN_FEATURE_BERR_REPORT   (1UL << 12)

#define GS_CAN_FLAG_OVERFLOW (1U << 0)

#define GS_HOST_CONFIG_LITTLE_ENDIAN 0x0000BEEFUL
#define GS_HOST_FRAME_ECHO_ID_RX     0xFFFFFFFFUL
#define GS_USB_CHANNEL_COUNT         2U
#define GS_USB_MAX_PACKET_SIZE       HS_GS_CAN_BULK_OUT_PACKET_SIZE
#define GS_USB_FRAME_QUEUE_LENGTH    256U

#define GS_CAN_DEFERRED_OP_NONE  (0U)
#define GS_CAN_DEFERRED_OP_START (1U)
#define GS_CAN_DEFERRED_OP_RESET (2U)

#if defined(__GNUC__)
#define PACKED_STRUCT __attribute__((packed))
#else
#define PACKED_STRUCT
#endif

/* CDC line coding defaults (115200 8N1). */
#define LINE_CODING_SIZE       (0x07U)
#define LINE_CODING_DTERATE    (115200U)
#define LINE_CODING_CHARFORMAT (0x00U)
#define LINE_CODING_PARITYTYPE (0x00U)
#define LINE_CODING_DATABITS   (0x08U)

#define COMM_FEATURE_DATA_SIZE (0x02U)
#define STATUS_ABSTRACT_STATE  (0x0000U)
#define COUNTRY_SETTING        (0x0000U)

#define NOTIF_PACKET_SIZE (0x08U)
#define UART_BITMAP_SIZE  (0x02U)
#define NOTIF_REQUEST_TYPE (0xA1U)

/*******************************************************************************
 * Types
 ******************************************************************************/
typedef struct
{
    usb_device_handle deviceHandle;
    class_handle_t cdcAcmHandle;
    volatile uint8_t attach;
    uint8_t speed;
    volatile uint8_t startTransactions;
    uint8_t currentConfiguration;
    uint8_t currentInterfaceAlternateSetting[USB_DEVICE_INTERFACE_COUNT];
} usb_cdc_shell_state_t;

typedef struct
{
    uint8_t serialStateBuf[NOTIF_PACKET_SIZE + UART_BITMAP_SIZE];
    bool dtePresent;
    uint16_t breakDuration;
    uint8_t dteStatus;
    uint8_t currentInterface;
    uint16_t uartState;
} usb_cdc_acm_info_t;

typedef enum
{
    kLedModeBlink = 0U,
    kLedModeOn,
    kLedModeOff
} led_mode_t;

typedef struct
{
    CAN_Type *base;
    uint8_t heartbeatMb;
    uint8_t shellTxMb;
    uint8_t rxStdMb;
    uint8_t rxExtMb;
    uint32_t heartbeatId;
    volatile uint32_t txOkCount;
    volatile uint32_t txErrCount;
    volatile uint32_t rxOkCount;
    volatile uint32_t rxOverflowCount;
    volatile uint32_t txBitCount;
    volatile uint32_t rxBitCount;
    volatile uint16_t utilTxPermille;
    volatile uint16_t utilRxPermille;
    volatile uint16_t utilTotalPermille;
    volatile uint16_t utilWindowMs;
    volatile uint32_t bitRate;
    volatile uint32_t modeFlags;
    volatile bool started;
} can_bus_context_t;

typedef struct PACKED_STRUCT
{
    uint32_t byteOrder;
} gs_host_config_t;

typedef struct PACKED_STRUCT
{
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t icount;
    uint32_t swVersion;
    uint32_t hwVersion;
} gs_device_config_t;

typedef struct PACKED_STRUCT
{
    uint32_t feature;
    uint32_t fclkCan;
    uint32_t tseg1Min;
    uint32_t tseg1Max;
    uint32_t tseg2Min;
    uint32_t tseg2Max;
    uint32_t sjwMax;
    uint32_t brpMin;
    uint32_t brpMax;
    uint32_t brpInc;
} gs_device_bt_const_t;

typedef struct PACKED_STRUCT
{
    uint32_t propSeg;
    uint32_t phaseSeg1;
    uint32_t phaseSeg2;
    uint32_t sjw;
    uint32_t brp;
} gs_device_bittiming_t;

typedef struct PACKED_STRUCT
{
    uint32_t mode;
    uint32_t flags;
} gs_device_mode_t;

typedef struct PACKED_STRUCT
{
    uint32_t echoId;
    uint32_t canId;
    uint8_t canDlc;
    uint8_t channel;
    uint8_t flags;
    uint8_t reserved;
    uint8_t data[8];
} gs_host_frame_classic_t;

typedef struct
{
    uint64_t timestampNs;
    uint64_t data64;
    uint32_t canId;
    uint8_t dlc;
    uint8_t channel;
    uint8_t frameFlags;
    uint8_t reserved;
} can_log_record_t;
typedef char can_log_record_size_must_be_24[(sizeof(can_log_record_t) == CAN_LOG_RECORD_SIZE_BYTES) ? 1 : -1];

typedef struct
{
    const char *name;
    uint8_t channelType;
    uint8_t syncType;
    uint8_t dataType;
    uint32_t byteOffset;
    uint32_t bitCount;
} mf4_channel_desc_t;

typedef struct
{
    bool ready;
    uint64_t recordCount;
    uint64_t dataBytes;
    uint32_t cgBlockOffset;
    uint32_t dtBlockOffset;
    uint32_t dtDataOffset;
} mf4_log_state_t;

typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} rtc_datetime_t;

typedef struct
{
    volatile uint32_t op;
    volatile uint32_t bitRate;
    volatile uint32_t flags;
    volatile uint8_t pending;
} gs_can_deferred_request_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void AppTask(void *pvParameters);
static void LedTask(void *pvParameters);
static void CanTask(void *pvParameters);
static void CanLogTask(void *pvParameters);
static void USB_DeviceApplicationInit(void);
static void USB_DeviceClockInit(void);
static void USB_DeviceIsrEnable(void);
static void ShellHandlePacket(const uint8_t *data, uint32_t length);
static void ShellHandleLine(char *line);
static void ShellSendPrompt(void);
static void ShellWrite(const char *text);
static void ShellWriteBytes(const uint8_t *data, uint32_t length);
static bool ShellWriteTry(const char *text);
static bool ShellWriteBytesTry(const uint8_t *data, uint32_t length);
static void ShellWriteLinef(const char *fmt, ...);
static void ShellCliInit(void);
static BaseType_t ShellCliHelpCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t ShellCliLedCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t ShellCliCanCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t ShellCliSdCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t ShellCliRtcCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static void ShellPrintHelp(void);
static void LedMarkCanActivity(void);

static FRESULT FsMount(void);
static FRESULT FsUnmount(void);
static bool FsEnsureMounted(void);
static const char *FsResultToString(FRESULT result);
static bool FsParseU32(const char *text, uint32_t *value);
static void FsCommand(int32_t argc, char *argv[], const char *rawLine);
static void FsCommandList(const char *path);
static void FsCommandCat(const char *path);
static void FsCommandWrite(const char *path, const char *payload, bool append);
static void FsCommandMkDir(const char *path);
static void FsCommandRemove(const char *path);
static void FsCommandMkfs(void);
static void FsCommandBench(uint32_t totalKiB, uint32_t runs);
static const char *ShellSkipToken(const char *cursor);
static bool ShellParseU32Auto(const char *text, uint32_t *value);
static bool ShellParseCanBus(const char *text, uint32_t *busIndex);
static bool RtcInit(void);
static bool RtcIsValid(void);
static uint32_t RtcGetUnixSeconds(void);
static bool RtcSetUnixSeconds(uint32_t unixSeconds);
static bool RtcUnixSecondsToDateTime(uint32_t unixSeconds, rtc_datetime_t *dateTime);
static void RtcCommand(int32_t argc, char *argv[]);

static status_t CanInitAll(uint32_t bitRate, uint32_t modeFlags);
static status_t CanSendFromShell(uint32_t busIndex, uint32_t id, const uint8_t *payload, uint8_t length);
static void CanCommand(int32_t argc, char *argv[]);
static status_t CanReleaseStopMode(CAN_Type *base);
static status_t CanInitOneWithFlags(can_bus_context_t *bus, uint32_t bitRate, uint32_t modeFlags);
static status_t CanDeinitOne(can_bus_context_t *bus);
static void CanPollRxDump(void);
static void CanPollRxMb(can_bus_context_t *bus, size_t busIndex, uint8_t mbIdx);
static void CanHandleRxFrame(size_t busIndex, const flexcan_frame_t *frame, status_t status);
static void CanDumpPrintFrame(size_t busIndex, const flexcan_frame_t *frame, status_t status);
static uint32_t CanEstimateFrameBits(const flexcan_frame_t *frame);
static void CanUpdateUtilization(TickType_t now);
static void CanResetRuntimeStats(void);
static status_t CanWriteTxMbWithRecovery(can_bus_context_t *bus, uint8_t mbIdx, const flexcan_frame_t *frame);

static uint32_t GsBswap32(uint32_t value);
static uint32_t GsToLe32(uint32_t value);
static uint32_t GsFromLe32(uint32_t value);
static bool GsIsLittleEndianHost(void);
static bool GsCanQueueFrame(const gs_host_frame_classic_t *frame);
static void GsCanTrySendNext(void);
static void CanRefreshReadyState(void);
static bool CanAnyControllerStarted(void);
static void CanApplyGsClockRoot(void);
static void GsCanHandleBulkOutPacket(const uint8_t *data, uint32_t length);
static bool GsCanFillBtConst(uint8_t channel, gs_device_bt_const_t *btConst);
static uint8_t GsCanDecodeChannelFromSetup(const usb_setup_struct_t *setup);
static bool GsCanHandleVendorRequest(usb_device_control_request_struct_t *controlRequest);
static void GsCanProcessDeferredRequests(void);
static void GsCanSetOverflowFlag(size_t busIndex);
static bool GsCanRxPublish(size_t busIndex, const flexcan_frame_t *frame, status_t status);
static bool GsCanTxEchoPublish(size_t busIndex, uint32_t echoId, const flexcan_frame_t *frame);
static usb_status_t USB_GsCanBulkOutCallback(usb_device_handle handle,
                                             usb_device_endpoint_callback_message_struct_t *message,
                                             void *callbackParam);
static usb_status_t USB_GsCanBulkInCallback(usb_device_handle handle,
                                            usb_device_endpoint_callback_message_struct_t *message,
                                            void *callbackParam);

static uint32_t Mf4Align8(uint32_t value);
static void Mf4PutU16(uint8_t *dst, uint16_t value);
static void Mf4PutU32(uint8_t *dst, uint32_t value);
static void Mf4PutU64(uint8_t *dst, uint64_t value);
static void Mf4PutS16(uint8_t *dst, int16_t value);
static void Mf4PutDouble(uint8_t *dst, double value);
static bool Mf4WriteAt(FIL *file, FSIZE_t offset, const void *data, UINT size);
static bool Mf4ReadAt(FIL *file, FSIZE_t offset, void *data, UINT size);
static uint64_t Mf4UnixSecondsToNano(uint32_t unixSeconds);
static void Mf4ComputeLayout(uint32_t *txOffsets, uint32_t *dtBlockOffset, uint32_t *dtDataOffset);
static bool Mf4WriteTextBlock(FIL *file, uint32_t offset, const char *text);
static bool Mf4WriteFreshFile(FIL *file, mf4_log_state_t *state);
static bool Mf4RecoverFile(FIL *file, mf4_log_state_t *state);
static bool Mf4PatchCounters(FIL *file, const mf4_log_state_t *state);
static bool Mf4Sync(FIL *file, const mf4_log_state_t *state);

static bool CanLogEnqueue(size_t busIndex, const flexcan_frame_t *frame, status_t status);

static usb_status_t USB_DeviceCdcVcomCallback(class_handle_t handle, uint32_t event, void *param);
static usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param);

DWORD get_fattime(void);

/*******************************************************************************
 * Globals
 ******************************************************************************/
extern usb_device_endpoint_struct_t g_UsbDeviceCdcVcomDicEndpoints[];
extern usb_device_endpoint_struct_t g_UsbDeviceGsCanEndpoints[];
extern usb_device_class_struct_t g_UsbDeviceCdcVcomConfig;
extern void USDHC1_DriverIRQHandler(void);

static usb_cdc_shell_state_t s_cdcState;
static usb_cdc_acm_info_t s_cdcAcmInfo;

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint8_t s_lineCoding[LINE_CODING_SIZE] = {
    (LINE_CODING_DTERATE >> 0U) & 0xFFU,
    (LINE_CODING_DTERATE >> 8U) & 0xFFU,
    (LINE_CODING_DTERATE >> 16U) & 0xFFU,
    (LINE_CODING_DTERATE >> 24U) & 0xFFU,
    LINE_CODING_CHARFORMAT,
    LINE_CODING_PARITYTYPE,
    LINE_CODING_DATABITS,
};

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint8_t s_abstractState[COMM_FEATURE_DATA_SIZE] = {
    (STATUS_ABSTRACT_STATE >> 0U) & 0xFFU,
    (STATUS_ABSTRACT_STATE >> 8U) & 0xFFU,
};

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint8_t s_countryCode[COMM_FEATURE_DATA_SIZE] = {
    (COUNTRY_SETTING >> 0U) & 0xFFU,
    (COUNTRY_SETTING >> 8U) & 0xFFU,
};

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_recvBuffer[DATA_BUFF_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_sendBuffer[DATA_BUFF_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_gsCanOutBuffer[GS_USB_MAX_PACKET_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static gs_host_frame_classic_t s_gsCanInFrame;

static volatile uint32_t s_recvSize = 0U;
static volatile uint8_t s_txBusy = 0U;
static volatile uint8_t s_shellSessionReady = 0U;
static volatile led_mode_t s_ledMode = kLedModeBlink;
static volatile TickType_t s_ledActivityLastTick = 0U;
static volatile bool s_ledActivitySeen = false;
static volatile bool s_cdcOutPrimed = false;
static volatile uint32_t s_gsCanOutSize = 0U;
static volatile bool s_rtcInitialized = false;
static bool s_shellProductionMode = true;
static bool s_shellCliReady = false;
static TickType_t s_canUtilLastTick = 0U;
static bool s_canUtilInitialized = false;
static uint32_t s_canUtilLastTxBits[2] = {0U, 0U};
static uint32_t s_canUtilLastRxBits[2] = {0U, 0U};

static const CLI_Command_Definition_t s_cliHelpCommand = {
    .pcCommand = "help",
    .pcHelpString = "help: mostra i comandi disponibili\r\n",
    .pxCommandInterpreter = ShellCliHelpCommand,
    .cExpectedNumberOfParameters = 0,
};

static const CLI_Command_Definition_t s_cliLedCommand = {
    .pcCommand = "led",
    .pcHelpString = "led <status|activity|on|off>\r\n",
    .pxCommandInterpreter = ShellCliLedCommand,
    .cExpectedNumberOfParameters = -1,
};

static const CLI_Command_Definition_t s_cliCanCommand = {
    .pcCommand = "can",
    .pcHelpString = "can <status|util|clear>\r\n",
    .pxCommandInterpreter = ShellCliCanCommand,
    .cExpectedNumberOfParameters = -1,
};

static const CLI_Command_Definition_t s_cliSdCommand = {
    .pcCommand = "sd",
    .pcHelpString = "sd status: spazio SD totale/libero/usato\r\n",
    .pxCommandInterpreter = ShellCliSdCommand,
    .cExpectedNumberOfParameters = -1,
};

static const CLI_Command_Definition_t s_cliRtcCommand = {
    .pcCommand = "rtc",
    .pcHelpString = "rtc <status|set <unix_epoch>>\r\n",
    .pxCommandInterpreter = ShellCliRtcCommand,
    .cExpectedNumberOfParameters = -1,
};

static FATFS s_fsObject;
static bool s_fsConfigured = false;
static bool s_fsMounted = false;
static const TCHAR s_fsDrive[] = {SDDISK + '0', ':', '/', '\0'};
static FIL s_canLogFile;
static bool s_canLogFileOpen = false;
static mf4_log_state_t s_mf4LogState;

static const mf4_channel_desc_t s_mf4Channels[MF4_CHANNEL_COUNT] = {
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

static can_bus_context_t s_canBuses[] = {
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
static volatile bool s_canReady = false;
static volatile bool s_canHeartbeatEnabled = false;
static volatile bool s_canDumpEnabled = false;
static volatile uint32_t s_canBitRate = CAN_DEFAULT_BITRATE;
static volatile uint32_t s_canModeFlags = 0U;

static QueueHandle_t s_canLogQueue = NULL;
static SemaphoreHandle_t s_shellTxMutex = NULL;

static volatile bool s_gsCanConfigured = false;
static volatile bool s_gsCanHostLe = true;
static volatile bool s_gsCanInBusy = false;
static volatile bool s_gsCanOutPrimed = false;
static volatile bool s_gsCanEpReady = false;
static volatile uint16_t s_gsCanTxHead = 0U;
static volatile uint16_t s_gsCanTxTail = 0U;
static volatile uint16_t s_gsCanTxCount = 0U;
static volatile bool s_gsCanOverflowPending[GS_USB_CHANNEL_COUNT] = {false, false};
static gs_host_frame_classic_t s_gsCanTxQueue[GS_USB_FRAME_QUEUE_LENGTH];
static gs_can_deferred_request_t s_gsCanDeferredReq[GS_USB_CHANNEL_COUNT];

static gs_host_config_t s_gsHostConfig;
static gs_device_bittiming_t s_gsBitTiming;
static gs_device_mode_t s_gsModeRequest;
static uint32_t s_gsBerrRequest;
static gs_device_config_t s_gsDeviceConfig;
static gs_device_bt_const_t s_gsBtConst;
static gs_device_bittiming_t s_gsBitTimingByChannel[GS_USB_CHANNEL_COUNT];
static bool s_gsBitTimingValid[GS_USB_CHANNEL_COUNT] = {false, false};

static usb_device_class_config_struct_t s_cdcAcmConfig[] = {{
    USB_DeviceCdcVcomCallback,
    0U,
    &g_UsbDeviceCdcVcomConfig,
}};

static usb_device_class_config_list_struct_t s_cdcAcmConfigList = {
    s_cdcAcmConfig,
    USB_DeviceCallback,
    1U,
};

/*******************************************************************************
 * USB IRQ handlers
 ******************************************************************************/
void USB_OTG1_IRQHandler(void)
{
    USB_DeviceEhciIsrFunction(s_cdcState.deviceHandle);
}

void USB_OTG2_IRQHandler(void)
{
    USB_DeviceEhciIsrFunction(s_cdcState.deviceHandle);
}

void USDHC1_IRQHandler(void)
{
    USDHC1_DriverIRQHandler();
}

/*******************************************************************************
 * Helpers
 ******************************************************************************/
static void ShellWriteBytes(const uint8_t *data, uint32_t length)
{
    BaseType_t locked = pdFALSE;
    uint32_t waitTicks;

    if ((data == NULL) || (length == 0U))
    {
        return;
    }

    if ((s_cdcState.attach == 0U) || (s_cdcState.cdcAcmHandle == NULL))
    {
        return;
    }

    if (s_shellTxMutex != NULL)
    {
        locked = xSemaphoreTake(s_shellTxMutex, pdMS_TO_TICKS(50U));
        if (locked != pdTRUE)
        {
            return;
        }
    }

    while (length > 0U)
    {
        usb_status_t status;
        uint32_t chunk = (length > DATA_BUFF_SIZE) ? DATA_BUFF_SIZE : length;

        waitTicks = 0U;
        while (s_txBusy != 0U)
        {
            vTaskDelay(pdMS_TO_TICKS(1U));
            waitTicks++;
            if ((waitTicks >= 50U) || (s_cdcState.attach == 0U))
            {
                if ((s_cdcState.deviceHandle != NULL) && (s_cdcState.attach != 0U))
                {
                    (void)USB_DeviceCancel(s_cdcState.deviceHandle, (uint8_t)(USB_CDC_VCOM_BULK_IN_ENDPOINT | (USB_IN << 7U)));
                }
                s_txBusy = 0U;
                break;
            }
        }

        (void)memcpy(s_sendBuffer, data, chunk);
        s_txBusy = 1U;
        status   = USB_DeviceCdcAcmSend(s_cdcState.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, s_sendBuffer, chunk);
        if (status != kStatus_USB_Success)
        {
            s_txBusy = 0U;
            break;
        }

        data += chunk;
        length -= chunk;
    }

    if (locked == pdTRUE)
    {
        (void)xSemaphoreGive(s_shellTxMutex);
    }
}

static void ShellWrite(const char *text)
{
    if (text == NULL)
    {
        return;
    }
    ShellWriteBytes((const uint8_t *)text, (uint32_t)strlen(text));
}

static bool ShellWriteBytesTry(const uint8_t *data, uint32_t length)
{
    BaseType_t locked = pdFALSE;
    usb_status_t status;
    uint32_t chunk;

    if ((data == NULL) || (length == 0U))
    {
        return false;
    }
    if ((s_cdcState.attach == 0U) || (s_cdcState.cdcAcmHandle == NULL))
    {
        return false;
    }
    if (s_shellTxMutex != NULL)
    {
        locked = xSemaphoreTake(s_shellTxMutex, 0U);
        if (locked != pdTRUE)
        {
            return false;
        }
    }
    if (s_txBusy != 0U)
    {
        if (locked == pdTRUE)
        {
            (void)xSemaphoreGive(s_shellTxMutex);
        }
        return false;
    }

    chunk = (length > DATA_BUFF_SIZE) ? DATA_BUFF_SIZE : length;
    (void)memcpy(s_sendBuffer, data, chunk);
    s_txBusy = 1U;
    status = USB_DeviceCdcAcmSend(s_cdcState.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, s_sendBuffer, chunk);
    if (status != kStatus_USB_Success)
    {
        s_txBusy = 0U;
    }

    if (locked == pdTRUE)
    {
        (void)xSemaphoreGive(s_shellTxMutex);
    }
    return (status == kStatus_USB_Success);
}

static bool ShellWriteTry(const char *text)
{
    if (text == NULL)
    {
        return false;
    }
    return ShellWriteBytesTry((const uint8_t *)text, (uint32_t)strlen(text));
}

static void ShellSendPrompt(void)
{
    ShellWrite(SHELL_PROMPT);
}

static void ShellWriteLinef(const char *fmt, ...)
{
    char line[SHELL_TX_LINE_SIZE];
    va_list args;
    int32_t len;

    va_start(args, fmt);
    len = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    if (len <= 0)
    {
        return;
    }

    line[sizeof(line) - 1U] = '\0';
    ShellWrite(line);
}

static bool ShellTokenEquals(const char *token, BaseType_t tokenLen, const char *value)
{
    size_t valueLen;

    if ((token == NULL) || (value == NULL) || (tokenLen < 0))
    {
        return false;
    }

    valueLen = strlen(value);
    return ((size_t)tokenLen == valueLen) && (strncmp(token, value, valueLen) == 0);
}

static void ShellCliInit(void)
{
    if (s_shellCliReady)
    {
        return;
    }

    if (FreeRTOS_CLIRegisterCommand(&s_cliHelpCommand) != pdTRUE)
    {
        return;
    }
    if (FreeRTOS_CLIRegisterCommand(&s_cliLedCommand) != pdTRUE)
    {
        return;
    }
    if (FreeRTOS_CLIRegisterCommand(&s_cliCanCommand) != pdTRUE)
    {
        return;
    }
    if (FreeRTOS_CLIRegisterCommand(&s_cliSdCommand) != pdTRUE)
    {
        return;
    }
    if (FreeRTOS_CLIRegisterCommand(&s_cliRtcCommand) != pdTRUE)
    {
        return;
    }

    s_shellCliReady = true;
}

static BaseType_t ShellCliHelpCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                   "Production CLI:\r\n"
                   "  help                 - mostra questo help\r\n"
                   "  led <status|activity|on|off>\r\n"
                   "  can <status|util|clear>\r\n"
                   "  sd status            - spazio SD totale/libero/usato\r\n"
                   "  rtc status           - stato RTC SNVS (UTC)\r\n"
                   "  rtc set <epoch>      - imposta RTC con Unix epoch\r\n"
                   "Nota: CAN init/send solo via SocketCAN.\r\n");
    return pdFALSE;
}

static BaseType_t ShellCliLedCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    const char *subcmd;
    BaseType_t subcmdLen = 0;
    const char *modeText;

    subcmd = FreeRTOS_CLIGetParameter(pcCommandString, 1U, &subcmdLen);
    if ((subcmd == NULL) || ShellTokenEquals(subcmd, subcmdLen, "status"))
    {
        if (s_ledMode == kLedModeOn)
        {
            modeText = "on";
        }
        else if (s_ledMode == kLedModeOff)
        {
            modeText = "off";
        }
        else
        {
            modeText = "activity";
        }

        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "LED mode=%s pulse=%lu ms\r\n", modeText,
                       (unsigned long)LED_ACTIVITY_PULSE_MS);
        return pdFALSE;
    }

    if (ShellTokenEquals(subcmd, subcmdLen, "activity") || ShellTokenEquals(subcmd, subcmdLen, "blink"))
    {
        s_ledMode = kLedModeBlink;
        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "LED activity mode ON (RX/TX)\r\n");
        return pdFALSE;
    }

    if (ShellTokenEquals(subcmd, subcmdLen, "on"))
    {
        s_ledMode = kLedModeOn;
        USER_LED_ON();
        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "LED acceso fisso\r\n");
        return pdFALSE;
    }

    if (ShellTokenEquals(subcmd, subcmdLen, "off"))
    {
        s_ledMode = kLedModeOff;
        USER_LED_OFF();
        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "LED spento fisso\r\n");
        return pdFALSE;
    }

    (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: led <status|activity|on|off>\r\n");
    return pdFALSE;
}

static BaseType_t ShellCliCanCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    const char *subcmd;
    BaseType_t subcmdLen = 0;

    subcmd = FreeRTOS_CLIGetParameter(pcCommandString, 1U, &subcmdLen);
    if (subcmd == NULL)
    {
        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: can <status|util|clear>\r\n");
        return pdFALSE;
    }

    if (ShellTokenEquals(subcmd, subcmdLen, "status"))
    {
        (void)snprintf(
            pcWriteBuffer, xWriteBufferLen,
            "SocketCAN mode: can0=CAN1 can1=CAN2 ready=%u bitrate=%lu\r\n"
            "can0 up=%u load=%u.%u%% tx=%u.%u%% rx=%u.%u%% tx_ok=%lu tx_err=%lu rx_ok=%lu ovf=%lu\r\n"
            "can1 up=%u load=%u.%u%% tx=%u.%u%% rx=%u.%u%% tx_ok=%lu tx_err=%lu rx_ok=%lu ovf=%lu\r\n",
            s_canReady ? 1U : 0U, (unsigned long)s_canBitRate,
            s_canBuses[0].started ? 1U : 0U, (unsigned int)(s_canBuses[0].utilTotalPermille / 10U),
            (unsigned int)(s_canBuses[0].utilTotalPermille % 10U), (unsigned int)(s_canBuses[0].utilTxPermille / 10U),
            (unsigned int)(s_canBuses[0].utilTxPermille % 10U), (unsigned int)(s_canBuses[0].utilRxPermille / 10U),
            (unsigned int)(s_canBuses[0].utilRxPermille % 10U), (unsigned long)s_canBuses[0].txOkCount,
            (unsigned long)s_canBuses[0].txErrCount, (unsigned long)s_canBuses[0].rxOkCount,
            (unsigned long)s_canBuses[0].rxOverflowCount, s_canBuses[1].started ? 1U : 0U,
            (unsigned int)(s_canBuses[1].utilTotalPermille / 10U), (unsigned int)(s_canBuses[1].utilTotalPermille % 10U),
            (unsigned int)(s_canBuses[1].utilTxPermille / 10U), (unsigned int)(s_canBuses[1].utilTxPermille % 10U),
            (unsigned int)(s_canBuses[1].utilRxPermille / 10U), (unsigned int)(s_canBuses[1].utilRxPermille % 10U),
            (unsigned long)s_canBuses[1].txOkCount, (unsigned long)s_canBuses[1].txErrCount,
            (unsigned long)s_canBuses[1].rxOkCount, (unsigned long)s_canBuses[1].rxOverflowCount);
        return pdFALSE;
    }

    if (ShellTokenEquals(subcmd, subcmdLen, "util"))
    {
        (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                       "can0 load=%u.%u%% tx=%u.%u%% rx=%u.%u%% window=%u ms\r\n"
                       "can1 load=%u.%u%% tx=%u.%u%% rx=%u.%u%% window=%u ms\r\n",
                       (unsigned int)(s_canBuses[0].utilTotalPermille / 10U),
                       (unsigned int)(s_canBuses[0].utilTotalPermille % 10U),
                       (unsigned int)(s_canBuses[0].utilTxPermille / 10U),
                       (unsigned int)(s_canBuses[0].utilTxPermille % 10U),
                       (unsigned int)(s_canBuses[0].utilRxPermille / 10U),
                       (unsigned int)(s_canBuses[0].utilRxPermille % 10U),
                       (unsigned int)s_canBuses[0].utilWindowMs,
                       (unsigned int)(s_canBuses[1].utilTotalPermille / 10U),
                       (unsigned int)(s_canBuses[1].utilTotalPermille % 10U),
                       (unsigned int)(s_canBuses[1].utilTxPermille / 10U),
                       (unsigned int)(s_canBuses[1].utilTxPermille % 10U),
                       (unsigned int)(s_canBuses[1].utilRxPermille / 10U),
                       (unsigned int)(s_canBuses[1].utilRxPermille % 10U),
                       (unsigned int)s_canBuses[1].utilWindowMs);
        return pdFALSE;
    }

    if (ShellTokenEquals(subcmd, subcmdLen, "clear"))
    {
        CanResetRuntimeStats();
        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Statistiche CAN azzerate\r\n");
        return pdFALSE;
    }

    (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: can <status|util|clear>\r\n");
    return pdFALSE;
}

static BaseType_t ShellCliSdCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    const char *subcmd;
    BaseType_t subcmdLen = 0;
    FATFS *fs = NULL;
    DWORD freeClusters = 0U;
    FRESULT fr;
    uint64_t totalBytes;
    uint64_t freeBytes;
    uint64_t usedBytes;
    uint32_t usedPermille = 0U;

    subcmd = FreeRTOS_CLIGetParameter(pcCommandString, 1U, &subcmdLen);
    if ((subcmd != NULL) && !ShellTokenEquals(subcmd, subcmdLen, "status"))
    {
        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: sd status\r\n");
        return pdFALSE;
    }

    if (!s_fsMounted)
    {
        fr = FsMount();
        if (fr != FR_OK)
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "SD non montata: %s (%d)\r\n", FsResultToString(fr), (int)fr);
            return pdFALSE;
        }
    }

    fr = f_getfree(s_fsDrive, &freeClusters, &fs);
    if ((fr != FR_OK) || (fs == NULL))
    {
        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "sd status fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
        return pdFALSE;
    }

    totalBytes = ((uint64_t)(fs->n_fatent - 2U) * (uint64_t)fs->csize * (uint64_t)FF_MAX_SS);
    freeBytes = ((uint64_t)freeClusters * (uint64_t)fs->csize * (uint64_t)FF_MAX_SS);
    if (freeBytes > totalBytes)
    {
        freeBytes = totalBytes;
    }
    usedBytes = totalBytes - freeBytes;
    if (totalBytes > 0ULL)
    {
        usedPermille = (uint32_t)((usedBytes * 1000ULL) / totalBytes);
    }

    (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                   "SD totale=%llu MiB libero=%llu MiB usato=%llu MiB (%u.%u%%)\r\n",
                   (unsigned long long)(totalBytes / (1024ULL * 1024ULL)),
                   (unsigned long long)(freeBytes / (1024ULL * 1024ULL)),
                   (unsigned long long)(usedBytes / (1024ULL * 1024ULL)),
                   (unsigned int)(usedPermille / 10U), (unsigned int)(usedPermille % 10U));
    return pdFALSE;
}

static BaseType_t ShellCliRtcCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    const char *subcmd;
    BaseType_t subcmdLen = 0;
    uint32_t unixSeconds;
    rtc_datetime_t dt;

    subcmd = FreeRTOS_CLIGetParameter(pcCommandString, 1U, &subcmdLen);
    if ((subcmd == NULL) || ShellTokenEquals(subcmd, subcmdLen, "status"))
    {
        unixSeconds = RtcGetUnixSeconds();
        (void)RtcUnixSecondsToDateTime(unixSeconds, &dt);
        (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                       "RTC init=%u valid=%u epoch=%lu UTC=%04u-%02u-%02u %02u:%02u:%02u\r\n",
                       s_rtcInitialized ? 1U : 0U, RtcIsValid() ? 1U : 0U, (unsigned long)unixSeconds,
                       (unsigned int)dt.year, (unsigned int)dt.month, (unsigned int)dt.day, (unsigned int)dt.hour,
                       (unsigned int)dt.minute, (unsigned int)dt.second);
        return pdFALSE;
    }

    if (ShellTokenEquals(subcmd, subcmdLen, "set"))
    {
        const char *param = NULL;
        BaseType_t paramLen = 0;
        char token[24];

        param = FreeRTOS_CLIGetParameter(pcCommandString, 2U, &paramLen);
        if ((param == NULL) || (paramLen <= 0) || ((size_t)paramLen >= sizeof(token)))
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: rtc set <unix_epoch>\r\n");
            return pdFALSE;
        }

        (void)memcpy(token, param, (size_t)paramLen);
        token[paramLen] = '\0';
        if (!ShellParseU32Auto(token, &unixSeconds))
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Epoch non valido\r\n");
            return pdFALSE;
        }

        if (!RtcSetUnixSeconds(unixSeconds))
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Errore impostazione RTC\r\n");
            return pdFALSE;
        }

        (void)RtcUnixSecondsToDateTime(unixSeconds, &dt);
        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "RTC impostato: %lu (%04u-%02u-%02u %02u:%02u:%02u UTC)\r\n",
                       (unsigned long)unixSeconds, (unsigned int)dt.year, (unsigned int)dt.month, (unsigned int)dt.day,
                       (unsigned int)dt.hour, (unsigned int)dt.minute, (unsigned int)dt.second);
        return pdFALSE;
    }

    (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: rtc <status|set <unix_epoch>>\r\n");
    return pdFALSE;
}

static void LedMarkCanActivity(void)
{
    s_ledActivityLastTick = xTaskGetTickCount();
    s_ledActivitySeen = true;
}

static uint32_t GsBswap32(uint32_t value)
{
    return ((value & 0x000000FFUL) << 24U) | ((value & 0x0000FF00UL) << 8U) | ((value & 0x00FF0000UL) >> 8U) |
           ((value & 0xFF000000UL) >> 24U);
}

static bool GsIsLittleEndianHost(void)
{
    const uint16_t marker = 0x0001U;
    return (*(const uint8_t *)&marker) == 0x01U;
}

static uint32_t GsToLe32(uint32_t value)
{
    if (GsIsLittleEndianHost())
    {
        return value;
    }
    return GsBswap32(value);
}

static uint32_t GsFromLe32(uint32_t value)
{
    if (GsIsLittleEndianHost())
    {
        return value;
    }
    return GsBswap32(value);
}

static uint32_t GsWireToCpu32(uint32_t value)
{
    if (s_gsCanHostLe)
    {
        return GsFromLe32(value);
    }
    return value;
}

static uint32_t GsCpuToWire32(uint32_t value)
{
    if (s_gsCanHostLe)
    {
        return GsToLe32(value);
    }
    return value;
}

static bool GsCanQueueFrame(const gs_host_frame_classic_t *frame)
{
    bool queued = false;

    if (frame == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    if (s_gsCanTxCount >= GS_USB_FRAME_QUEUE_LENGTH)
    {
        taskEXIT_CRITICAL();
        return false;
    }

    s_gsCanTxQueue[s_gsCanTxHead] = *frame;
    s_gsCanTxHead                 = (uint16_t)((s_gsCanTxHead + 1U) % GS_USB_FRAME_QUEUE_LENGTH);
    s_gsCanTxCount++;
    queued = true;
    taskEXIT_CRITICAL();

    return queued;
}

static void GsCanTrySendNext(void)
{
    gs_host_frame_classic_t nextFrame;
    usb_status_t status;

    if (!s_gsCanConfigured || !s_gsCanEpReady || s_gsCanInBusy || (s_cdcState.attach == 0U) || (s_cdcState.deviceHandle == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();
    if (s_gsCanTxCount == 0U)
    {
        taskEXIT_CRITICAL();
        return;
    }

    nextFrame = s_gsCanTxQueue[s_gsCanTxTail];
    s_gsCanTxTail = (uint16_t)((s_gsCanTxTail + 1U) % GS_USB_FRAME_QUEUE_LENGTH);
    s_gsCanTxCount--;
    taskEXIT_CRITICAL();

    s_gsCanInFrame = nextFrame;
    s_gsCanInBusy  = true;
    status = USB_DeviceSendRequest(s_cdcState.deviceHandle, (uint8_t)(USB_GS_CAN_BULK_IN_ENDPOINT | (USB_IN << 7U)),
                                   (uint8_t *)&s_gsCanInFrame, sizeof(s_gsCanInFrame));
    if (status != kStatus_USB_Success)
    {
        s_gsCanInBusy = false;
        /* Keep frame for retry instead of dropping it when host side is temporarily busy. */
        taskENTER_CRITICAL();
        if (s_gsCanTxCount < GS_USB_FRAME_QUEUE_LENGTH)
        {
            s_gsCanTxTail = (uint16_t)((s_gsCanTxTail + GS_USB_FRAME_QUEUE_LENGTH - 1U) % GS_USB_FRAME_QUEUE_LENGTH);
            s_gsCanTxQueue[s_gsCanTxTail] = nextFrame;
            s_gsCanTxCount++;
        }
        taskEXIT_CRITICAL();
    }
}

static void GsCanSetOverflowFlag(size_t busIndex)
{
    if (busIndex < GS_USB_CHANNEL_COUNT)
    {
        s_gsCanOverflowPending[busIndex] = true;
    }
}

static bool GsCanRxPublish(size_t busIndex, const flexcan_frame_t *frame, status_t status)
{
    gs_host_frame_classic_t hostFrame;
    uint32_t canId;
    uint8_t data[8];
    uint8_t i;
    uint8_t dlc;
    bool queued;

    if ((frame == NULL) || (busIndex >= GS_USB_CHANNEL_COUNT) || !s_gsCanConfigured || !s_canBuses[busIndex].started)
    {
        return false;
    }

    if ((frame->format == (uint32_t)kFLEXCAN_FrameFormatExtend))
    {
        canId = (frame->id & CAN_EFF_MASK) | CAN_EFF_FLAG;
    }
    else
    {
        canId = ((frame->id >> CAN_ID_STD_SHIFT) & CAN_SFF_MASK);
    }

    dlc = frame->length;
    if (dlc > 8U)
    {
        dlc = 8U;
    }

    data[0] = frame->dataByte0;
    data[1] = frame->dataByte1;
    data[2] = frame->dataByte2;
    data[3] = frame->dataByte3;
    data[4] = frame->dataByte4;
    data[5] = frame->dataByte5;
    data[6] = frame->dataByte6;
    data[7] = frame->dataByte7;

    (void)memset(&hostFrame, 0, sizeof(hostFrame));
    hostFrame.echoId  = GsCpuToWire32(GS_HOST_FRAME_ECHO_ID_RX);
    hostFrame.canId   = GsCpuToWire32(canId);
    hostFrame.canDlc  = dlc;
    hostFrame.channel = (uint8_t)busIndex;
    hostFrame.flags   = 0U;

    if ((status == kStatus_FLEXCAN_RxOverflow) || s_gsCanOverflowPending[busIndex])
    {
        hostFrame.flags |= GS_CAN_FLAG_OVERFLOW;
        s_gsCanOverflowPending[busIndex] = false;
    }

    for (i = 0U; i < dlc; i++)
    {
        hostFrame.data[i] = data[i];
    }

    queued = GsCanQueueFrame(&hostFrame);
    if (!queued)
    {
        GsCanSetOverflowFlag(busIndex);
    }
    return queued;
}

static bool GsCanTxEchoPublish(size_t busIndex, uint32_t echoId, const flexcan_frame_t *frame)
{
    gs_host_frame_classic_t hostFrame;
    uint32_t canId;
    uint8_t dlc;
    uint8_t i;
    uint8_t data[8];

    if ((frame == NULL) || (busIndex >= GS_USB_CHANNEL_COUNT) || !s_gsCanConfigured)
    {
        return false;
    }

    if ((frame->format == (uint32_t)kFLEXCAN_FrameFormatExtend))
    {
        canId = (frame->id & CAN_EFF_MASK) | CAN_EFF_FLAG;
    }
    else
    {
        canId = ((frame->id >> CAN_ID_STD_SHIFT) & CAN_SFF_MASK);
    }
    if (frame->type == (uint32_t)kFLEXCAN_FrameTypeRemote)
    {
        canId |= CAN_RTR_FLAG;
    }

    dlc = frame->length;
    if (dlc > 8U)
    {
        dlc = 8U;
    }

    data[0] = frame->dataByte0;
    data[1] = frame->dataByte1;
    data[2] = frame->dataByte2;
    data[3] = frame->dataByte3;
    data[4] = frame->dataByte4;
    data[5] = frame->dataByte5;
    data[6] = frame->dataByte6;
    data[7] = frame->dataByte7;

    (void)memset(&hostFrame, 0, sizeof(hostFrame));
    hostFrame.echoId  = GsCpuToWire32(echoId);
    hostFrame.canId   = GsCpuToWire32(canId);
    hostFrame.canDlc  = dlc;
    hostFrame.channel = (uint8_t)busIndex;

    for (i = 0U; i < dlc; i++)
    {
        hostFrame.data[i] = data[i];
    }

    return GsCanQueueFrame(&hostFrame);
}

static bool GsCanFillBtConst(uint8_t channel, gs_device_bt_const_t *btConst)
{
    uint32_t canClock;

    if (btConst == NULL)
    {
        return false;
    }
    if (channel >= GS_USB_CHANNEL_COUNT)
    {
        return false;
    }

    canClock = CLOCK_GetClockRootFreq(kCLOCK_CanClkRoot);
    if (canClock == 0U)
    {
        /* If CAN root is not configured yet, set it once while all controllers are stopped. */
        CanApplyGsClockRoot();
        canClock = CLOCK_GetClockRootFreq(kCLOCK_CanClkRoot);
        if (canClock == 0U)
        {
            canClock = 20000000U;
        }
    }

    btConst->feature  = GsCpuToWire32(GS_CAN_FEATURE_LISTEN_ONLY | GS_CAN_FEATURE_LOOP_BACK);
    btConst->fclkCan  = GsCpuToWire32(canClock);
    btConst->tseg1Min = GsCpuToWire32(2U);
    btConst->tseg1Max = GsCpuToWire32(16U);
    btConst->tseg2Min = GsCpuToWire32(2U);
    btConst->tseg2Max = GsCpuToWire32(8U);
    btConst->sjwMax   = GsCpuToWire32(4U);
    btConst->brpMin   = GsCpuToWire32(1U);
    btConst->brpMax   = GsCpuToWire32(256U);
    btConst->brpInc   = GsCpuToWire32(1U);
    return true;
}

static uint8_t GsCanDecodeChannelFromSetup(const usb_setup_struct_t *setup)
{
    uint8_t channelLo;
    uint8_t channelHi;
    uint8_t indexHi;
    uint8_t indexLo;

    if (setup == NULL)
    {
        return 0xFFU;
    }

    channelLo = (uint8_t)(setup->wValue & 0xFFU);
    channelHi = (uint8_t)((setup->wValue >> 8U) & 0xFFU);
    indexHi = (uint8_t)((setup->wIndex >> 8U) & 0xFFU);
    indexLo = (uint8_t)(setup->wIndex & 0xFFU);

    /* Most stacks use wValue low byte. */
    if ((channelLo < GS_USB_CHANNEL_COUNT) &&
        !((channelLo == 0U) && (channelHi > 0U) && (channelHi < GS_USB_CHANNEL_COUNT)))
    {
        return channelLo;
    }

    /* Some stacks place channel in wValue high byte (for channel 1: wValue=0x0100). */
    if (channelHi < GS_USB_CHANNEL_COUNT)
    {
        return channelHi;
    }

    /* Last resort: channel in wIndex high byte (low byte remains interface number). */
    if (indexHi < GS_USB_CHANNEL_COUNT)
    {
        return indexHi;
    }

    /* Compatibility fallback: channel passed directly in wIndex low byte. */
    if (indexLo < GS_USB_CHANNEL_COUNT)
    {
        return indexLo;
    }

    return 0xFFU;
}

static bool RtcIsLeapYear(uint16_t year)
{
    if ((year % 4U) != 0U)
    {
        return false;
    }
    if ((year % 100U) != 0U)
    {
        return true;
    }
    return (year % 400U) == 0U;
}

static uint64_t RtcReadRawCounter(void)
{
    uint32_t msbA;
    uint32_t lsb;
    uint32_t msbB;
    uint32_t retry = RTC_COUNTER_SYNC_RETRY;

    do
    {
        msbA = SNVS->LPSRTCMR & SNVS_LPSRTCMR_SRTC_MASK;
        lsb  = SNVS->LPSRTCLR;
        msbB = SNVS->LPSRTCMR & SNVS_LPSRTCMR_SRTC_MASK;
        retry--;
    } while ((msbA != msbB) && (retry > 0U));

    return (((uint64_t)msbA << 32U) | (uint64_t)lsb);
}

static bool RtcEnableCounter(bool enable)
{
    uint32_t timeout = RTC_ENABLE_TIMEOUT_LOOPS;

    if (enable)
    {
        SNVS->LPCR |= SNVS_LPCR_SRTC_ENV_MASK;
    }
    else
    {
        SNVS->LPCR &= ~SNVS_LPCR_SRTC_ENV_MASK;
    }

    while (timeout > 0U)
    {
        bool isEnabled = (SNVS->LPCR & SNVS_LPCR_SRTC_ENV_MASK) != 0U;
        if (isEnabled == enable)
        {
            return true;
        }
        timeout--;
    }
    return false;
}

static bool RtcInit(void)
{
    if (s_rtcInitialized)
    {
        return true;
    }

    /* Clear sticky LP status flags. */
    SNVS->LPSR |= SNVS->LPSR;

    if ((SNVS->LPCR & SNVS_LPCR_SRTC_ENV_MASK) == 0U)
    {
        if (!RtcEnableCounter(true))
        {
            return false;
        }
    }

    s_rtcInitialized = true;
    return true;
}

static bool RtcIsValid(void)
{
    if (!RtcInit())
    {
        return false;
    }

    if ((SNVS->LPCR & SNVS_LPCR_SRTC_ENV_MASK) == 0U)
    {
        return false;
    }

    return (SNVS->LPGPR[0] == RTC_VALID_GPR_MAGIC);
}

static uint32_t RtcGetUnixSeconds(void)
{
    uint64_t counter;

    if (!RtcInit())
    {
        return 0U;
    }

    counter = RtcReadRawCounter();
    return (uint32_t)(counter >> RTC_COUNTER_SHIFT);
}

static bool RtcSetUnixSeconds(uint32_t unixSeconds)
{
    if (!RtcInit())
    {
        return false;
    }

    if (!RtcEnableCounter(false))
    {
        return false;
    }

    SNVS->LPSRTCLR = (uint32_t)(unixSeconds << RTC_COUNTER_SHIFT);
    SNVS->LPSRTCMR = (uint32_t)(unixSeconds >> (32U - RTC_COUNTER_SHIFT));

    if (!RtcEnableCounter(true))
    {
        return false;
    }

    SNVS->LPGPR[0] = RTC_VALID_GPR_MAGIC;
    SNVS->LPGPR[1] = unixSeconds;
    return true;
}

static bool RtcUnixSecondsToDateTime(uint32_t unixSeconds, rtc_datetime_t *dateTime)
{
    static const uint8_t kMonthDays[12] = {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};
    uint32_t days;
    uint32_t secOfDay;
    uint16_t year = 1970U;
    uint8_t month = 1U;
    uint8_t i;

    if (dateTime == NULL)
    {
        return false;
    }

    days    = unixSeconds / RTC_SECONDS_PER_DAY;
    secOfDay = unixSeconds % RTC_SECONDS_PER_DAY;

    while (true)
    {
        uint32_t daysThisYear = RtcIsLeapYear(year) ? RTC_DAYS_PER_LEAP_YEAR : RTC_DAYS_PER_NORMAL_YEAR;
        if (days < daysThisYear)
        {
            break;
        }
        days -= daysThisYear;
        year++;
    }

    for (i = 0U; i < 12U; i++)
    {
        uint32_t daysThisMonth = kMonthDays[i];
        if ((i == 1U) && RtcIsLeapYear(year))
        {
            daysThisMonth++;
        }
        if (days < daysThisMonth)
        {
            month = (uint8_t)(i + 1U);
            break;
        }
        days -= daysThisMonth;
    }

    dateTime->year   = year;
    dateTime->month  = month;
    dateTime->day    = (uint8_t)(days + 1U);
    dateTime->hour   = (uint8_t)(secOfDay / RTC_SECONDS_PER_HOUR);
    secOfDay        %= RTC_SECONDS_PER_HOUR;
    dateTime->minute = (uint8_t)(secOfDay / RTC_SECONDS_PER_MINUTE);
    dateTime->second = (uint8_t)(secOfDay % RTC_SECONDS_PER_MINUTE);
    return true;
}

static void RtcCommand(int32_t argc, char *argv[])
{
    uint32_t unixSeconds;
    rtc_datetime_t dt;

    if (argc < 2)
    {
        ShellWrite("Uso: rtc <status|set>\r\n");
        return;
    }

    if (strcmp(argv[1], "status") == 0)
    {
        unixSeconds = RtcGetUnixSeconds();
        (void)RtcUnixSecondsToDateTime(unixSeconds, &dt);
        ShellWriteLinef("RTC init=%u valid=%u unix=%lu %04u-%02u-%02u %02u:%02u:%02u UTC\r\n",
                        s_rtcInitialized ? 1U : 0U, RtcIsValid() ? 1U : 0U, (unsigned long)unixSeconds,
                        (unsigned int)dt.year, (unsigned int)dt.month, (unsigned int)dt.day,
                        (unsigned int)dt.hour, (unsigned int)dt.minute, (unsigned int)dt.second);
        return;
    }

    if (strcmp(argv[1], "set") == 0)
    {
        if ((argc < 3) || !ShellParseU32Auto(argv[2], &unixSeconds))
        {
            ShellWrite("Uso: rtc set <unix_epoch>\r\n");
            return;
        }

        if (!RtcSetUnixSeconds(unixSeconds))
        {
            ShellWrite("RTC set fallito\r\n");
            return;
        }

        (void)RtcUnixSecondsToDateTime(unixSeconds, &dt);
        ShellWriteLinef("RTC impostato: %lu (%04u-%02u-%02u %02u:%02u:%02u UTC)\r\n",
                        (unsigned long)unixSeconds, (unsigned int)dt.year, (unsigned int)dt.month,
                        (unsigned int)dt.day, (unsigned int)dt.hour, (unsigned int)dt.minute,
                        (unsigned int)dt.second);
        return;
    }

    ShellWrite("Uso: rtc <status|set>\r\n");
}

DWORD get_fattime(void)
{
    rtc_datetime_t dt;
    uint16_t year;

    if (RtcIsValid())
    {
        (void)RtcUnixSecondsToDateTime(RtcGetUnixSeconds(), &dt);
    }
    else
    {
        dt.year   = RTC_DEFAULT_FAT_YEAR;
        dt.month  = 1U;
        dt.day    = 1U;
        dt.hour   = 0U;
        dt.minute = 0U;
        dt.second = 0U;
    }

    year = dt.year;
    if (year < 1980U)
    {
        year = 1980U;
    }
    else if (year > 2107U)
    {
        year = 2107U;
    }

    return ((DWORD)(year - 1980U) << 25U) | ((DWORD)dt.month << 21U) | ((DWORD)dt.day << 16U) |
           ((DWORD)dt.hour << 11U) | ((DWORD)dt.minute << 5U) | ((DWORD)(dt.second / 2U));
}

static const char *FsResultToString(FRESULT result)
{
    switch (result)
    {
        case FR_OK:
            return "FR_OK";
        case FR_DISK_ERR:
            return "FR_DISK_ERR";
        case FR_INT_ERR:
            return "FR_INT_ERR";
        case FR_NOT_READY:
            return "FR_NOT_READY";
        case FR_NO_FILE:
            return "FR_NO_FILE";
        case FR_NO_PATH:
            return "FR_NO_PATH";
        case FR_INVALID_NAME:
            return "FR_INVALID_NAME";
        case FR_DENIED:
            return "FR_DENIED";
        case FR_EXIST:
            return "FR_EXIST";
        case FR_INVALID_OBJECT:
            return "FR_INVALID_OBJECT";
        case FR_WRITE_PROTECTED:
            return "FR_WRITE_PROTECTED";
        case FR_INVALID_DRIVE:
            return "FR_INVALID_DRIVE";
        case FR_NOT_ENABLED:
            return "FR_NOT_ENABLED";
        case FR_NO_FILESYSTEM:
            return "FR_NO_FILESYSTEM";
        case FR_MKFS_ABORTED:
            return "FR_MKFS_ABORTED";
        case FR_TIMEOUT:
            return "FR_TIMEOUT";
        case FR_LOCKED:
            return "FR_LOCKED";
        case FR_NOT_ENOUGH_CORE:
            return "FR_NOT_ENOUGH_CORE";
        case FR_TOO_MANY_OPEN_FILES:
            return "FR_TOO_MANY_OPEN_FILES";
        case FR_INVALID_PARAMETER:
            return "FR_INVALID_PARAMETER";
        default:
            return "FR_UNKNOWN";
    }
}

static FRESULT FsMount(void)
{
    FRESULT fr;

    if (!s_fsConfigured)
    {
        BOARD_SD_Config(&g_sd, NULL, BOARD_SDMMC_SD_HOST_IRQ_PRIORITY, NULL);
        s_fsConfigured = true;
    }

    fr = f_mount(&s_fsObject, s_fsDrive, 1U);
    if (fr != FR_OK)
    {
        s_fsMounted = false;
        return fr;
    }

#if (FF_FS_RPATH >= 2U)
    fr = f_chdrive(s_fsDrive);
    if (fr != FR_OK)
    {
        s_fsMounted = false;
        return fr;
    }
#endif

    s_fsMounted = true;
    return FR_OK;
}

static FRESULT FsUnmount(void)
{
    if (s_canLogFileOpen)
    {
        if (s_mf4LogState.ready)
        {
            (void)Mf4Sync(&s_canLogFile, &s_mf4LogState);
        }
        else
        {
            (void)f_sync(&s_canLogFile);
        }
        (void)f_close(&s_canLogFile);
        s_canLogFileOpen = false;
    }
    s_mf4LogState.ready = false;

    FRESULT fr = f_mount(NULL, s_fsDrive, 0U);
    if (fr == FR_OK)
    {
        s_fsMounted = false;
    }
    return fr;
}

static bool FsEnsureMounted(void)
{
    if (!s_fsMounted)
    {
        ShellWrite("Filesystem non montato. Usa: fs mount\r\n");
        return false;
    }
    return true;
}

static bool FsParseU32(const char *text, uint32_t *value)
{
    char *end = NULL;
    unsigned long parsed;

    if ((text == NULL) || (value == NULL) || (text[0] == '\0'))
    {
        return false;
    }

    parsed = strtoul(text, &end, 10);
    if ((end == text) || ((end != NULL) && (*end != '\0')) || (parsed > UINT32_MAX))
    {
        return false;
    }

    *value = (uint32_t)parsed;
    return true;
}

static bool ShellParseU32Auto(const char *text, uint32_t *value)
{
    char *end = NULL;
    unsigned long parsed;

    if ((text == NULL) || (value == NULL) || (text[0] == '\0'))
    {
        return false;
    }

    parsed = strtoul(text, &end, 0);
    if ((end == text) || ((end != NULL) && (*end != '\0')) || (parsed > UINT32_MAX))
    {
        return false;
    }

    *value = (uint32_t)parsed;
    return true;
}

static bool ShellParseCanBus(const char *text, uint32_t *busIndex)
{
    uint32_t rawBus;

    if ((text == NULL) || (busIndex == NULL))
    {
        return false;
    }

    if ((strcmp(text, "can0") == 0) || (strcmp(text, "CAN0") == 0))
    {
        *busIndex = 1U;
        return true;
    }
    if ((strcmp(text, "can1") == 0) || (strcmp(text, "CAN1") == 0))
    {
        *busIndex = 2U;
        return true;
    }

    if (!ShellParseU32Auto(text, &rawBus))
    {
        return false;
    }
    if ((rawBus == 0U) || (rawBus > ARRAY_SIZE(s_canBuses)))
    {
        return false;
    }

    *busIndex = rawBus;
    return true;
}

static const char *ShellSkipToken(const char *cursor)
{
    if (cursor == NULL)
    {
        return NULL;
    }

    while ((*cursor != '\0') && (isspace((unsigned char)*cursor) != 0))
    {
        cursor++;
    }

    while ((*cursor != '\0') && (isspace((unsigned char)*cursor) == 0))
    {
        cursor++;
    }

    while ((*cursor != '\0') && (isspace((unsigned char)*cursor) != 0))
    {
        cursor++;
    }

    return cursor;
}

static void FsCommandList(const char *path)
{
    FRESULT fr;
    DIR dir;
    FILINFO info;
    const char *targetPath = (path != NULL) ? path : "/";

    if (!FsEnsureMounted())
    {
        return;
    }

    fr = f_opendir(&dir, targetPath);
    if (fr != FR_OK)
    {
        ShellWriteLinef("ls fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
        return;
    }

    ShellWriteLinef("Listing %s\r\n", targetPath);
    while (true)
    {
        const char *name;

        fr = f_readdir(&dir, &info);
        if (fr != FR_OK)
        {
            ShellWriteLinef("readdir fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
            break;
        }

        if (info.fname[0] == '\0')
        {
            break;
        }

        name = info.fname;
#if FF_USE_LFN
        if (name[0] == '\0')
        {
            name = info.altname;
        }
#endif
        if (name[0] == '\0')
        {
            continue;
        }

        ShellWriteLinef("%c %10lu %s\r\n", ((info.fattrib & AM_DIR) != 0U) ? 'd' : 'f', (unsigned long)info.fsize, name);
    }

    (void)f_closedir(&dir);
}

static void FsCommandCat(const char *path)
{
    FRESULT fr;
    FIL file;
    UINT read = 0U;
    uint8_t buffer[128U];

    if ((path == NULL) || (path[0] == '\0'))
    {
        ShellWrite("Uso: fs cat <file>\r\n");
        return;
    }
    if (!FsEnsureMounted())
    {
        return;
    }

    fr = f_open(&file, path, FA_READ);
    if (fr != FR_OK)
    {
        ShellWriteLinef("open fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
        return;
    }

    while (true)
    {
        fr = f_read(&file, buffer, sizeof(buffer), &read);
        if (fr != FR_OK)
        {
            ShellWriteLinef("read fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
            break;
        }
        if (read == 0U)
        {
            break;
        }
        ShellWriteBytes(buffer, read);
    }

    (void)f_close(&file);
    ShellWrite("\r\n");
}

static void FsCommandWrite(const char *path, const char *payload, bool append)
{
    FRESULT fr;
    FIL file;
    UINT written = 0U;
    uint32_t length;
    BYTE mode = append ? (BYTE)(FA_OPEN_APPEND | FA_WRITE) : (BYTE)(FA_CREATE_ALWAYS | FA_WRITE);

    if ((path == NULL) || (path[0] == '\0'))
    {
        ShellWrite("Uso: fs write <file> <text> | fs append <file> <text>\r\n");
        return;
    }
    if (!FsEnsureMounted())
    {
        return;
    }

    if (payload == NULL)
    {
        payload = "";
    }
    length = (uint32_t)strlen(payload);

    fr = f_open(&file, path, mode);
    if (fr != FR_OK)
    {
        ShellWriteLinef("open fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
        return;
    }

    fr = f_write(&file, payload, length, &written);
    if ((fr == FR_OK) && (written != length))
    {
        fr = FR_DISK_ERR;
    }
    if (fr == FR_OK)
    {
        fr = f_sync(&file);
    }
    (void)f_close(&file);

    if (fr != FR_OK)
    {
        ShellWriteLinef("write fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
        return;
    }

    ShellWriteLinef("Scritti %lu byte su %s\r\n", (unsigned long)written, path);
}

static void FsCommandMkDir(const char *path)
{
    FRESULT fr;

    if ((path == NULL) || (path[0] == '\0'))
    {
        ShellWrite("Uso: fs mkdir <dir>\r\n");
        return;
    }
    if (!FsEnsureMounted())
    {
        return;
    }

    fr = f_mkdir(path);
    if (fr == FR_OK)
    {
        ShellWriteLinef("Directory creata: %s\r\n", path);
        return;
    }
    if (fr == FR_EXIST)
    {
        ShellWriteLinef("Directory gia' esistente: %s\r\n", path);
        return;
    }

    ShellWriteLinef("mkdir fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
}

static void FsCommandRemove(const char *path)
{
    FRESULT fr;

    if ((path == NULL) || (path[0] == '\0'))
    {
        ShellWrite("Uso: fs rm <file-or-dir>\r\n");
        return;
    }
    if (!FsEnsureMounted())
    {
        return;
    }

    fr = f_unlink(path);
    if (fr != FR_OK)
    {
        ShellWriteLinef("rm fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
        return;
    }

    ShellWriteLinef("Rimosso: %s\r\n", path);
}

static void FsCommandMkfs(void)
{
    FRESULT fr;
    BYTE work[FF_MAX_SS];

    (void)FsUnmount();

    fr = f_mkfs(s_fsDrive, 0U, work, sizeof(work));
    if (fr != FR_OK)
    {
        ShellWriteLinef("mkfs fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
        return;
    }

    fr = FsMount();
    if (fr != FR_OK)
    {
        ShellWriteLinef("mkfs ok, mount fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
        return;
    }

    ShellWrite("mkfs completato e filesystem montato\r\n");
}

SDK_ALIGN(static uint8_t s_fsBenchBuffer[FS_BENCH_BUFFER_SIZE], 32U);

static void FsCommandBench(uint32_t totalKiB, uint32_t runs)
{
    FRESULT fr;
    FIL file;
    uint64_t totalBytes;
    uint32_t run;
    uint32_t i;

    if (!FsEnsureMounted())
    {
        return;
    }
    if ((totalKiB == 0U) || (runs == 0U))
    {
        ShellWrite("Parametri non validi. Uso: fs bench [kib] [runs]\r\n");
        return;
    }

    for (i = 0U; i < sizeof(s_fsBenchBuffer); i++)
    {
        s_fsBenchBuffer[i] = (uint8_t)i;
    }

    totalBytes = (uint64_t)totalKiB * 1024ULL;
    ShellWriteLinef("Benchmark: %lu KiB x %lu run\r\n", (unsigned long)totalKiB, (unsigned long)runs);

    for (run = 0U; run < runs; run++)
    {
        TickType_t writeStart;
        TickType_t writeEnd;
        TickType_t readStart;
        TickType_t readEnd;
        uint64_t remaining;
        uint64_t checksum = 0ULL;
        uint32_t writeMs;
        uint32_t readMs;
        uint32_t writeKiBs;
        uint32_t readKiBs;

        fr = f_open(&file, FS_BENCH_FILE_PATH, FA_CREATE_ALWAYS | FA_WRITE);
        if (fr != FR_OK)
        {
            ShellWriteLinef("bench open write fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
            return;
        }

        writeStart = xTaskGetTickCount();
        remaining  = totalBytes;
        while (remaining > 0ULL)
        {
            UINT written = 0U;
            UINT chunk = (remaining > (uint64_t)sizeof(s_fsBenchBuffer)) ? (UINT)sizeof(s_fsBenchBuffer) : (UINT)remaining;

            fr = f_write(&file, s_fsBenchBuffer, chunk, &written);
            if ((fr != FR_OK) || (written != chunk))
            {
                if (fr == FR_OK)
                {
                    fr = FR_DISK_ERR;
                }
                break;
            }
            remaining -= (uint64_t)written;
        }
        if (fr == FR_OK)
        {
            fr = f_sync(&file);
        }
        writeEnd = xTaskGetTickCount();
        (void)f_close(&file);

        if (fr != FR_OK)
        {
            ShellWriteLinef("bench write fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
            return;
        }

        fr = f_open(&file, FS_BENCH_FILE_PATH, FA_READ);
        if (fr != FR_OK)
        {
            ShellWriteLinef("bench open read fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
            return;
        }

        readStart = xTaskGetTickCount();
        remaining = totalBytes;
        while (remaining > 0ULL)
        {
            UINT read = 0U;
            UINT chunk = (remaining > (uint64_t)sizeof(s_fsBenchBuffer)) ? (UINT)sizeof(s_fsBenchBuffer) : (UINT)remaining;

            fr = f_read(&file, s_fsBenchBuffer, chunk, &read);
            if ((fr != FR_OK) || (read != chunk))
            {
                if (fr == FR_OK)
                {
                    fr = FR_INT_ERR;
                }
                break;
            }

            for (i = 0U; i < read; i++)
            {
                checksum += s_fsBenchBuffer[i];
            }

            remaining -= (uint64_t)read;
        }
        readEnd = xTaskGetTickCount();
        (void)f_close(&file);

        if (fr != FR_OK)
        {
            ShellWriteLinef("bench read fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
            return;
        }

        writeMs = (uint32_t)((writeEnd - writeStart) * portTICK_PERIOD_MS);
        readMs  = (uint32_t)((readEnd - readStart) * portTICK_PERIOD_MS);
        if (writeMs == 0U)
        {
            writeMs = 1U;
        }
        if (readMs == 0U)
        {
            readMs = 1U;
        }

        writeKiBs = (uint32_t)(((totalBytes * 1000ULL) / 1024ULL) / writeMs);
        readKiBs  = (uint32_t)(((totalBytes * 1000ULL) / 1024ULL) / readMs);

        ShellWriteLinef("run %lu: write=%lu KiB/s (%lu ms), read=%lu KiB/s (%lu ms), checksum=%lu\r\n",
                        (unsigned long)(run + 1U), (unsigned long)writeKiBs, (unsigned long)writeMs,
                        (unsigned long)readKiBs, (unsigned long)readMs, (unsigned long)checksum);
    }
}

static void FsCommand(int32_t argc, char *argv[], const char *rawLine)
{
    if (argc < 2)
    {
        ShellWrite("Uso: fs <mount|umount|ls|cat|write|append|rm|mkdir|mkfs|bench>\r\n");
        return;
    }

    if (strcmp(argv[1], "mount") == 0)
    {
        FRESULT fr = FsMount();
        if (fr == FR_OK)
        {
            ShellWrite("Filesystem montato\r\n");
        }
        else
        {
            ShellWriteLinef("mount fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
        }
        return;
    }

    if (strcmp(argv[1], "umount") == 0)
    {
        FRESULT fr = FsUnmount();
        if (fr == FR_OK)
        {
            ShellWrite("Filesystem smontato\r\n");
        }
        else
        {
            ShellWriteLinef("umount fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
        }
        return;
    }

    if (strcmp(argv[1], "ls") == 0)
    {
        FsCommandList((argc >= 3) ? argv[2] : "/");
        return;
    }

    if (strcmp(argv[1], "cat") == 0)
    {
        FsCommandCat((argc >= 3) ? argv[2] : NULL);
        return;
    }

    if ((strcmp(argv[1], "write") == 0) || (strcmp(argv[1], "append") == 0))
    {
        const char *payload;
        const char *cursor = rawLine;
        bool append = (strcmp(argv[1], "append") == 0);

        if (argc < 3)
        {
            ShellWrite("Uso: fs write <file> <text> | fs append <file> <text>\r\n");
            return;
        }

        cursor  = ShellSkipToken(cursor);
        cursor  = ShellSkipToken(cursor);
        payload = ShellSkipToken(cursor);
        FsCommandWrite(argv[2], payload, append);
        return;
    }

    if (strcmp(argv[1], "rm") == 0)
    {
        FsCommandRemove((argc >= 3) ? argv[2] : NULL);
        return;
    }

    if (strcmp(argv[1], "mkdir") == 0)
    {
        FsCommandMkDir((argc >= 3) ? argv[2] : NULL);
        return;
    }

    if (strcmp(argv[1], "mkfs") == 0)
    {
        FsCommandMkfs();
        return;
    }

    if (strcmp(argv[1], "bench") == 0)
    {
        uint32_t kib = FS_BENCH_DEFAULT_KIB;
        uint32_t runs = FS_BENCH_DEFAULT_RUNS;

        if ((argc >= 3) && !FsParseU32(argv[2], &kib))
        {
            ShellWrite("Uso: fs bench [kib] [runs]\r\n");
            return;
        }
        if ((argc >= 4) && !FsParseU32(argv[3], &runs))
        {
            ShellWrite("Uso: fs bench [kib] [runs]\r\n");
            return;
        }
        FsCommandBench(kib, runs);
        return;
    }

    ShellWrite("Comando fs sconosciuto\r\n");
}

static status_t CanReleaseStopMode(CAN_Type *base)
{
    uint32_t stopReqMask = 0U;
    uint32_t stopAckMask = 0U;
    uint32_t timeout = 100000U;

    if (base == CAN1)
    {
        stopReqMask = IOMUXC_GPR_GPR4_CAN1_STOP_REQ_MASK;
        stopAckMask = IOMUXC_GPR_GPR4_CAN1_STOP_ACK_MASK;
    }
    else if (base == CAN2)
    {
        stopReqMask = IOMUXC_GPR_GPR4_CAN2_STOP_REQ_MASK;
        stopAckMask = IOMUXC_GPR_GPR4_CAN2_STOP_ACK_MASK;
    }

    if (stopReqMask == 0U)
    {
        return kStatus_Success;
    }

    CLOCK_EnableClock(kCLOCK_Iomuxc);
    CLOCK_EnableClock(kCLOCK_IomuxcGpr);
    IOMUXC_GPR->GPR4 &= ~stopReqMask;

    while (((IOMUXC_GPR->GPR4 & stopAckMask) != 0U) && (timeout > 0U))
    {
        timeout--;
    }

    if ((IOMUXC_GPR->GPR4 & stopAckMask) != 0U)
    {
        return kStatus_Timeout;
    }

    return kStatus_Success;
}

static status_t CanInitOneWithFlags(can_bus_context_t *bus, uint32_t bitRate, uint32_t modeFlags)
{
    flexcan_config_t config;
    flexcan_rx_mb_config_t rxStdConfig;
    flexcan_rx_mb_config_t rxExtConfig;
    status_t status;
    uint32_t sourceClock = CLOCK_GetClockRootFreq(kCLOCK_CanClkRoot);

    if (sourceClock == 0U)
    {
        return kStatus_Fail;
    }

    status = CanReleaseStopMode(bus->base);
    if (status != kStatus_Success)
    {
        return status;
    }

    FLEXCAN_GetDefaultConfig(&config);
    config.bitRate          = bitRate;
    config.maxMbNum           = 16U;
    config.enableLoopBack     = ((modeFlags & GS_CAN_MODE_LOOP_BACK) != 0U);
    config.enableIndividMask  = false;
    config.disableSelfReception = true;
    config.enableListenOnlyMode = ((modeFlags & GS_CAN_MODE_LISTEN_ONLY) != 0U);

    FLEXCAN_Init(bus->base, &config, sourceClock);

    if (((bus->base->MCR & CAN_MCR_MDIS_MASK) != 0U) || ((bus->base->MCR & CAN_MCR_LPMACK_MASK) != 0U))
    {
        return kStatus_Fail;
    }

    status = FLEXCAN_EnterFreezeMode(bus->base);
    if (status != kStatus_Success)
    {
        return status;
    }

    status = FLEXCAN_ExitFreezeMode(bus->base);
    if (status != kStatus_Success)
    {
        return status;
    }

    FLEXCAN_SetTxMbConfig(bus->base, bus->heartbeatMb, true);
    FLEXCAN_SetTxMbConfig(bus->base, bus->shellTxMb, true);
    FLEXCAN_SetRxMbGlobalMask(bus->base, 0U);

    rxStdConfig.id     = FLEXCAN_ID_STD(0U);
    rxStdConfig.format = kFLEXCAN_FrameFormatStandard;
    rxStdConfig.type   = kFLEXCAN_FrameTypeData;
    FLEXCAN_SetRxMbConfig(bus->base, bus->rxStdMb, &rxStdConfig, true);

    rxExtConfig.id     = FLEXCAN_ID_EXT(0U);
    rxExtConfig.format = kFLEXCAN_FrameFormatExtend;
    rxExtConfig.type   = kFLEXCAN_FrameTypeData;
    FLEXCAN_SetRxMbConfig(bus->base, bus->rxExtMb, &rxExtConfig, true);

    FLEXCAN_ClearMbStatusFlags(bus->base, ((uint64_t)1U << bus->rxStdMb) | ((uint64_t)1U << bus->rxExtMb));

    bus->txOkCount  = 0U;
    bus->txErrCount = 0U;
    bus->rxOkCount = 0U;
    bus->rxOverflowCount = 0U;
    bus->txBitCount = 0U;
    bus->rxBitCount = 0U;
    bus->utilTxPermille = 0U;
    bus->utilRxPermille = 0U;
    bus->utilTotalPermille = 0U;
    bus->utilWindowMs = 0U;
    bus->bitRate = bitRate;
    bus->modeFlags = modeFlags;
    bus->started = true;

    return kStatus_Success;
}

static status_t CanDeinitOne(can_bus_context_t *bus)
{
    if (bus == NULL)
    {
        return kStatus_InvalidArgument;
    }

    /* Mark stopped first so other tasks stop touching MB/registers during deinit. */
    bus->started = false;
    bus->bitRate = 0U;
    bus->modeFlags = 0U;
    bus->utilTxPermille = 0U;
    bus->utilRxPermille = 0U;
    bus->utilTotalPermille = 0U;
    bus->utilWindowMs = 0U;
    FLEXCAN_Deinit(bus->base);
    return kStatus_Success;
}

static uint32_t CanEstimateFrameBits(const flexcan_frame_t *frame)
{
    bool isExtended;
    bool isRemote;
    uint32_t dlc;
    uint32_t rawBits;

    if (frame == NULL)
    {
        return 0U;
    }

    isExtended = (frame->format == (uint32_t)kFLEXCAN_FrameFormatExtend);
    isRemote = (frame->type == (uint32_t)kFLEXCAN_FrameTypeRemote);
    dlc = frame->length;
    if (dlc > 8U)
    {
        dlc = 8U;
    }

    rawBits = isExtended ? 67U : 47U;
    if (!isRemote)
    {
        rawBits += (dlc * 8U);
    }

    /* Fast approximation that includes average stuffing overhead. */
    return rawBits + ((rawBits + 4U) / 5U);
}

static void CanUpdateUtilization(TickType_t now)
{
    TickType_t deltaTicks;
    uint32_t dtMs;
    size_t i;

    if (!s_canUtilInitialized)
    {
        s_canUtilInitialized = true;
        s_canUtilLastTick = now;
        for (i = 0U; i < ARRAY_SIZE(s_canBuses); i++)
        {
            s_canUtilLastTxBits[i] = s_canBuses[i].txBitCount;
            s_canUtilLastRxBits[i] = s_canBuses[i].rxBitCount;
            s_canBuses[i].utilTxPermille = 0U;
            s_canBuses[i].utilRxPermille = 0U;
            s_canBuses[i].utilTotalPermille = 0U;
            s_canBuses[i].utilWindowMs = 0U;
        }
        return;
    }

    deltaTicks = now - s_canUtilLastTick;
    dtMs = (uint32_t)(deltaTicks * portTICK_PERIOD_MS);
    if (dtMs < CAN_UTIL_UPDATE_MS)
    {
        return;
    }

    s_canUtilLastTick = now;

    for (i = 0U; i < ARRAY_SIZE(s_canBuses); i++)
    {
        uint32_t txNow = s_canBuses[i].txBitCount;
        uint32_t rxNow = s_canBuses[i].rxBitCount;
        uint32_t dTx = txNow - s_canUtilLastTxBits[i];
        uint32_t dRx = rxNow - s_canUtilLastRxBits[i];
        uint64_t capacityBits = ((uint64_t)s_canBitRate * (uint64_t)dtMs) / 1000ULL;
        uint32_t txPermille = 0U;
        uint32_t rxPermille = 0U;
        uint32_t totalPermille = 0U;

        s_canUtilLastTxBits[i] = txNow;
        s_canUtilLastRxBits[i] = rxNow;

        if (s_canBuses[i].started && (capacityBits > 0ULL))
        {
            txPermille = (uint32_t)(((uint64_t)dTx * 1000ULL) / capacityBits);
            rxPermille = (uint32_t)(((uint64_t)dRx * 1000ULL) / capacityBits);
            totalPermille = txPermille + rxPermille;
            if (txPermille > 1000U)
            {
                txPermille = 1000U;
            }
            if (rxPermille > 1000U)
            {
                rxPermille = 1000U;
            }
            if (totalPermille > 1000U)
            {
                totalPermille = 1000U;
            }
        }

        s_canBuses[i].utilTxPermille = (uint16_t)txPermille;
        s_canBuses[i].utilRxPermille = (uint16_t)rxPermille;
        s_canBuses[i].utilTotalPermille = (uint16_t)totalPermille;
        s_canBuses[i].utilWindowMs = (uint16_t)((dtMs > 0xFFFFU) ? 0xFFFFU : dtMs);
    }
}

static void CanResetRuntimeStats(void)
{
    size_t i;

    for (i = 0U; i < ARRAY_SIZE(s_canBuses); i++)
    {
        s_canBuses[i].txOkCount = 0U;
        s_canBuses[i].txErrCount = 0U;
        s_canBuses[i].rxOkCount = 0U;
        s_canBuses[i].rxOverflowCount = 0U;
        s_canBuses[i].txBitCount = 0U;
        s_canBuses[i].rxBitCount = 0U;
        s_canBuses[i].utilTxPermille = 0U;
        s_canBuses[i].utilRxPermille = 0U;
        s_canBuses[i].utilTotalPermille = 0U;
        s_canBuses[i].utilWindowMs = 0U;
        s_canUtilLastTxBits[i] = 0U;
        s_canUtilLastRxBits[i] = 0U;
    }

    s_canUtilInitialized = false;
    s_canUtilLastTick = xTaskGetTickCount();
}

static status_t CanWriteTxMbWithRecovery(can_bus_context_t *bus, uint8_t mbIdx, const flexcan_frame_t *frame)
{
    status_t status;

    if ((bus == NULL) || (frame == NULL))
    {
        return kStatus_InvalidArgument;
    }

    status = FLEXCAN_WriteTxMb(bus->base, mbIdx, frame);
    if (status == kStatus_FLEXCAN_TxBusy)
    {
        /* Recover from a stale pending TX (for example no ACK/bus errors): abort MB and retry once. */
        FLEXCAN_SetTxMbConfig(bus->base, mbIdx, true);
        status = FLEXCAN_WriteTxMb(bus->base, mbIdx, frame);
    }

    return status;
}

static status_t CanSendFrame(can_bus_context_t *bus, uint8_t mbIdx, uint32_t id, const uint8_t *payload, uint8_t length)
{
    flexcan_frame_t frame;
    status_t status;

    if ((bus == NULL) || (length > 8U))
    {
        return kStatus_InvalidArgument;
    }

    (void)memset(&frame, 0, sizeof(frame));
    frame.type   = (uint32_t)kFLEXCAN_FrameTypeData;
    frame.length = length;
    if (id <= 0x7FFU)
    {
        frame.format = (uint32_t)kFLEXCAN_FrameFormatStandard;
        frame.id     = FLEXCAN_ID_STD(id);
    }
    else
    {
        frame.format = (uint32_t)kFLEXCAN_FrameFormatExtend;
        frame.id     = FLEXCAN_ID_EXT(id & 0x1FFFFFFFU);
    }

    frame.dataByte0 = (length > 0U) ? payload[0] : 0U;
    frame.dataByte1 = (length > 1U) ? payload[1] : 0U;
    frame.dataByte2 = (length > 2U) ? payload[2] : 0U;
    frame.dataByte3 = (length > 3U) ? payload[3] : 0U;
    frame.dataByte4 = (length > 4U) ? payload[4] : 0U;
    frame.dataByte5 = (length > 5U) ? payload[5] : 0U;
    frame.dataByte6 = (length > 6U) ? payload[6] : 0U;
    frame.dataByte7 = (length > 7U) ? payload[7] : 0U;

    LedMarkCanActivity();
    status = CanWriteTxMbWithRecovery(bus, mbIdx, &frame);
    if (status == kStatus_Success)
    {
        bus->txOkCount++;
        bus->txBitCount += CanEstimateFrameBits(&frame);
    }
    else
    {
        bus->txErrCount++;
    }
    return status;
}

static status_t CanSendFrameRaw(can_bus_context_t *bus, uint8_t mbIdx, const flexcan_frame_t *frame)
{
    status_t status;

    if ((bus == NULL) || (frame == NULL))
    {
        return kStatus_InvalidArgument;
    }

    LedMarkCanActivity();
    status = CanWriteTxMbWithRecovery(bus, mbIdx, frame);
    if (status == kStatus_Success)
    {
        bus->txOkCount++;
        bus->txBitCount += CanEstimateFrameBits(frame);
    }
    else
    {
        bus->txErrCount++;
    }
    return status;
}

static void CanDumpPrintFrame(size_t busIndex, const flexcan_frame_t *frame, status_t status)
{
    char line[SHELL_TX_LINE_SIZE];
    uint8_t length;
    bool isExtended;
    int32_t pos;

    if (frame == NULL)
    {
        return;
    }
    length = frame->length;
    if (length > 8U)
    {
        length = 8U;
    }
    isExtended = (frame->format == (uint32_t)kFLEXCAN_FrameFormatExtend);

    if (isExtended)
    {
        pos = snprintf(line, sizeof(line), "can%lu  %08lX   [%u]", (unsigned long)busIndex,
                       (unsigned long)(frame->id & 0x1FFFFFFFU), (unsigned int)length);
    }
    else
    {
        pos = snprintf(line, sizeof(line), "can%lu  %03lX   [%u]", (unsigned long)busIndex,
                       (unsigned long)((frame->id & CAN_ID_STD_MASK) >> CAN_ID_STD_SHIFT), (unsigned int)length);
    }

    if (pos < 0)
    {
        return;
    }

    if (frame->type == (uint32_t)kFLEXCAN_FrameTypeRemote)
    {
        pos += snprintf(line + (size_t)pos, sizeof(line) - (size_t)pos, " RTR");
    }
    else
    {
        const uint8_t bytes[8] = {frame->dataByte0, frame->dataByte1, frame->dataByte2, frame->dataByte3,
                                  frame->dataByte4, frame->dataByte5, frame->dataByte6, frame->dataByte7};
        uint8_t i;
        for (i = 0U; i < length; i++)
        {
            if ((size_t)pos >= sizeof(line))
            {
                break;
            }
            pos += snprintf(line + (size_t)pos, sizeof(line) - (size_t)pos, " %02X", (unsigned int)bytes[i]);
        }
    }

    if (status == kStatus_FLEXCAN_RxOverflow)
    {
        if ((size_t)pos < sizeof(line))
        {
            pos += snprintf(line + (size_t)pos, sizeof(line) - (size_t)pos, " OVR");
        }
    }

    if ((size_t)pos < sizeof(line))
    {
        (void)snprintf(line + (size_t)pos, sizeof(line) - (size_t)pos, "\r\n");
    }
    line[sizeof(line) - 1U] = '\0';
    (void)ShellWriteTry(line);
}

static uint32_t Mf4Align8(uint32_t value)
{
    return (value + 7U) & ~7U;
}

static void Mf4PutU16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void Mf4PutU32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16U) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static void Mf4PutU64(uint8_t *dst, uint64_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16U) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24U) & 0xFFU);
    dst[4] = (uint8_t)((value >> 32U) & 0xFFU);
    dst[5] = (uint8_t)((value >> 40U) & 0xFFU);
    dst[6] = (uint8_t)((value >> 48U) & 0xFFU);
    dst[7] = (uint8_t)((value >> 56U) & 0xFFU);
}

static void Mf4PutS16(uint8_t *dst, int16_t value)
{
    Mf4PutU16(dst, (uint16_t)value);
}

static void Mf4PutDouble(uint8_t *dst, double value)
{
    union
    {
        double d;
        uint64_t u;
    } v;
    v.d = value;
    Mf4PutU64(dst, v.u);
}

static bool Mf4WriteAt(FIL *file, FSIZE_t offset, const void *data, UINT size)
{
    UINT written = 0U;
    FRESULT fr;

    if ((file == NULL) || (data == NULL) || (size == 0U))
    {
        return false;
    }

    fr = f_lseek(file, offset);
    if (fr != FR_OK)
    {
        return false;
    }

    fr = f_write(file, data, size, &written);
    return (fr == FR_OK) && (written == size);
}

static bool Mf4ReadAt(FIL *file, FSIZE_t offset, void *data, UINT size)
{
    UINT readBytes = 0U;
    FRESULT fr;

    if ((file == NULL) || (data == NULL) || (size == 0U))
    {
        return false;
    }

    fr = f_lseek(file, offset);
    if (fr != FR_OK)
    {
        return false;
    }

    fr = f_read(file, data, size, &readBytes);
    return (fr == FR_OK) && (readBytes == size);
}

static uint64_t Mf4UnixSecondsToNano(uint32_t unixSeconds)
{
    return ((uint64_t)unixSeconds) * 1000000000ULL;
}

static void Mf4ComputeLayout(uint32_t *txOffsets, uint32_t *dtBlockOffset, uint32_t *dtDataOffset)
{
    uint32_t cursor = MF4_OFFSET_CN0 + (MF4_CHANNEL_COUNT * MF4_CN_BLOCK_SIZE);
    size_t i;

    for (i = 0U; i < MF4_CHANNEL_COUNT; i++)
    {
        uint32_t txLen = (uint32_t)strlen(s_mf4Channels[i].name) + 1U;
        uint32_t blockLen = Mf4Align8(MF4_BLOCK_COMMON_SIZE + txLen);
        if (txOffsets != NULL)
        {
            txOffsets[i] = cursor;
        }
        cursor += blockLen;
    }

    cursor = Mf4Align8(cursor);
    *dtBlockOffset = cursor;
    *dtDataOffset = cursor + MF4_BLOCK_COMMON_SIZE;
}

static bool Mf4WriteTextBlock(FIL *file, uint32_t offset, const char *text)
{
    uint8_t block[96];
    uint32_t textLen;
    uint32_t blockLen;

    if ((file == NULL) || (text == NULL))
    {
        return false;
    }

    textLen = (uint32_t)strlen(text) + 1U;
    blockLen = Mf4Align8(MF4_BLOCK_COMMON_SIZE + textLen);
    if (blockLen > sizeof(block))
    {
        return false;
    }

    (void)memset(block, 0, sizeof(block));
    (void)memcpy(block, MF4_TX_BLOCK_ID, 4U);
    Mf4PutU64(&block[8], blockLen);
    Mf4PutU64(&block[16], 0U);
    (void)memcpy(&block[MF4_BLOCK_COMMON_SIZE], text, textLen);

    return Mf4WriteAt(file, offset, block, blockLen);
}

static bool Mf4WriteFreshFile(FIL *file, mf4_log_state_t *state)
{
    uint8_t block[MF4_CN_BLOCK_SIZE];
    uint8_t idBlock[MF4_ID_BLOCK_SIZE];
    uint32_t txOffsets[MF4_CHANNEL_COUNT];
    uint32_t dtBlockOffset;
    uint32_t dtDataOffset;
    uint64_t startTimeNs = 0U;
    uint8_t timeFlags = 0U;
    size_t i;

    if ((file == NULL) || (state == NULL))
    {
        return false;
    }

    Mf4ComputeLayout(txOffsets, &dtBlockOffset, &dtDataOffset);

    if (RtcIsValid())
    {
        startTimeNs = Mf4UnixSecondsToNano(RtcGetUnixSeconds());
        timeFlags = 2U;
    }

    if (f_lseek(file, 0U) != FR_OK)
    {
        return false;
    }
    if (f_truncate(file) != FR_OK)
    {
        return false;
    }

    (void)memset(idBlock, 0, sizeof(idBlock));
    (void)memcpy(&idBlock[0], MF4_ID_BLOCK_MAGIC, 8U);
    (void)memcpy(&idBlock[8], MF4_VERSION_TEXT, 8U);
    (void)memcpy(&idBlock[16], MF4_PROGRAM_TEXT, 8U);
    Mf4PutU16(&idBlock[28], MF4_VERSION_NUM);
    if (!Mf4WriteAt(file, MF4_OFFSET_ID, idBlock, sizeof(idBlock)))
    {
        return false;
    }

    (void)memset(block, 0, sizeof(block));
    (void)memcpy(&block[0], "##HD", 4U);
    Mf4PutU64(&block[8], MF4_HD_BLOCK_SIZE);
    Mf4PutU64(&block[16], 6U);
    Mf4PutU64(&block[24], MF4_OFFSET_DG);
    Mf4PutU64(&block[32], MF4_OFFSET_FH);
    Mf4PutU64(&block[40], 0U);
    Mf4PutU64(&block[48], 0U);
    Mf4PutU64(&block[56], 0U);
    Mf4PutU64(&block[64], 0U);
    Mf4PutU64(&block[72], startTimeNs);
    Mf4PutS16(&block[80], 0);
    Mf4PutS16(&block[82], 0);
    block[84] = timeFlags;
    block[85] = 0U;
    block[86] = 0U;
    block[87] = 0U;
    Mf4PutDouble(&block[88], 0.0);
    Mf4PutDouble(&block[96], 0.0);
    if (!Mf4WriteAt(file, MF4_OFFSET_HD, block, MF4_HD_BLOCK_SIZE))
    {
        return false;
    }

    (void)memset(block, 0, sizeof(block));
    (void)memcpy(&block[0], "##FH", 4U);
    Mf4PutU64(&block[8], MF4_FH_BLOCK_SIZE);
    Mf4PutU64(&block[16], 2U);
    Mf4PutU64(&block[24], 0U);
    Mf4PutU64(&block[32], 0U);
    Mf4PutU64(&block[40], startTimeNs);
    Mf4PutS16(&block[48], 0);
    Mf4PutS16(&block[50], 0);
    block[52] = timeFlags;
    if (!Mf4WriteAt(file, MF4_OFFSET_FH, block, MF4_FH_BLOCK_SIZE))
    {
        return false;
    }

    (void)memset(block, 0, sizeof(block));
    (void)memcpy(&block[0], "##DG", 4U);
    Mf4PutU64(&block[8], MF4_DG_BLOCK_SIZE);
    Mf4PutU64(&block[16], 4U);
    Mf4PutU64(&block[24], 0U);
    Mf4PutU64(&block[32], MF4_OFFSET_CG);
    Mf4PutU64(&block[40], dtBlockOffset);
    Mf4PutU64(&block[48], 0U);
    if (!Mf4WriteAt(file, MF4_OFFSET_DG, block, MF4_DG_BLOCK_SIZE))
    {
        return false;
    }

    (void)memset(block, 0, sizeof(block));
    (void)memcpy(&block[0], "##CG", 4U);
    Mf4PutU64(&block[8], MF4_CG_BLOCK_SIZE);
    Mf4PutU64(&block[16], 6U);
    Mf4PutU64(&block[24], 0U);
    Mf4PutU64(&block[32], MF4_OFFSET_CN0);
    Mf4PutU64(&block[40], 0U);
    Mf4PutU64(&block[48], 0U);
    Mf4PutU64(&block[56], 0U);
    Mf4PutU64(&block[64], 0U);
    Mf4PutU64(&block[72], 0U);
    Mf4PutU64(&block[80], 0U);
    Mf4PutU16(&block[88], 0U);
    Mf4PutU16(&block[90], (uint16_t)'/');
    Mf4PutU32(&block[92], 0U);
    Mf4PutU32(&block[96], sizeof(can_log_record_t));
    Mf4PutU32(&block[100], 0U);
    if (!Mf4WriteAt(file, MF4_OFFSET_CG, block, MF4_CG_BLOCK_SIZE))
    {
        return false;
    }

    for (i = 0U; i < MF4_CHANNEL_COUNT; i++)
    {
        uint32_t cnOffset = MF4_OFFSET_CN0 + ((uint32_t)i * MF4_CN_BLOCK_SIZE);
        uint32_t nextCn = (i + 1U < MF4_CHANNEL_COUNT) ? (cnOffset + MF4_CN_BLOCK_SIZE) : 0U;

        (void)memset(block, 0, sizeof(block));
        (void)memcpy(&block[0], "##CN", 4U);
        Mf4PutU64(&block[8], MF4_CN_BLOCK_SIZE);
        Mf4PutU64(&block[16], 8U);
        Mf4PutU64(&block[24], nextCn);
        Mf4PutU64(&block[32], 0U);
        Mf4PutU64(&block[40], txOffsets[i]);
        Mf4PutU64(&block[48], 0U);
        Mf4PutU64(&block[56], 0U);
        Mf4PutU64(&block[64], 0U);
        Mf4PutU64(&block[72], 0U);
        Mf4PutU64(&block[80], 0U);
        block[88] = s_mf4Channels[i].channelType;
        block[89] = s_mf4Channels[i].syncType;
        block[90] = s_mf4Channels[i].dataType;
        block[91] = 0U;
        Mf4PutU32(&block[92], s_mf4Channels[i].byteOffset);
        Mf4PutU32(&block[96], s_mf4Channels[i].bitCount);
        Mf4PutU32(&block[100], MF4_CN_FLAGS_DEFAULT);
        Mf4PutU32(&block[104], 0U);
        block[108] = 0U;
        block[109] = 0U;
        Mf4PutU16(&block[110], 0U);
        Mf4PutDouble(&block[112], 0.0);
        Mf4PutDouble(&block[120], 0.0);
        Mf4PutDouble(&block[128], 0.0);
        Mf4PutDouble(&block[136], 0.0);
        Mf4PutDouble(&block[144], 0.0);
        Mf4PutDouble(&block[152], 0.0);
        if (!Mf4WriteAt(file, cnOffset, block, MF4_CN_BLOCK_SIZE))
        {
            return false;
        }
    }

    for (i = 0U; i < MF4_CHANNEL_COUNT; i++)
    {
        if (!Mf4WriteTextBlock(file, txOffsets[i], s_mf4Channels[i].name))
        {
            return false;
        }
    }

    (void)memset(block, 0, sizeof(block));
    (void)memcpy(&block[0], "##DT", 4U);
    Mf4PutU64(&block[8], MF4_BLOCK_COMMON_SIZE);
    Mf4PutU64(&block[16], 0U);
    if (!Mf4WriteAt(file, dtBlockOffset, block, MF4_BLOCK_COMMON_SIZE))
    {
        return false;
    }

    state->ready = true;
    state->recordCount = 0U;
    state->dataBytes = 0U;
    state->cgBlockOffset = MF4_OFFSET_CG;
    state->dtBlockOffset = dtBlockOffset;
    state->dtDataOffset = dtDataOffset;

    if (!Mf4Sync(file, state))
    {
        return false;
    }

    return f_lseek(file, dtDataOffset) == FR_OK;
}

static bool Mf4PatchCounters(FIL *file, const mf4_log_state_t *state)
{
    uint8_t value[8];
    FSIZE_t savedPos;

    if ((file == NULL) || (state == NULL) || !state->ready)
    {
        return false;
    }

    savedPos = f_tell(file);

    Mf4PutU64(value, state->recordCount);
    if (!Mf4WriteAt(file, state->cgBlockOffset + MF4_CG_CYCLES_FIELD_OFFSET, value, sizeof(value)))
    {
        return false;
    }

    Mf4PutU64(value, (uint64_t)MF4_BLOCK_COMMON_SIZE + state->dataBytes);
    if (!Mf4WriteAt(file, state->dtBlockOffset + MF4_DT_LEN_FIELD_OFFSET, value, sizeof(value)))
    {
        return false;
    }

    return f_lseek(file, savedPos) == FR_OK;
}

static bool Mf4Sync(FIL *file, const mf4_log_state_t *state)
{
    if (!Mf4PatchCounters(file, state))
    {
        return false;
    }
    return f_sync(file) == FR_OK;
}

static bool Mf4RecoverFile(FIL *file, mf4_log_state_t *state)
{
    uint8_t magic[8];
    uint8_t program[8];
    uint32_t dtBlockOffset;
    uint32_t dtDataOffset;
    FSIZE_t fileSize;
    FSIZE_t dataBytes;
    FSIZE_t remainder;

    if ((file == NULL) || (state == NULL))
    {
        return false;
    }

    Mf4ComputeLayout(NULL, &dtBlockOffset, &dtDataOffset);
    fileSize = f_size(file);

    if (fileSize < dtDataOffset)
    {
        return Mf4WriteFreshFile(file, state);
    }

    if (!Mf4ReadAt(file, 0U, magic, sizeof(magic)) || !Mf4ReadAt(file, 16U, program, sizeof(program)))
    {
        return false;
    }
    if ((memcmp(magic, MF4_ID_BLOCK_MAGIC, sizeof(magic)) != 0) ||
        (memcmp(program, MF4_PROGRAM_TEXT, sizeof(program)) != 0))
    {
        return Mf4WriteFreshFile(file, state);
    }

    dataBytes = fileSize - dtDataOffset;
    remainder = dataBytes % sizeof(can_log_record_t);
    if (remainder != 0U)
    {
        FSIZE_t validSize = fileSize - remainder;
        if (f_lseek(file, validSize) != FR_OK)
        {
            return false;
        }
        if (f_truncate(file) != FR_OK)
        {
            return false;
        }
        fileSize = validSize;
        dataBytes = fileSize - dtDataOffset;
    }

    state->ready = true;
    state->recordCount = dataBytes / sizeof(can_log_record_t);
    state->dataBytes = dataBytes;
    state->cgBlockOffset = MF4_OFFSET_CG;
    state->dtBlockOffset = dtBlockOffset;
    state->dtDataOffset = dtDataOffset;

    if (!Mf4Sync(file, state))
    {
        return false;
    }

    return f_lseek(file, state->dtDataOffset + state->dataBytes) == FR_OK;
}

static bool CanLogEnqueue(size_t busIndex, const flexcan_frame_t *frame, status_t status)
{
    can_log_record_t record;
    BaseType_t sent;
    uint32_t canId;
    uint8_t dlc;

    if ((frame == NULL) || (s_canLogQueue == NULL))
    {
        return false;
    }

    if (frame->format == (uint32_t)kFLEXCAN_FrameFormatExtend)
    {
        canId = (frame->id & CAN_EFF_MASK) | CAN_EFF_FLAG;
    }
    else
    {
        canId = ((frame->id >> CAN_ID_STD_SHIFT) & CAN_SFF_MASK);
    }
    if (frame->type == (uint32_t)kFLEXCAN_FrameTypeRemote)
    {
        canId |= CAN_RTR_FLAG;
    }

    dlc = frame->length;
    if (dlc > 8U)
    {
        dlc = 8U;
    }

    (void)memset(&record, 0, sizeof(record));
    record.timestampNs = ((uint64_t)xTaskGetTickCount()) * ((uint64_t)portTICK_PERIOD_MS) * 1000000ULL;
    record.channel = (uint8_t)busIndex;
    record.dlc = dlc;
    record.frameFlags = 0U;
    if (status == kStatus_FLEXCAN_RxOverflow)
    {
        record.frameFlags |= GS_CAN_FLAG_OVERFLOW;
    }
    record.canId = canId;
    record.data64 = ((uint64_t)frame->dataByte0) | (((uint64_t)frame->dataByte1) << 8U) |
                    (((uint64_t)frame->dataByte2) << 16U) | (((uint64_t)frame->dataByte3) << 24U) |
                    (((uint64_t)frame->dataByte4) << 32U) | (((uint64_t)frame->dataByte5) << 40U) |
                    (((uint64_t)frame->dataByte6) << 48U) | (((uint64_t)frame->dataByte7) << 56U);

    sent = xQueueSendToBack(s_canLogQueue, &record, 0U);
    return sent == pdTRUE;
}

static void CanHandleRxFrame(size_t busIndex, const flexcan_frame_t *frame, status_t status)
{
    if ((frame == NULL) || (busIndex >= ARRAY_SIZE(s_canBuses)))
    {
        return;
    }

    s_canBuses[busIndex].rxOkCount++;
    s_canBuses[busIndex].rxBitCount += CanEstimateFrameBits(frame);
    if (status == kStatus_FLEXCAN_RxOverflow)
    {
        s_canBuses[busIndex].rxOverflowCount++;
    }

    if (s_canDumpEnabled)
    {
        CanDumpPrintFrame(busIndex, frame, status);
    }

    LedMarkCanActivity();

    if (s_gsCanConfigured && s_canBuses[busIndex].started && !GsCanRxPublish(busIndex, frame, status))
    {
        GsCanSetOverflowFlag(busIndex);
    }
    (void)CanLogEnqueue(busIndex, frame, status);
}

static void CanPollRxMb(can_bus_context_t *bus, size_t busIndex, uint8_t mbIdx)
{
    uint64_t mbMask;
    flexcan_frame_t frame;
    status_t status;

    if ((bus == NULL) || !bus->started)
    {
        return;
    }

    mbMask = (uint64_t)1U << mbIdx;
    if (FLEXCAN_GetMbStatusFlags(bus->base, mbMask) == 0U)
    {
        return;
    }

    FLEXCAN_ClearMbStatusFlags(bus->base, mbMask);
    status = FLEXCAN_ReadRxMb(bus->base, mbIdx, &frame);
    if ((status == kStatus_Success) || (status == kStatus_FLEXCAN_RxOverflow))
    {
        CanHandleRxFrame(busIndex, &frame, status);
    }
}

static void CanPollRxDump(void)
{
    size_t i;

    if (!s_canReady)
    {
        return;
    }

    for (i = 0U; i < ARRAY_SIZE(s_canBuses); i++)
    {
        CanPollRxMb(&s_canBuses[i], i, s_canBuses[i].rxStdMb);
        CanPollRxMb(&s_canBuses[i], i, s_canBuses[i].rxExtMb);
    }
}

static status_t CanInitAll(uint32_t bitRate, uint32_t modeFlags)
{
    size_t i;
    uint32_t canRootHz;

    s_canReady = false;
    s_canDumpEnabled = false;

    /* Match co_device_basic CAN clock setup:
     * root from USB1 PLL/8 with divider 2 => 20 MHz protocol engine clock.
     */
    CanApplyGsClockRoot();

    canRootHz = CLOCK_GetClockRootFreq(kCLOCK_CanClkRoot);
    ShellWriteLinef("CAN clock root=%lu Hz (mux=0 div=2)\r\n", (unsigned long)canRootHz);

    for (i = 0U; i < ARRAY_SIZE(s_canBuses); i++)
    {
        s_canBuses[i].started = false;
        s_canBuses[i].bitRate = 0U;
        s_canBuses[i].modeFlags = 0U;
        ShellWriteLinef("Init CAN%lu...\r\n", (unsigned long)(i + 1U));
        status_t status = CanInitOneWithFlags(&s_canBuses[i], bitRate, modeFlags);
        if (status != kStatus_Success)
        {
            ShellWriteLinef("CAN%lu init errore=%ld MCR=0x%08lx ESR1=0x%08lx\r\n", (unsigned long)(i + 1U), (long)status,
                            (unsigned long)s_canBuses[i].base->MCR, (unsigned long)s_canBuses[i].base->ESR1);
            return status;
        }
    }
    s_canBitRate = bitRate;
    s_canModeFlags = modeFlags;
    s_canReady   = true;
    return kStatus_Success;
}

static status_t CanSendFromShell(uint32_t busIndex, uint32_t id, const uint8_t *payload, uint8_t length)
{
    if ((busIndex == 0U) || (busIndex > ARRAY_SIZE(s_canBuses)) || !s_canReady)
    {
        return kStatus_InvalidArgument;
    }
    if (!s_canBuses[busIndex - 1U].started)
    {
        return kStatus_Fail;
    }

    return CanSendFrame(&s_canBuses[busIndex - 1U], s_canBuses[busIndex - 1U].shellTxMb, id, payload, length);
}

static void CanCommand(int32_t argc, char *argv[])
{
    if (argc < 2)
    {
        ShellWrite("Uso: can <status|init|hb|dump|send>\r\n");
        return;
    }

    if (strcmp(argv[1], "status") == 0)
    {
        size_t i;
        ShellWriteLinef("CAN ready=%u heartbeat=%u dump=%u bitrate=%lu loopback=%u listen=%u gs_cfg=%u gs_ep=%u gs_q=%u\r\n",
                        s_canReady ? 1U : 0U, s_canHeartbeatEnabled ? 1U : 0U, s_canDumpEnabled ? 1U : 0U,
                        (unsigned long)s_canBitRate,
                        ((s_canModeFlags & GS_CAN_MODE_LOOP_BACK) != 0U) ? 1U : 0U,
                        ((s_canModeFlags & GS_CAN_MODE_LISTEN_ONLY) != 0U) ? 1U : 0U, s_gsCanConfigured ? 1U : 0U,
                        s_gsCanEpReady ? 1U : 0U, (unsigned int)s_gsCanTxCount);
        ShellWrite("Mappa bus: CAN1=can0  CAN2=can1\r\n");
        for (i = 0U; i < ARRAY_SIZE(s_canBuses); i++)
        {
            ShellWriteLinef("CAN%lu on=%u tx_ok=%lu tx_err=%lu rx_ok=%lu rx_ovf=%lu mode=0x%lx ESR1=0x%08lx defer=%lu/%u\r\n",
                            (unsigned long)(i + 1U), s_canBuses[i].started ? 1U : 0U,
                            (unsigned long)s_canBuses[i].txOkCount, (unsigned long)s_canBuses[i].txErrCount,
                            (unsigned long)s_canBuses[i].rxOkCount, (unsigned long)s_canBuses[i].rxOverflowCount,
                            (unsigned long)s_canBuses[i].modeFlags,
                            (unsigned long)s_canBuses[i].base->ESR1,
                            (unsigned long)s_gsCanDeferredReq[i].op,
                            (unsigned int)s_gsCanDeferredReq[i].pending);
        }
        return;
    }

    if (strcmp(argv[1], "init") == 0)
    {
        uint32_t bitRate = s_canBitRate;
        uint32_t modeFlags = 0U;
        status_t status;

        if ((argc >= 3) && !ShellParseU32Auto(argv[2], &bitRate))
        {
            ShellWrite("Uso: can init [bitrate] [normal|loop|listen]\r\n");
            return;
        }
        if (argc >= 4)
        {
            if ((strcmp(argv[3], "loop") == 0) || (strcmp(argv[3], "loopback") == 0))
            {
                modeFlags |= GS_CAN_MODE_LOOP_BACK;
            }
            else if (strcmp(argv[3], "listen") == 0)
            {
                modeFlags |= GS_CAN_MODE_LISTEN_ONLY;
            }
            else if ((strcmp(argv[3], "normal") != 0) && (strcmp(argv[3], "none") != 0))
            {
                ShellWrite("Uso: can init [bitrate] [normal|loop|listen]\r\n");
                return;
            }
        }

        status = CanInitAll(bitRate, modeFlags);
        if (status == kStatus_Success)
        {
            ShellWriteLinef("CAN inizializzato a %lu bps (loopback=%u listen=%u)\r\n", (unsigned long)bitRate,
                            ((modeFlags & GS_CAN_MODE_LOOP_BACK) != 0U) ? 1U : 0U,
                            ((modeFlags & GS_CAN_MODE_LISTEN_ONLY) != 0U) ? 1U : 0U);
        }
        else
        {
            ShellWriteLinef("CAN init fallito: %ld\r\n", (long)status);
        }
        return;
    }

    if (strcmp(argv[1], "hb") == 0)
    {
        if ((argc < 3) || ((strcmp(argv[2], "on") != 0) && (strcmp(argv[2], "off") != 0)))
        {
            ShellWrite("Uso: can hb on|off\r\n");
            return;
        }

        s_canHeartbeatEnabled = (strcmp(argv[2], "on") == 0);
        if (s_canHeartbeatEnabled && !s_canReady)
        {
            s_canHeartbeatEnabled = false;
            ShellWrite("CAN non inizializzato. Esegui prima: can init [bitrate]\r\n");
            return;
        }
        ShellWriteLinef("Heartbeat CAN %s\r\n", s_canHeartbeatEnabled ? "ON" : "OFF");
        return;
    }

    if (strcmp(argv[1], "dump") == 0)
    {
        bool enable;

        if ((argc < 3) || ((strcmp(argv[2], "on") != 0) && (strcmp(argv[2], "off") != 0)))
        {
            ShellWrite("Uso: can dump on|off\r\n");
            return;
        }

        enable = (strcmp(argv[2], "on") == 0);
        if (enable && !s_canReady)
        {
            ShellWrite("CAN non inizializzato. Esegui prima: can init [bitrate]\r\n");
            return;
        }

        s_canDumpEnabled = enable;
        ShellWriteLinef("CAN dump %s\r\n", s_canDumpEnabled ? "ON" : "OFF");
        return;
    }

    if (strcmp(argv[1], "send") == 0)
    {
        uint32_t bus;
        uint32_t id;
        uint8_t payload[8] = {0};
        uint8_t length;
        int32_t i;
        status_t status;

        if ((argc < 4) || (argc > 12))
        {
            ShellWrite("Uso: can send <1|2|can0|can1> <id> [b0 ... b7]\r\n");
            return;
        }
        if (!ShellParseCanBus(argv[2], &bus) || !ShellParseU32Auto(argv[3], &id))
        {
            ShellWrite("Uso: can send <1|2|can0|can1> <id> [b0 ... b7]\r\n");
            return;
        }

        if ((s_canModeFlags & GS_CAN_MODE_LOOP_BACK) != 0U)
        {
            ShellWrite("ATTENZIONE: loopback attivo, il frame non esce sul bus fisico.\r\n");
        }
        if ((s_canModeFlags & GS_CAN_MODE_LISTEN_ONLY) != 0U)
        {
            ShellWrite("ATTENZIONE: listen-only attivo, TX/ACK su bus non garantiti.\r\n");
        }

        length = (uint8_t)(argc - 4);
        for (i = 0; i < (int32_t)length; i++)
        {
            uint32_t byteValue;
            if (!ShellParseU32Auto(argv[4 + i], &byteValue) || (byteValue > 0xFFU))
            {
                ShellWrite("Byte CAN non valido\r\n");
                return;
            }
            payload[i] = (uint8_t)byteValue;
        }

        status = CanSendFromShell(bus, id, payload, length);
        if (status == kStatus_Success)
        {
            ShellWriteLinef("CAN%lu TX ok id=0x%lx len=%u\r\n", (unsigned long)bus, (unsigned long)id, length);
        }
        else
        {
            ShellWriteLinef("CAN TX fallita: %ld\r\n", (long)status);
        }
        return;
    }

    ShellWrite("Comando can sconosciuto\r\n");
}

static void ShellPrintHelp(void)
{
    ShellWrite("Comandi disponibili:\r\n");
    ShellWrite("  help            - mostra questo help\r\n");
    ShellWrite("  led blink       - impulso su traffico CAN RX/TX (default)\r\n");
    ShellWrite("  led on          - accende il LED\r\n");
    ShellWrite("  led off         - spegne il LED\r\n");
    ShellWrite("  led toggle      - alterna on/off (modo manuale)\r\n");
    ShellWrite("  fs mount        - monta la SD con FatFs\r\n");
    ShellWrite("  fs umount       - smonta la SD\r\n");
    ShellWrite("  fs ls [path]    - lista directory\r\n");
    ShellWrite("  fs cat <file>   - stampa file\r\n");
    ShellWrite("  fs write <f> <t>- sovrascrive file con testo\r\n");
    ShellWrite("  fs append <f> <t>- aggiunge testo a file\r\n");
    ShellWrite("  fs rm <path>    - elimina file/directory vuota\r\n");
    ShellWrite("  fs mkdir <dir>  - crea directory\r\n");
    ShellWrite("  fs mkfs         - formatta FAT e rimonta\r\n");
    ShellWrite("  fs bench [kib] [runs] - benchmark read/write\r\n");
    ShellWrite("  rtc status      - stato RTC SNVS (UTC)\r\n");
    ShellWrite("  rtc set <epoch> - imposta RTC con Unix epoch (UTC)\r\n");
    ShellWrite("  can status      - stato CAN1/CAN2\r\n");
    ShellWrite("  can init [bps] [normal|loop|listen] - inizializza CAN1/CAN2\r\n");
    ShellWrite("  can hb on|off   - heartbeat periodico sui due CAN\r\n");
    ShellWrite("  can dump on|off - stampa live dei frame ricevuti (stile candump)\r\n");
    ShellWrite("  can send <1|2|can0|can1> <id> [b0..b7] - invia frame\r\n");
    ShellWrite("  cansend <1|2|can0|can1> <id> [b0..b7]  - alias rapido di can send\r\n");
}

static void ShellHandleLine(char *line)
{
    char rawLine[SHELL_LINE_BUFFER_SIZE];
    char *argv[16];
    int32_t argc = 0;
    char *token;

    (void)strncpy(rawLine, line, sizeof(rawLine) - 1U);
    rawLine[sizeof(rawLine) - 1U] = '\0';
    token = strtok(line, " \t");

    while ((token != NULL) && (argc < (int32_t)(sizeof(argv) / sizeof(argv[0]))))
    {
        argv[argc++] = token;
        token = strtok(NULL, " \t");
    }

    if (argc == 0)
    {
        return;
    }

    if (s_shellProductionMode && s_shellCliReady)
    {
        char cliOutput[SHELL_CLI_OUTPUT_SIZE];
        BaseType_t moreData;
        const char *commandInput = rawLine;

        do
        {
            cliOutput[0] = '\0';
            moreData = FreeRTOS_CLIProcessCommand(commandInput, cliOutput, sizeof(cliOutput));
            if (cliOutput[0] != '\0')
            {
                ShellWrite(cliOutput);
            }
            commandInput = "";
        } while (moreData != pdFALSE);
        return;
    }

    if (strcmp(argv[0], "help") == 0)
    {
        ShellPrintHelp();
        return;
    }

    if (strcmp(argv[0], "led") == 0)
    {
        if (argc < 2)
        {
            ShellWrite("Uso: led blink|on|off|toggle\r\n");
            return;
        }

        if (strcmp(argv[1], "blink") == 0)
        {
            s_ledMode = kLedModeBlink;
            ShellWrite("LED in activity mode (impulso su traffico CAN RX/TX)\r\n");
            return;
        }

        if (strcmp(argv[1], "on") == 0)
        {
            s_ledMode = kLedModeOn;
            USER_LED_ON();
            ShellWrite("LED acceso\r\n");
            return;
        }

        if (strcmp(argv[1], "off") == 0)
        {
            s_ledMode = kLedModeOff;
            USER_LED_OFF();
            ShellWrite("LED spento\r\n");
            return;
        }

        if (strcmp(argv[1], "toggle") == 0)
        {
            if (s_ledMode == kLedModeOn)
            {
                s_ledMode = kLedModeOff;
                USER_LED_OFF();
            }
            else
            {
                s_ledMode = kLedModeOn;
                USER_LED_ON();
            }
            ShellWrite("LED invertito\r\n");
            return;
        }

        ShellWrite("Uso: led blink|on|off|toggle\r\n");
        return;
    }

    if (strcmp(argv[0], "cansend") == 0)
    {
        char cmdCan[] = "can";
        char cmdSend[] = "send";
        char *canArgv[16];
        int32_t i;
        int32_t canArgc = argc + 1;

        if (canArgc > (int32_t)(sizeof(canArgv) / sizeof(canArgv[0])))
        {
            ShellWrite("Uso: cansend <1|2|can0|can1> <id> [b0 ... b7]\r\n");
            return;
        }

        canArgv[0] = cmdCan;
        canArgv[1] = cmdSend;
        for (i = 1; i < argc; i++)
        {
            canArgv[i + 1] = argv[i];
        }

        CanCommand(canArgc, canArgv);
        return;
    }

    if (strcmp(argv[0], "fs") == 0)
    {
        FsCommand(argc, argv, rawLine);
        return;
    }

    if (strcmp(argv[0], "can") == 0)
    {
        CanCommand(argc, argv);
        return;
    }

    if (strcmp(argv[0], "rtc") == 0)
    {
        RtcCommand(argc, argv);
        return;
    }

    ShellWrite("Comando sconosciuto\r\n");
}

static void ShellHandlePacket(const uint8_t *data, uint32_t length)
{
    static char lineBuffer[SHELL_LINE_BUFFER_SIZE];
    static size_t lineLength = 0U;
    static bool skipNextLf   = false;
    uint32_t i;

    for (i = 0U; i < length; i++)
    {
        uint8_t ch = data[i];

        if (skipNextLf && (ch == (uint8_t)'\n'))
        {
            skipNextLf = false;
            continue;
        }
        skipNextLf = false;

        if ((ch == (uint8_t)'\r') || (ch == (uint8_t)'\n'))
        {
            if (ch == (uint8_t)'\r')
            {
                skipNextLf = true;
            }

            ShellWrite("\r\n");
            lineBuffer[lineLength] = '\0';
            ShellHandleLine(lineBuffer);
            lineLength = 0U;
            ShellSendPrompt();
            continue;
        }

        if ((ch == (uint8_t)'\b') || (ch == 127U))
        {
            if (lineLength > 0U)
            {
                lineLength--;
                ShellWrite("\b \b");
            }
            continue;
        }

        if ((lineLength < (SHELL_LINE_BUFFER_SIZE - 1U)) && (isprint((int)ch) != 0))
        {
            lineBuffer[lineLength++] = (char)ch;
            ShellWriteBytes(&ch, 1U);
        }
    }
}

static void CanRefreshReadyState(void)
{
    size_t i;
    bool anyStarted = false;

    for (i = 0U; i < ARRAY_SIZE(s_canBuses); i++)
    {
        if (s_canBuses[i].started)
        {
            anyStarted = true;
            break;
        }
    }
    s_canReady = anyStarted;
}

static bool CanAnyControllerStarted(void)
{
    size_t i;

    for (i = 0U; i < ARRAY_SIZE(s_canBuses); i++)
    {
        if (s_canBuses[i].started)
        {
            return true;
        }
    }
    return false;
}

static void CanApplyGsClockRoot(void)
{
    /* CAN1/CAN2 share the same root; never touch it while any controller is running. */
    if (CanAnyControllerStarted())
    {
        return;
    }
    CLOCK_SetMux(kCLOCK_CanMux, 0U);
    CLOCK_SetDiv(kCLOCK_CanDiv, 2U);
}

static void GsCanProcessDeferredRequests(void)
{
    size_t channel;

    for (channel = 0U; channel < GS_USB_CHANNEL_COUNT; channel++)
    {
        uint32_t op;
        uint32_t bitRate;
        uint32_t flags;
        status_t status = kStatus_Success;

        if (s_gsCanDeferredReq[channel].pending == 0U)
        {
            continue;
        }

        op = s_gsCanDeferredReq[channel].op;
        bitRate = s_gsCanDeferredReq[channel].bitRate;
        flags = s_gsCanDeferredReq[channel].flags & GS_CAN_MODE_SUPPORTED_MASK;
        s_gsCanDeferredReq[channel].pending = 0U;
        s_gsCanDeferredReq[channel].op = GS_CAN_DEFERRED_OP_NONE;

        if (op == GS_CAN_DEFERRED_OP_START)
        {
            bool needsReinit = !s_canBuses[channel].started ||
                               (s_canBuses[channel].bitRate != bitRate) ||
                               (s_canBuses[channel].modeFlags != flags);

            if (needsReinit)
            {
                if (s_canBuses[channel].started)
                {
                    (void)CanDeinitOne(&s_canBuses[channel]);
                }

                CanApplyGsClockRoot();
                status = CanInitOneWithFlags(&s_canBuses[channel], bitRate, flags);
                if (status == kStatus_Success)
                {
                    s_canBitRate = bitRate;
                    s_canModeFlags = flags;
                }
            }
        }
        else if (op == GS_CAN_DEFERRED_OP_RESET)
        {
            if (s_canBuses[channel].started)
            {
                (void)CanDeinitOne(&s_canBuses[channel]);
            }
        }

        CanRefreshReadyState();
        if (!s_canReady)
        {
            s_canModeFlags = 0U;
        }
        (void)status;
    }
}

static void GsCanHandleBulkOutPacket(const uint8_t *data, uint32_t length)
{
    uint32_t offset = 0U;

    while ((data != NULL) && ((length - offset) >= sizeof(gs_host_frame_classic_t)))
    {
        const gs_host_frame_classic_t *hostFrame = (const gs_host_frame_classic_t *)(data + offset);
        uint8_t channel = hostFrame->channel;
        uint32_t canId = GsWireToCpu32(hostFrame->canId);
        uint32_t echoId = GsWireToCpu32(hostFrame->echoId);
        uint8_t dlc = hostFrame->canDlc;
        flexcan_frame_t txFrame;
        status_t status;

        offset += sizeof(gs_host_frame_classic_t);

        if ((channel >= ARRAY_SIZE(s_canBuses)) || !s_canBuses[channel].started)
        {
            continue;
        }

        if (dlc > 8U)
        {
            dlc = 8U;
        }

        (void)memset(&txFrame, 0, sizeof(txFrame));
        txFrame.length = dlc;
        if ((canId & CAN_EFF_FLAG) != 0U)
        {
            txFrame.format = (uint32_t)kFLEXCAN_FrameFormatExtend;
            txFrame.id     = FLEXCAN_ID_EXT(canId & CAN_EFF_MASK);
        }
        else
        {
            txFrame.format = (uint32_t)kFLEXCAN_FrameFormatStandard;
            txFrame.id     = FLEXCAN_ID_STD(canId & CAN_SFF_MASK);
        }

        if ((canId & CAN_RTR_FLAG) != 0U)
        {
            txFrame.type = (uint32_t)kFLEXCAN_FrameTypeRemote;
        }
        else
        {
            txFrame.type = (uint32_t)kFLEXCAN_FrameTypeData;
            txFrame.dataByte0 = (dlc > 0U) ? hostFrame->data[0] : 0U;
            txFrame.dataByte1 = (dlc > 1U) ? hostFrame->data[1] : 0U;
            txFrame.dataByte2 = (dlc > 2U) ? hostFrame->data[2] : 0U;
            txFrame.dataByte3 = (dlc > 3U) ? hostFrame->data[3] : 0U;
            txFrame.dataByte4 = (dlc > 4U) ? hostFrame->data[4] : 0U;
            txFrame.dataByte5 = (dlc > 5U) ? hostFrame->data[5] : 0U;
            txFrame.dataByte6 = (dlc > 6U) ? hostFrame->data[6] : 0U;
            txFrame.dataByte7 = (dlc > 7U) ? hostFrame->data[7] : 0U;
        }

        status = CanSendFrameRaw(&s_canBuses[channel], s_canBuses[channel].shellTxMb, &txFrame);
        (void)status;

        /* Always return an echo frame so the host gs_usb queue cannot stall waiting for completion. */
        if (!GsCanTxEchoPublish(channel, echoId, &txFrame))
        {
            GsCanSetOverflowFlag(channel);
        }
    }
}

static bool GsCanHandleVendorRequest(usb_device_control_request_struct_t *controlRequest)
{
    usb_setup_struct_t *setup;
    uint8_t channel;
    uint8_t indexLo;

    if ((controlRequest == NULL) || (controlRequest->setup == NULL))
    {
        return false;
    }

    setup = controlRequest->setup;
    if ((setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) != USB_REQUEST_TYPE_RECIPIENT_INTERFACE)
    {
        return false;
    }
    indexLo = (uint8_t)(setup->wIndex & 0xFFU);
    if (indexLo != USB_GS_CAN_INTERFACE_INDEX)
    {
        bool channelInIndex = ((setup->bRequest == GS_USB_BREQ_BT_CONST) || (setup->bRequest == GS_USB_BREQ_BITTIMING) ||
                               (setup->bRequest == GS_USB_BREQ_MODE) || (setup->bRequest == GS_USB_BREQ_BERR)) &&
                              (indexLo < GS_USB_CHANNEL_COUNT);
        if (!channelInIndex)
        {
            return false;
        }
    }

    switch (setup->bRequest)
    {
        case GS_USB_BREQ_HOST_FORMAT:
            if (controlRequest->isSetup != 0U)
            {
                if (((setup->bmRequestType & USB_REQUEST_TYPE_DIR_MASK) != USB_REQUEST_TYPE_DIR_OUT) ||
                    (setup->wLength != sizeof(s_gsHostConfig)))
                {
                    return false;
                }
                controlRequest->buffer = (uint8_t *)&s_gsHostConfig;
                controlRequest->length = sizeof(s_gsHostConfig);
                return true;
            }
            s_gsCanHostLe = (GsFromLe32(s_gsHostConfig.byteOrder) == GS_HOST_CONFIG_LITTLE_ENDIAN);
            return true;

        case GS_USB_BREQ_DEVICE_CONFIG:
            if (controlRequest->isSetup == 0U)
            {
                return false;
            }
            if (((setup->bmRequestType & USB_REQUEST_TYPE_DIR_MASK) != USB_REQUEST_TYPE_DIR_IN) ||
                (setup->wLength < sizeof(s_gsDeviceConfig)))
            {
                return false;
            }
            (void)memset(&s_gsDeviceConfig, 0, sizeof(s_gsDeviceConfig));
            s_gsDeviceConfig.icount    = GS_USB_CHANNEL_COUNT - 1U;
            s_gsDeviceConfig.swVersion = GsCpuToWire32(2U);
            s_gsDeviceConfig.hwVersion = GsCpuToWire32(1U);
            controlRequest->buffer = (uint8_t *)&s_gsDeviceConfig;
            controlRequest->length = sizeof(s_gsDeviceConfig);
            return true;

        case GS_USB_BREQ_BT_CONST:
            if (controlRequest->isSetup == 0U)
            {
                return false;
            }
            if (((setup->bmRequestType & USB_REQUEST_TYPE_DIR_MASK) != USB_REQUEST_TYPE_DIR_IN) ||
                (setup->wLength < sizeof(s_gsBtConst)))
            {
                return false;
            }
            channel = GsCanDecodeChannelFromSetup(setup);
            if (!GsCanFillBtConst(channel, &s_gsBtConst))
            {
                return false;
            }
            controlRequest->buffer = (uint8_t *)&s_gsBtConst;
            controlRequest->length = sizeof(s_gsBtConst);
            return true;

        case GS_USB_BREQ_BITTIMING:
            if (controlRequest->isSetup != 0U)
            {
                if (((setup->bmRequestType & USB_REQUEST_TYPE_DIR_MASK) != USB_REQUEST_TYPE_DIR_OUT) ||
                    (setup->wLength != sizeof(s_gsBitTiming)))
                {
                    return false;
                }
                controlRequest->buffer = (uint8_t *)&s_gsBitTiming;
                controlRequest->length = sizeof(s_gsBitTiming);
                return true;
            }

            channel = GsCanDecodeChannelFromSetup(setup);
            if (channel >= GS_USB_CHANNEL_COUNT)
            {
                return false;
            }
            s_gsBitTimingByChannel[channel] = s_gsBitTiming;
            s_gsBitTimingValid[channel]      = true;
            return true;

        case GS_USB_BREQ_MODE:
            if (controlRequest->isSetup != 0U)
            {
                if (((setup->bmRequestType & USB_REQUEST_TYPE_DIR_MASK) != USB_REQUEST_TYPE_DIR_OUT) ||
                    (setup->wLength != sizeof(s_gsModeRequest)))
                {
                    return false;
                }
                controlRequest->buffer = (uint8_t *)&s_gsModeRequest;
                controlRequest->length = sizeof(s_gsModeRequest);
                return true;
            }
            channel = GsCanDecodeChannelFromSetup(setup);
            if (channel >= GS_USB_CHANNEL_COUNT)
            {
                return false;
            }
            {
                uint32_t mode      = GsWireToCpu32(s_gsModeRequest.mode);
                uint32_t rawFlags  = GsWireToCpu32(s_gsModeRequest.flags);
                uint32_t flags     = rawFlags & GS_CAN_MODE_SUPPORTED_MASK;
                if (mode == GS_CAN_MODE_START)
                {
                    uint32_t canClock;
                    uint32_t brp;
                    uint32_t propSeg;
                    uint32_t phaseSeg1;
                    uint32_t phaseSeg2;
                    uint32_t tq;
                    uint32_t bitRate;

                    if (!s_gsBitTimingValid[channel])
                    {
                        return false;
                    }

                    canClock  = CLOCK_GetClockRootFreq(kCLOCK_CanClkRoot);
                    if (canClock == 0U)
                    {
                        canClock = 20000000U;
                    }
                    brp       = GsWireToCpu32(s_gsBitTimingByChannel[channel].brp);
                    propSeg   = GsWireToCpu32(s_gsBitTimingByChannel[channel].propSeg);
                    phaseSeg1 = GsWireToCpu32(s_gsBitTimingByChannel[channel].phaseSeg1);
                    phaseSeg2 = GsWireToCpu32(s_gsBitTimingByChannel[channel].phaseSeg2);
                    tq        = 1U + propSeg + phaseSeg1 + phaseSeg2;

                    if ((canClock == 0U) || (brp == 0U) || (tq == 0U))
                    {
                        return false;
                    }

                    bitRate = canClock / (brp * tq);
                    if (bitRate == 0U)
                    {
                        return false;
                    }

                    s_gsCanDeferredReq[channel].bitRate = bitRate;
                    s_gsCanDeferredReq[channel].flags   = flags;
                    s_gsCanDeferredReq[channel].op      = GS_CAN_DEFERRED_OP_START;
                    s_gsCanDeferredReq[channel].pending = 1U;
                }
                else if (mode == GS_CAN_MODE_RESET)
                {
                    s_gsCanDeferredReq[channel].flags   = 0U;
                    s_gsCanDeferredReq[channel].bitRate = 0U;
                    s_gsCanDeferredReq[channel].op      = GS_CAN_DEFERRED_OP_RESET;
                    s_gsCanDeferredReq[channel].pending = 1U;
                }
                else
                {
                    return false;
                }
            }
            return true;

        case GS_USB_BREQ_BERR:
            if (controlRequest->isSetup != 0U)
            {
                if (((setup->bmRequestType & USB_REQUEST_TYPE_DIR_MASK) != USB_REQUEST_TYPE_DIR_OUT) ||
                    (setup->wLength != sizeof(s_gsBerrRequest)))
                {
                    return false;
                }
                controlRequest->buffer = (uint8_t *)&s_gsBerrRequest;
                controlRequest->length = sizeof(s_gsBerrRequest);
                return true;
            }
            return true;

        default:
            return false;
    }
}

static usb_status_t USB_GsCanBulkOutCallback(usb_device_handle handle,
                                             usb_device_endpoint_callback_message_struct_t *message,
                                             void *callbackParam)
{
    (void)handle;
    (void)callbackParam;

    if (message != NULL)
    {
        s_gsCanOutSize = message->length;
        s_gsCanOutPrimed = false;
    }
    return kStatus_USB_Success;
}

static usb_status_t USB_GsCanBulkInCallback(usb_device_handle handle,
                                            usb_device_endpoint_callback_message_struct_t *message,
                                            void *callbackParam)
{
    (void)handle;
    (void)message;
    (void)callbackParam;

    s_gsCanInBusy = false;
    return kStatus_USB_Success;
}

/*******************************************************************************
 * USB callbacks
 ******************************************************************************/
static usb_status_t USB_DeviceCdcVcomCallback(class_handle_t handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    usb_device_cdc_acm_request_param_struct_t *acmReqParam = (usb_device_cdc_acm_request_param_struct_t *)param;
    usb_device_endpoint_callback_message_struct_t *epCbParam = (usb_device_endpoint_callback_message_struct_t *)param;

    switch (event)
    {
        case kUSB_DeviceCdcEventSendResponse:
            if ((epCbParam->length != 0U) &&
                ((epCbParam->length % g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize) == 0U))
            {
                if (USB_DeviceCdcAcmSend(handle, USB_CDC_VCOM_BULK_IN_ENDPOINT, NULL, 0U) != kStatus_USB_Success)
                {
                    s_txBusy = 0U;
                }
            }
            else
            {
                s_txBusy = 0U;
            }
            error = kStatus_USB_Success;
            break;

        case kUSB_DeviceCdcEventRecvResponse:
            if (s_cdcState.attach == 1U)
            {
                s_recvSize = epCbParam->length;
                s_cdcOutPrimed = false;
                error      = kStatus_USB_Success;
                if (s_recvSize == 0U)
                {
                    error = USB_DeviceCdcAcmRecv(
                        handle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_recvBuffer, g_UsbDeviceCdcVcomDicEndpoints[1].maxPacketSize);
                    if (error == kStatus_USB_Success)
                    {
                        s_cdcOutPrimed = true;
                    }
                }
            }
            break;

        case kUSB_DeviceCdcEventSerialStateNotif:
            ((usb_device_cdc_acm_struct_t *)handle)->hasSentState = 0U;
            error                                                 = kStatus_USB_Success;
            break;

        case kUSB_DeviceCdcEventGetLineCoding:
            *(acmReqParam->buffer) = s_lineCoding;
            *(acmReqParam->length) = LINE_CODING_SIZE;
            error                  = kStatus_USB_Success;
            break;

        case kUSB_DeviceCdcEventSetLineCoding:
            if (acmReqParam->isSetup == 1U)
            {
                *(acmReqParam->buffer) = s_lineCoding;
                *(acmReqParam->length) = sizeof(s_lineCoding);
            }
            error = kStatus_USB_Success;
            break;

        case kUSB_DeviceCdcEventSetCommFeature:
            if (USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE == acmReqParam->setupValue)
            {
                if (acmReqParam->isSetup == 1U)
                {
                    *(acmReqParam->buffer) = s_abstractState;
                    *(acmReqParam->length) = sizeof(s_abstractState);
                }
                error = kStatus_USB_Success;
            }
            else if (USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING == acmReqParam->setupValue)
            {
                if (acmReqParam->isSetup == 1U)
                {
                    *(acmReqParam->buffer) = s_countryCode;
                    *(acmReqParam->length) = sizeof(s_countryCode);
                }
                error = kStatus_USB_Success;
            }
            break;

        case kUSB_DeviceCdcEventGetCommFeature:
            if (USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE == acmReqParam->setupValue)
            {
                *(acmReqParam->buffer) = s_abstractState;
                *(acmReqParam->length) = COMM_FEATURE_DATA_SIZE;
                error                  = kStatus_USB_Success;
            }
            else if (USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING == acmReqParam->setupValue)
            {
                *(acmReqParam->buffer) = s_countryCode;
                *(acmReqParam->length) = COMM_FEATURE_DATA_SIZE;
                error                  = kStatus_USB_Success;
            }
            break;

        case kUSB_DeviceCdcEventSetControlLineState:
        {
            uint32_t len;
            uint8_t *uartBitmap;
            uint8_t wasStarted = s_cdcState.startTransactions;

            s_cdcAcmInfo.dteStatus = (uint8_t)acmReqParam->setupValue;
            if ((s_cdcAcmInfo.dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE) != 0U)
            {
                s_cdcAcmInfo.uartState |= (USB_DEVICE_CDC_UART_STATE_RX_CARRIER | USB_DEVICE_CDC_UART_STATE_TX_CARRIER);
            }
            else
            {
                s_cdcAcmInfo.uartState &=
                    (uint16_t)(~(USB_DEVICE_CDC_UART_STATE_RX_CARRIER | USB_DEVICE_CDC_UART_STATE_TX_CARRIER));
            }
            s_cdcAcmInfo.dtePresent =
                ((s_cdcAcmInfo.dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE) != 0U);

            s_cdcAcmInfo.serialStateBuf[0] = NOTIF_REQUEST_TYPE;
            s_cdcAcmInfo.serialStateBuf[1] = USB_DEVICE_CDC_NOTIF_SERIAL_STATE;
            s_cdcAcmInfo.serialStateBuf[2] = 0x00U;
            s_cdcAcmInfo.serialStateBuf[3] = 0x00U;
            s_cdcAcmInfo.serialStateBuf[4] = (uint8_t)acmReqParam->interfaceIndex;
            s_cdcAcmInfo.serialStateBuf[5] = 0x00U;
            s_cdcAcmInfo.serialStateBuf[6] = UART_BITMAP_SIZE;
            s_cdcAcmInfo.serialStateBuf[7] = 0x00U;

            uartBitmap    = &s_cdcAcmInfo.serialStateBuf[NOTIF_PACKET_SIZE + UART_BITMAP_SIZE - 2U];
            uartBitmap[0] = (uint8_t)(s_cdcAcmInfo.uartState & 0xFFU);
            uartBitmap[1] = (uint8_t)((s_cdcAcmInfo.uartState >> 8U) & 0xFFU);

#if ((defined USB_DEVICE_CONFIG_CDC_CIC_EP_DISABLE) && (USB_DEVICE_CONFIG_CDC_CIC_EP_DISABLE > 0U))
            error = kStatus_USB_Success;
#else
            len = (uint32_t)(NOTIF_PACKET_SIZE + UART_BITMAP_SIZE);
            if (((usb_device_cdc_acm_struct_t *)handle)->hasSentState == 0U)
            {
                (void)USB_DeviceCdcAcmSend(handle, USB_CDC_VCOM_INTERRUPT_IN_ENDPOINT, s_cdcAcmInfo.serialStateBuf, len);
                ((usb_device_cdc_acm_struct_t *)handle)->hasSentState = 1U;
            }
            error = kStatus_USB_Success;
#endif

            if (s_cdcState.attach == 1U)
            {
                s_cdcState.startTransactions = 1U;
                if (wasStarted == 0U)
                {
                    s_shellSessionReady = 0U;
                }
                if (!s_cdcOutPrimed && (s_recvSize == 0U))
                {
                    if (USB_DeviceCdcAcmRecv(
                            handle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_recvBuffer,
                            g_UsbDeviceCdcVcomDicEndpoints[1].maxPacketSize) == kStatus_USB_Success)
                    {
                        s_cdcOutPrimed = true;
                    }
                }
            }
            else
            {
                s_cdcState.startTransactions = 0U;
                s_shellSessionReady          = 0U;
            }
        }
        break;

        default:
            break;
    }

    return error;
}

static usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint16_t *temp16   = (uint16_t *)param;
    uint8_t *temp8     = (uint8_t *)param;

    switch (event)
    {
        case kUSB_DeviceEventBusReset:
        {
            size_t i;
            s_cdcState.attach               = 0U;
            s_cdcState.startTransactions    = 0U;
            s_cdcState.currentConfiguration = 0U;
            s_shellSessionReady             = 0U;
            s_recvSize                      = 0U;
            s_cdcOutPrimed                  = false;
            s_txBusy                        = 0U;
            s_gsCanConfigured               = false;
            s_gsCanInBusy                   = false;
            s_gsCanOutPrimed                = false;
            s_gsCanEpReady                  = false;
            s_gsCanOutSize                  = 0U;
            s_gsCanTxHead                   = 0U;
            s_gsCanTxTail                   = 0U;
            s_gsCanTxCount                  = 0U;
            for (i = 0U; i < GS_USB_CHANNEL_COUNT; i++)
            {
                s_gsCanDeferredReq[i].op = GS_CAN_DEFERRED_OP_NONE;
                s_gsCanDeferredReq[i].bitRate = 0U;
                s_gsCanDeferredReq[i].flags = 0U;
                s_gsCanDeferredReq[i].pending = 0U;
            }
            error                           = kStatus_USB_Success;
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U))
            if (kStatus_USB_Success == USB_DeviceClassGetSpeed(CONTROLLER_ID, &s_cdcState.speed))
            {
                (void)USB_DeviceSetSpeed(handle, s_cdcState.speed);
            }
#endif
            break;
        }

        case kUSB_DeviceEventSetConfiguration:
            if (0U == (*temp8))
            {
                size_t i;
                if (s_gsCanEpReady)
                {
                    (void)USB_DeviceDeinitEndpoint(handle, (uint8_t)(USB_GS_CAN_BULK_IN_ENDPOINT | (USB_IN << 7U)));
                    (void)USB_DeviceDeinitEndpoint(handle, (uint8_t)(USB_GS_CAN_BULK_OUT_ENDPOINT | (USB_OUT << 7U)));
                }
                s_cdcState.attach               = 0U;
                s_cdcState.startTransactions    = 0U;
                s_cdcState.currentConfiguration = 0U;
                s_shellSessionReady             = 0U;
                s_recvSize                      = 0U;
                s_cdcOutPrimed                  = false;
                s_gsCanConfigured               = false;
                s_gsCanInBusy                   = false;
                s_gsCanOutPrimed                = false;
                s_gsCanEpReady                  = false;
                s_gsCanOutSize                  = 0U;
                for (i = 0U; i < GS_USB_CHANNEL_COUNT; i++)
                {
                    s_gsCanDeferredReq[i].op = GS_CAN_DEFERRED_OP_NONE;
                    s_gsCanDeferredReq[i].bitRate = 0U;
                    s_gsCanDeferredReq[i].flags = 0U;
                    s_gsCanDeferredReq[i].pending = 0U;
                }
                error                           = kStatus_USB_Success;
            }
            else if (USB_CDC_VCOM_CONFIGURE_INDEX == (*temp8))
            {
                usb_status_t cdcStatus;
                usb_status_t gsStatus;
                usb_device_endpoint_init_struct_t epInit;
                usb_device_endpoint_callback_struct_t epCb;
                size_t i;

                s_cdcState.attach               = 1U;
                s_cdcState.currentConfiguration = *temp8;
                s_cdcState.startTransactions    = 0U;
                s_recvSize                      = 0U;
                s_cdcOutPrimed                  = false;
                s_gsCanOutSize                  = 0U;
                for (i = 0U; i < GS_USB_CHANNEL_COUNT; i++)
                {
                    s_gsCanDeferredReq[i].op = GS_CAN_DEFERRED_OP_NONE;
                    s_gsCanDeferredReq[i].bitRate = 0U;
                    s_gsCanDeferredReq[i].flags = 0U;
                    s_gsCanDeferredReq[i].pending = 0U;
                }
                cdcStatus = USB_DeviceCdcAcmRecv(
                    s_cdcState.cdcAcmHandle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_recvBuffer,
                    g_UsbDeviceCdcVcomDicEndpoints[1].maxPacketSize);
                if (cdcStatus == kStatus_USB_Success)
                {
                    s_cdcOutPrimed = true;
                }
                error = kStatus_USB_Success;

                if (s_gsCanEpReady)
                {
                    (void)USB_DeviceDeinitEndpoint(handle, (uint8_t)(USB_GS_CAN_BULK_IN_ENDPOINT | (USB_IN << 7U)));
                    (void)USB_DeviceDeinitEndpoint(handle, (uint8_t)(USB_GS_CAN_BULK_OUT_ENDPOINT | (USB_OUT << 7U)));
                }

                epCb.callbackParam = NULL;
                epCb.isBusy        = 0U;

                epInit.zlt             = 0U;
                epInit.transferType    = USB_ENDPOINT_BULK;
                epInit.interval        = 0U;
                epInit.endpointAddress = (uint8_t)(USB_GS_CAN_BULK_IN_ENDPOINT | (USB_IN << 7U));
                epInit.maxPacketSize   = g_UsbDeviceGsCanEndpoints[0].maxPacketSize;
                epCb.callbackFn        = USB_GsCanBulkInCallback;
                gsStatus               = USB_DeviceInitEndpoint(handle, &epInit, &epCb);
                if (gsStatus != kStatus_USB_Success)
                {
                    error = gsStatus;
                    break;
                }

                epInit.endpointAddress = (uint8_t)(USB_GS_CAN_BULK_OUT_ENDPOINT | (USB_OUT << 7U));
                epInit.maxPacketSize   = g_UsbDeviceGsCanEndpoints[1].maxPacketSize;
                epCb.callbackFn        = USB_GsCanBulkOutCallback;
                gsStatus               = USB_DeviceInitEndpoint(handle, &epInit, &epCb);
                if (gsStatus != kStatus_USB_Success)
                {
                    (void)USB_DeviceDeinitEndpoint(handle, (uint8_t)(USB_GS_CAN_BULK_IN_ENDPOINT | (USB_IN << 7U)));
                    error = gsStatus;
                    break;
                }

                gsStatus = USB_DeviceRecvRequest(handle, (uint8_t)(USB_GS_CAN_BULK_OUT_ENDPOINT | (USB_OUT << 7U)),
                                                 s_gsCanOutBuffer, g_UsbDeviceGsCanEndpoints[1].maxPacketSize);
                if (gsStatus != kStatus_USB_Success)
                {
                    (void)USB_DeviceDeinitEndpoint(handle, (uint8_t)(USB_GS_CAN_BULK_IN_ENDPOINT | (USB_IN << 7U)));
                    (void)USB_DeviceDeinitEndpoint(handle, (uint8_t)(USB_GS_CAN_BULK_OUT_ENDPOINT | (USB_OUT << 7U)));
                    error = gsStatus;
                    break;
                }

                s_gsCanConfigured = true;
                s_gsCanOutPrimed  = true;
                s_gsCanEpReady    = true;
                s_gsCanInBusy     = false;
            }
            break;

        case kUSB_DeviceEventSetInterface:
            if (0U != s_cdcState.attach)
            {
                uint8_t interface        = (uint8_t)((*temp16 & 0xFF00U) >> 8U);
                uint8_t alternateSetting = (uint8_t)(*temp16 & 0x00FFU);
                if ((interface == USB_GS_CAN_INTERFACE_INDEX) && (alternateSetting == 0U))
                {
                    s_cdcState.currentInterfaceAlternateSetting[interface] = alternateSetting;
                    error = kStatus_USB_Success;
                }
                else if ((interface == USB_CDC_VCOM_COMM_INTERFACE_INDEX) &&
                    (alternateSetting < USB_CDC_VCOM_COMM_INTERFACE_ALTERNATE_COUNT))
                {
                    s_cdcState.currentInterfaceAlternateSetting[interface] = alternateSetting;
                    error = kStatus_USB_Success;
                }
                else if ((interface == USB_CDC_VCOM_DATA_INTERFACE_INDEX) &&
                         (alternateSetting < USB_CDC_VCOM_DATA_INTERFACE_ALTERNATE_COUNT))
                {
                    s_cdcState.currentInterfaceAlternateSetting[interface] = alternateSetting;
                    error = kStatus_USB_Success;
                }
            }
            break;

        case kUSB_DeviceEventGetConfiguration:
            if (param != NULL)
            {
                *temp8 = s_cdcState.currentConfiguration;
                error  = kStatus_USB_Success;
            }
            break;

        case kUSB_DeviceEventGetInterface:
            if (param != NULL)
            {
                uint8_t interface = (uint8_t)((*temp16 & 0xFF00U) >> 8U);
                if (interface < USB_DEVICE_INTERFACE_COUNT)
                {
                    *temp16 = (*temp16 & 0xFF00U) | s_cdcState.currentInterfaceAlternateSetting[interface];
                    error   = kStatus_USB_Success;
                }
            }
            break;

        case kUSB_DeviceEventVendorRequest:
            if (param != NULL)
            {
                if (GsCanHandleVendorRequest((usb_device_control_request_struct_t *)param))
                {
                    error = kStatus_USB_Success;
                }
                else
                {
                    error = kStatus_USB_InvalidRequest;
                }
            }
            break;

        case kUSB_DeviceEventGetDeviceDescriptor:
            if (param != NULL)
            {
                error = USB_DeviceGetDeviceDescriptor(handle, (usb_device_get_device_descriptor_struct_t *)param);
            }
            break;

        case kUSB_DeviceEventGetConfigurationDescriptor:
            if (param != NULL)
            {
                error = USB_DeviceGetConfigurationDescriptor(handle,
                                                             (usb_device_get_configuration_descriptor_struct_t *)param);
            }
            break;

        case kUSB_DeviceEventGetStringDescriptor:
            if (param != NULL)
            {
                error = USB_DeviceGetStringDescriptor(handle, (usb_device_get_string_descriptor_struct_t *)param);
            }
            break;

        default:
            break;
    }

    return error;
}

/*******************************************************************************
 * USB clock/init
 ******************************************************************************/
static void USB_DeviceClockInit(void)
{
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL,
        BOARD_USB_PHY_TXCAL45DP,
        BOARD_USB_PHY_TXCAL45DM,
    };

    if (CONTROLLER_ID == kUSB_ControllerEhci0)
    {
        CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000U);
    }
    else
    {
        CLOCK_EnableUsbhs1PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs1Clock(kCLOCK_Usb480M, 480000000U);
    }
    (void)USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);
}

static void USB_DeviceIsrEnable(void)
{
    uint8_t irqNumber;
    uint8_t usbDeviceEhciIrq[] = USBHS_IRQS;

    irqNumber = usbDeviceEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
    EnableIRQ((IRQn_Type)irqNumber);
}

static void USB_DeviceApplicationInit(void)
{
    size_t i;

    USB_DeviceClockInit();
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
    SYSMPU_Enable(SYSMPU, 0U);
#endif

    s_cdcState.speed        = USB_SPEED_FULL;
    s_cdcState.attach       = 0U;
    s_cdcState.cdcAcmHandle = NULL;
    s_cdcState.deviceHandle = NULL;
    s_gsCanConfigured       = false;
    s_gsCanInBusy           = false;
    s_gsCanOutPrimed        = false;
    s_gsCanEpReady          = false;
    s_gsCanOutSize          = 0U;
    s_gsCanTxHead           = 0U;
    s_gsCanTxTail           = 0U;
    s_gsCanTxCount          = 0U;
    s_cdcOutPrimed          = false;
    for (i = 0U; i < GS_USB_CHANNEL_COUNT; i++)
    {
        s_gsCanDeferredReq[i].op = GS_CAN_DEFERRED_OP_NONE;
        s_gsCanDeferredReq[i].bitRate = 0U;
        s_gsCanDeferredReq[i].flags = 0U;
        s_gsCanDeferredReq[i].pending = 0U;
    }

    if (kStatus_USB_Success == USB_DeviceClassInit(CONTROLLER_ID, &s_cdcAcmConfigList, &s_cdcState.deviceHandle))
    {
        s_cdcState.cdcAcmHandle = s_cdcAcmConfigList.config->classHandle;
        USB_DeviceIsrEnable();
        SDK_DelayAtLeastUs(5000U, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
        USB_DeviceRun(s_cdcState.deviceHandle);
    }
}

/*******************************************************************************
 * App task
 ******************************************************************************/
static void LedTask(void *pvParameters)
{
    led_mode_t appliedMode   = kLedModeBlink;

    (void)pvParameters;

    for (;;)
    {
        led_mode_t mode = s_ledMode;

        if (mode == kLedModeBlink)
        {
            TickType_t now = xTaskGetTickCount();
            bool ledOn = false;

            if (s_ledActivitySeen &&
                ((now - s_ledActivityLastTick) <= pdMS_TO_TICKS(LED_ACTIVITY_PULSE_MS)))
            {
                ledOn = true;
            }

            if (ledOn)
            {
                USER_LED_ON();
            }
            else
            {
                USER_LED_OFF();
            }
            appliedMode = kLedModeBlink;
            vTaskDelay(pdMS_TO_TICKS(5U));
        }
        else if ((mode == kLedModeOn) && (appliedMode != kLedModeOn))
        {
            USER_LED_ON();
            appliedMode = kLedModeOn;
            vTaskDelay(pdMS_TO_TICKS(20U));
        }
        else if ((mode == kLedModeOff) && (appliedMode != kLedModeOff))
        {
            USER_LED_OFF();
            appliedMode = kLedModeOff;
            vTaskDelay(pdMS_TO_TICKS(20U));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(20U));
        }
    }
}

static void CanTask(void *pvParameters)
{
    uint32_t counter = 0U;
    TickType_t lastHeartbeatTick = xTaskGetTickCount();

    (void)pvParameters;

    for (;;)
    {
        TickType_t now = xTaskGetTickCount();

        if (s_canReady && s_canHeartbeatEnabled &&
            ((now - lastHeartbeatTick) >= pdMS_TO_TICKS(CAN_HEARTBEAT_PERIOD_MS)))
        {
            size_t i;
            uint8_t payload[8];

            payload[0] = (uint8_t)(counter & 0xFFU);
            payload[1] = (uint8_t)((counter >> 8U) & 0xFFU);
            payload[2] = (uint8_t)((counter >> 16U) & 0xFFU);
            payload[3] = (uint8_t)((counter >> 24U) & 0xFFU);
            payload[4] = 0xCAU;
            payload[5] = 0x11U;
            payload[6] = 0xCAU;
            payload[7] = 0x22U;

            for (i = 0U; i < ARRAY_SIZE(s_canBuses); i++)
            {
                if (s_canBuses[i].started)
                {
                    (void)CanSendFrame(&s_canBuses[i], s_canBuses[i].heartbeatMb, s_canBuses[i].heartbeatId, payload, 8U);
                }
            }

            counter++;
            lastHeartbeatTick = now;
        }

        CanPollRxDump();
        CanUpdateUtilization(now);

        if (s_canReady && s_canDumpEnabled)
        {
            vTaskDelay(pdMS_TO_TICKS(CAN_DUMP_POLL_PERIOD_MS));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(10U));
        }
    }
}

static void CanLogTask(void *pvParameters)
{
    TickType_t lastFlushTick = xTaskGetTickCount();
    uint32_t framesSinceSync = 0U;
    can_log_record_t record;

    (void)pvParameters;

    for (;;)
    {
        if (!s_fsMounted)
        {
            if (FsMount() != FR_OK)
            {
                vTaskDelay(pdMS_TO_TICKS(CAN_LOG_MOUNT_RETRY_MS));
                continue;
            }
        }

        if (!s_canLogFileOpen)
        {
            FRESULT fr = f_open(&s_canLogFile, CAN_LOG_PATH, FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
            if (fr != FR_OK)
            {
                vTaskDelay(pdMS_TO_TICKS(CAN_LOG_MOUNT_RETRY_MS));
                continue;
            }
            s_canLogFileOpen = true;
            lastFlushTick    = xTaskGetTickCount();
            framesSinceSync  = 0U;

            if (!Mf4RecoverFile(&s_canLogFile, &s_mf4LogState))
            {
                (void)f_close(&s_canLogFile);
                s_canLogFileOpen = false;
                s_mf4LogState.ready = false;
                s_fsMounted      = false;
                (void)f_mount(NULL, s_fsDrive, 0U);
                vTaskDelay(pdMS_TO_TICKS(CAN_LOG_MOUNT_RETRY_MS));
                continue;
            }
        }

        if (xQueueReceive(s_canLogQueue, &record, pdMS_TO_TICKS(50U)) == pdTRUE)
        {
            UINT written = 0U;
            FRESULT fr   = f_write(&s_canLogFile, &record, sizeof(record), &written);
            if ((fr != FR_OK) || (written != sizeof(record)))
            {
                (void)f_close(&s_canLogFile);
                s_canLogFileOpen = false;
                s_mf4LogState.ready = false;
                s_fsMounted      = false;
                (void)f_mount(NULL, s_fsDrive, 0U);
                vTaskDelay(pdMS_TO_TICKS(CAN_LOG_MOUNT_RETRY_MS));
                continue;
            }
            s_mf4LogState.recordCount++;
            s_mf4LogState.dataBytes += sizeof(record);

            framesSinceSync++;
            if (framesSinceSync >= CAN_LOG_SYNC_FRAME_INTERVAL)
            {
                if (!Mf4Sync(&s_canLogFile, &s_mf4LogState))
                {
                    (void)f_close(&s_canLogFile);
                    s_canLogFileOpen = false;
                    s_mf4LogState.ready = false;
                    s_fsMounted      = false;
                    (void)f_mount(NULL, s_fsDrive, 0U);
                    vTaskDelay(pdMS_TO_TICKS(CAN_LOG_MOUNT_RETRY_MS));
                    continue;
                }
                framesSinceSync = 0U;
                lastFlushTick   = xTaskGetTickCount();
            }
        }

        if (s_canLogFileOpen &&
            ((xTaskGetTickCount() - lastFlushTick) >= pdMS_TO_TICKS(CAN_LOG_FLUSH_PERIOD_MS)))
        {
            if (!Mf4Sync(&s_canLogFile, &s_mf4LogState))
            {
                (void)f_close(&s_canLogFile);
                s_canLogFileOpen = false;
                s_mf4LogState.ready = false;
                s_fsMounted      = false;
                (void)f_mount(NULL, s_fsDrive, 0U);
                vTaskDelay(pdMS_TO_TICKS(CAN_LOG_MOUNT_RETRY_MS));
                continue;
            }
            lastFlushTick  = xTaskGetTickCount();
            framesSinceSync = 0U;
        }
    }
}

static void AppTask(void *pvParameters)
{
    uint8_t packet[DATA_BUFF_SIZE];
    uint8_t gsPacket[GS_USB_MAX_PACKET_SIZE];

    (void)pvParameters;
    USB_DeviceApplicationInit();

    for (;;)
    {
        if (s_gsCanConfigured)
        {
            GsCanProcessDeferredRequests();
        }

        if (s_gsCanConfigured && s_gsCanEpReady)
        {
            if ((s_gsCanOutSize != 0U) && (s_gsCanOutSize != USB_CANCELLED_TRANSFER_LENGTH))
            {
                uint32_t gsSize = s_gsCanOutSize;
                if (gsSize > sizeof(gsPacket))
                {
                    gsSize = sizeof(gsPacket);
                }

                (void)memcpy(gsPacket, s_gsCanOutBuffer, gsSize);
                s_gsCanOutSize = 0U;
                GsCanHandleBulkOutPacket(gsPacket, gsSize);

                if (USB_DeviceRecvRequest(s_cdcState.deviceHandle, (uint8_t)(USB_GS_CAN_BULK_OUT_ENDPOINT | (USB_OUT << 7U)),
                                          s_gsCanOutBuffer, g_UsbDeviceGsCanEndpoints[1].maxPacketSize) ==
                    kStatus_USB_Success)
                {
                    s_gsCanOutPrimed = true;
                }
            }
            else if (!s_gsCanOutPrimed)
            {
                if (USB_DeviceRecvRequest(s_cdcState.deviceHandle, (uint8_t)(USB_GS_CAN_BULK_OUT_ENDPOINT | (USB_OUT << 7U)),
                                          s_gsCanOutBuffer, g_UsbDeviceGsCanEndpoints[1].maxPacketSize) ==
                    kStatus_USB_Success)
                {
                    s_gsCanOutPrimed = true;
                }
            }

            GsCanTrySendNext();
        }

        if (s_cdcState.attach == 1U)
        {
            if (!s_cdcOutPrimed && (s_recvSize == 0U))
            {
                if (USB_DeviceCdcAcmRecv(
                        s_cdcState.cdcAcmHandle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_recvBuffer,
                        g_UsbDeviceCdcVcomDicEndpoints[1].maxPacketSize) == kStatus_USB_Success)
                {
                    s_cdcOutPrimed = true;
                }
            }
            if (s_shellSessionReady == 0U)
            {
                s_shellSessionReady = 1U;
                ShellSendPrompt();
            }

            if (s_recvSize == USB_CANCELLED_TRANSFER_LENGTH)
            {
                s_recvSize = 0U;
                s_cdcOutPrimed = false;
            }

            if ((s_recvSize != 0U) && (s_recvSize != USB_CANCELLED_TRANSFER_LENGTH))
            {
                uint32_t rxSize = s_recvSize;
                if (rxSize > DATA_BUFF_SIZE)
                {
                    rxSize = DATA_BUFF_SIZE;
                }

                (void)memcpy(packet, s_recvBuffer, rxSize);
                s_recvSize = 0U;
                s_cdcOutPrimed = false;

                ShellHandlePacket(packet, rxSize);

                if (USB_DeviceCdcAcmRecv(
                        s_cdcState.cdcAcmHandle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_recvBuffer,
                        g_UsbDeviceCdcVcomDicEndpoints[1].maxPacketSize) == kStatus_USB_Success)
                {
                    s_cdcOutPrimed = true;
                }
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(2U));
            }
        }
        else
        {
            s_shellSessionReady = 0U;
            s_cdcOutPrimed = false;
            vTaskDelay(pdMS_TO_TICKS(2U));
        }
    }
}

/*******************************************************************************
 * Main
 ******************************************************************************/
int main(void)
{
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    SystemCoreClockUpdate();

    USER_LED_INIT(LOGIC_LED_OFF);
    (void)RtcInit();
    ShellCliInit();

    s_canLogQueue = xQueueCreate(CAN_LOG_QUEUE_LENGTH, sizeof(can_log_record_t));
    s_shellTxMutex = xSemaphoreCreateMutex();
    if (s_shellTxMutex == NULL)
    {
        while (1)
        {
        }
    }

    if (xTaskCreate(LedTask, "led", LED_TASK_STACK_WORDS, NULL, LED_TASK_PRIORITY, NULL) != pdPASS)
    {
        while (1)
        {
        }
    }

    if (xTaskCreate(CanTask, "can", CAN_TASK_STACK_WORDS, NULL, CAN_TASK_PRIORITY, NULL) != pdPASS)
    {
        while (1)
        {
        }
    }

    if (s_canLogQueue != NULL)
    {
        if (xTaskCreate(CanLogTask, "can_log", CAN_LOG_TASK_STACK_WORDS, NULL, CAN_LOG_TASK_PRIORITY, NULL) != pdPASS)
        {
            s_canLogQueue = NULL;
        }
    }

    if (xTaskCreate(AppTask, "usb_shell", APP_TASK_STACK_WORDS, NULL, APP_TASK_PRIORITY, NULL) != pdPASS)
    {
        while (1)
        {
        }
    }

    vTaskStartScheduler();

    while (1)
    {
    }
}
