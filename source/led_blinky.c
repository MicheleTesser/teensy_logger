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

/*
 * Split implementation units.
 * We keep a single translation unit on purpose to preserve static state visibility
 * while making manual inspection easier by topic.
 */
/* Module map:
 * - led_blinky_usb_shell.inc: USB IRQ + shell TX/RX + CLI + gs_usb basic helpers.
 * - led_blinky_rtc_fs.inc: RTC/SNVS + FAT timestamp + filesystem commands.
 * - led_blinky_can_core_mf4.inc: CAN low-level init/TX/RX accounting + MF4 writer core.
 * - led_blinky_can_shell_gs.inc: CAN polling/commands + gs_usb control/data plane.
 * - led_blinky_usb_tasks_main.inc: USB callbacks/init + FreeRTOS tasks + main().
 */
#include "app/led_blinky_usb_shell.inc"
#include "app/led_blinky_rtc_fs.inc"
#include "app/led_blinky_can_core_mf4.inc"
#include "app/led_blinky_can_shell_gs.inc"
#include "app/led_blinky_usb_tasks_main.inc"
