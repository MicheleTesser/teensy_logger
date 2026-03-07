#ifndef APP_SHARED_H
#define APP_SHARED_H

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
#include "usb_device_msc.h"
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
#define APP_TASK_PRIORITY    (tskIDLE_PRIORITY + 3U)
#define LED_TASK_STACK_WORDS 256U
#define LED_TASK_PRIORITY    (tskIDLE_PRIORITY + 2U)
#define CAN_TASK_STACK_WORDS 1280U
#define CAN_TASK_PRIORITY    (tskIDLE_PRIORITY + 3U)

#define LED_ACTIVITY_PULSE_MS   60U
#define CAN_HEARTBEAT_PERIOD_MS 100U
#define CAN_DUMP_POLL_PERIOD_MS 2U
#define CAN_DEFAULT_BITRATE     500000U
#define CAN_HEARTBEAT_MB        8U
#define CAN_SHELL_TX_MB         9U
#define CAN_RX_STD_MB           10U
#define CAN_RX_EXT_MB           11U
#define CAN_GS_TX_MB_FIRST      12U
#define CAN_GS_TX_MB_LAST       31U
#define CAN1_HEARTBEAT_ID       0x321U
#define CAN2_HEARTBEAT_ID       0x322U
#define CAN_AUTOSTART_RETRY_MS  1000U

#define SHELL_PROMPT           "usb-shell> "
#define SHELL_LINE_BUFFER_SIZE 256U
#define SHELL_TX_LINE_SIZE     128U
#define SHELL_CLI_OUTPUT_SIZE  1024U
#define BOOTLOADER_BAUD_TRIGGER 134U
#define BOOTLOADER_REBOOT_DELAY_MS 120U
#define BOOTLOADER_REBOOT_DELAY_CLI_MS 200U

#define FS_BENCH_FILE_PATH    "/bench.bin"
#define FS_BENCH_BUFFER_SIZE  4096U
#define FS_BENCH_DEFAULT_KIB  1024U
#define FS_BENCH_DEFAULT_RUNS 1U

#define CAN_UTIL_UPDATE_MS       200U
#define CAN_ISR_RX_FIFO_LENGTH   1024U
#define CAN_ISR_RX_FIFO_MASK     (CAN_ISR_RX_FIFO_LENGTH - 1U)
#define CAN_GS_TX_QUEUE_LENGTH   2048U
#define CAN_GS_TX_QUEUE_MASK     (CAN_GS_TX_QUEUE_LENGTH - 1U)

#define CAN_LOG_TASK_STACK_WORDS    768U
#define CAN_LOG_TASK_PRIORITY       (tskIDLE_PRIORITY + 2U)
#define CAN_LOG_QUEUE_LENGTH        128U
#define CAN_LOG_FLUSH_PERIOD_MS     1000U
#define CAN_LOG_MOUNT_RETRY_MS      1000U
#define CAN_LOG_SYNC_FRAME_INTERVAL 64U
#define CAN_LOG_USB_DETECT_GRACE_MS 2500U
#define CAN_LOG_RECORD_SIZE_BYTES   24U
#define CAN_LOG_PATH_MAX            32U
#define CAN_LOG_EXPORT_PATH_MAX     96U
#define CAN_LOG_MAX_SEQ_INDEX       100000U

#define SD_USAGE_CACHE_TTL_MS       5000U

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
#define GS_USB_FRAME_QUEUE_LENGTH    2048U

#define GS_CAN_DEFERRED_OP_NONE  (0U)
#define GS_CAN_DEFERRED_OP_START (1U)
#define GS_CAN_DEFERRED_OP_RESET (2U)

#if defined(__GNUC__)
#define PACKED_STRUCT __attribute__((packed))
#else
#define PACKED_STRUCT
#endif

#if defined(__GNUC__) || defined(__clang__)
#define APP_LIKELY(x)   (__builtin_expect(!!(x), 1))
#define APP_UNLIKELY(x) (__builtin_expect(!!(x), 0))
#else
#define APP_LIKELY(x)   (x)
#define APP_UNLIKELY(x) (x)
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

#define GS_USB_HOST_FRAME_SIZE          ((uint32_t)sizeof(gs_host_frame_classic_t))
#define GS_USB_MAX_FRAMES_PER_TRANSFER  (GS_USB_MAX_PACKET_SIZE / GS_USB_HOST_FRAME_SIZE)

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
    bool valid;
    uint64_t totalBytes;
    uint64_t freeBytes;
    uint64_t usedBytes;
    uint16_t usedPermille;
    TickType_t tick;
    FRESULT lastResult;
} sd_usage_cache_t;

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

typedef struct
{
    flexcan_frame_t frame;
    status_t status;
} can_isr_rx_entry_t;
typedef char can_isr_fifo_length_pow2_must_hold[((CAN_ISR_RX_FIFO_LENGTH & (CAN_ISR_RX_FIFO_LENGTH - 1U)) == 0U) ? 1 : -1];

typedef struct
{
    flexcan_frame_t frame;
    uint32_t echoId;
} can_gs_tx_entry_t;
typedef char can_gs_tx_queue_length_pow2_must_hold[((CAN_GS_TX_QUEUE_LENGTH & (CAN_GS_TX_QUEUE_LENGTH - 1U)) == 0U) ? 1 : -1];

/* Cross-module function declarations */
void AppTask(void *pvParameters);
void LedTask(void *pvParameters);
void CanTask(void *pvParameters);
void CanLogTask(void *pvParameters);
void USB_DeviceApplicationInit(void);
void USB_DeviceClockInit(void);
void USB_DeviceIsrEnable(void);
void ShellHandlePacket(const uint8_t *data, uint32_t length);
void ShellHandleLine(char *line);
void ShellSendPrompt(void);
void ShellWrite(const char *text);
void ShellWriteBytes(const uint8_t *data, uint32_t length);
bool ShellWriteTry(const char *text);
bool ShellWriteBytesTry(const uint8_t *data, uint32_t length);
void ShellWriteLinef(const char *fmt, ...);
void ShellCliInit(void);
BaseType_t ShellCliHelpCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t ShellCliLedCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t ShellCliCanCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t ShellCliSdCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t ShellCliRtcCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t ShellCliTimeCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t ShellCliLogCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t ShellCliBootCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
void LedMarkCanActivity(void);
void AppRequestBootloaderReboot(uint32_t delayMs);
void AppCancelBootloaderReboot(void);
void AppProcessBootloaderReboot(void);
void AppRebootToBootloaderNow(void) __attribute__((noreturn));

FRESULT FsMount(void);
FRESULT FsUnmount(void);
bool FsEnsureMounted(void);
void SdUsageInvalidate(void);
FRESULT SdUsageRefresh(bool force);
bool SdUsageSnapshot(sd_usage_cache_t *snapshot);
const char *FsResultToString(FRESULT result);
bool FsParseU32(const char *text, uint32_t *value);
void FsCommand(int32_t argc, char *argv[], const char *rawLine);
void FsCommandList(const char *path);
void FsCommandCat(const char *path);
void FsCommandWrite(const char *path, const char *payload, bool append);
void FsCommandMkDir(const char *path);
void FsCommandRemove(const char *path);
void FsCommandMkfs(void);
void FsCommandBench(uint32_t totalKiB, uint32_t runs);
const char *ShellSkipToken(const char *cursor);
bool ShellParseU32Auto(const char *text, uint32_t *value);
bool ShellParseCanBus(const char *text, uint32_t *busIndex);
bool RtcInit(void);
bool RtcIsValid(void);
uint32_t RtcGetUnixSeconds(void);
bool RtcSetUnixSeconds(uint32_t unixSeconds);
bool RtcAutoSetFromBuildIfInvalid(void);
bool RtcUnixSecondsToDateTime(uint32_t unixSeconds, rtc_datetime_t *dateTime);
void RtcCommand(int32_t argc, char *argv[]);

status_t CanInitAll(uint32_t bitRate, uint32_t modeFlags);
status_t CanReleaseStopMode(CAN_Type *base);
status_t CanInitOneWithFlags(can_bus_context_t *bus, uint32_t bitRate, uint32_t modeFlags);
status_t CanDeinitOne(can_bus_context_t *bus);
void CanPollRxDump(void);
void CanPollRxMb(can_bus_context_t *bus, size_t busIndex, uint8_t mbIdx);
void CanRxFifoReset(size_t busIndex);
bool CanRxFifoPop(size_t busIndex, flexcan_frame_t *frame, status_t *status);
bool CanProcessRxFifo(uint32_t budgetPerBus);
void CanHandleRxFrame(size_t busIndex, const flexcan_frame_t *frame, status_t status);
void CanDumpPrintFrame(size_t busIndex, const flexcan_frame_t *frame, status_t status);
uint32_t CanEstimateFrameBits(const flexcan_frame_t *frame);
void CanUpdateUtilization(TickType_t now);
void CanResetRuntimeStats(void);
status_t CanWriteTxMbWithRecovery(can_bus_context_t *bus, uint8_t mbIdx, const flexcan_frame_t *frame);
status_t CanSendFrame(can_bus_context_t *bus, uint8_t mbIdx, uint32_t id, const uint8_t *payload, uint8_t length);
status_t CanSendFrameRaw(can_bus_context_t *bus, uint8_t mbIdx, const flexcan_frame_t *frame);

uint32_t GsBswap32(uint32_t value);
uint32_t GsToLe32(uint32_t value);
uint32_t GsFromLe32(uint32_t value);
bool GsIsLittleEndianHost(void);
uint32_t GsWireToCpu32(uint32_t value);
uint32_t GsCpuToWire32(uint32_t value);
bool GsCanQueueFrame(const gs_host_frame_classic_t *frame);
void GsCanTrySendNext(void);
void CanRefreshReadyState(void);
bool CanAnyControllerStarted(void);
void CanApplyGsClockRoot(void);
void GsCanHandleBulkOutPacket(const uint8_t *data, uint32_t length);
bool GsCanFillBtConst(uint8_t channel, gs_device_bt_const_t *btConst);
uint8_t GsCanDecodeChannelFromSetup(const usb_setup_struct_t *setup);
bool GsCanHandleVendorRequest(usb_device_control_request_struct_t *controlRequest);
void GsCanProcessDeferredRequests(void);
void GsCanSetOverflowFlag(size_t busIndex);
bool GsCanRxPublish(size_t busIndex, const flexcan_frame_t *frame, status_t status);
bool GsCanTxEchoPublish(size_t busIndex, uint32_t echoId, const flexcan_frame_t *frame);
void GsCanTxQueueReset(size_t busIndex);
bool GsCanTxQueuePush(size_t busIndex, uint32_t echoId, const flexcan_frame_t *frame);
bool GsCanServiceTxQueues(uint32_t budgetPerBus);
bool GsCanTxQueueHasPending(void);
usb_status_t USB_GsCanBulkOutCallback(usb_device_handle handle,
                                             usb_device_endpoint_callback_message_struct_t *message,
                                             void *callbackParam);
usb_status_t USB_GsCanBulkInCallback(usb_device_handle handle,
                                            usb_device_endpoint_callback_message_struct_t *message,
                                            void *callbackParam);

uint32_t Mf4Align8(uint32_t value);
void Mf4PutU16(uint8_t *dst, uint16_t value);
void Mf4PutU32(uint8_t *dst, uint32_t value);
void Mf4PutU64(uint8_t *dst, uint64_t value);
void Mf4PutS16(uint8_t *dst, int16_t value);
void Mf4PutDouble(uint8_t *dst, double value);
bool Mf4WriteAt(FIL *file, FSIZE_t offset, const void *data, UINT size);
bool Mf4ReadAt(FIL *file, FSIZE_t offset, void *data, UINT size);
uint64_t Mf4UnixSecondsToNano(uint32_t unixSeconds);
void Mf4ComputeLayout(uint32_t *txOffsets, uint32_t *dtBlockOffset, uint32_t *dtDataOffset);
bool Mf4WriteTextBlock(FIL *file, uint32_t offset, const char *text);
bool Mf4WriteFreshFile(FIL *file, mf4_log_state_t *state);
bool Mf4RecoverFile(FIL *file, mf4_log_state_t *state);
bool Mf4PatchCounters(FIL *file, const mf4_log_state_t *state);
bool Mf4Sync(FIL *file, const mf4_log_state_t *state);
bool CanLogOpenFile(FIL *file, mf4_log_state_t *state);
FRESULT CanLogExportCsv(const char *srcPath, const char *dstPath, uint32_t *recordsWritten);

bool CanLogEnqueue(size_t busIndex, const flexcan_frame_t *frame, status_t status);

usb_status_t USB_DeviceCdcVcomCallback(class_handle_t handle, uint32_t event, void *param);
usb_status_t USB_DeviceMscCallback(class_handle_t handle, uint32_t event, void *param);
usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param);

DWORD get_fattime(void);

/* Shared globals */
extern usb_device_endpoint_struct_t g_UsbDeviceCdcVcomDicEndpoints[];
extern usb_device_endpoint_struct_t g_UsbDeviceGsCanEndpoints[];
extern usb_device_endpoint_struct_t g_UsbDeviceMscDiskEndpoints[];
extern usb_device_class_struct_t g_UsbDeviceCdcVcomConfig;
extern usb_device_class_struct_t g_UsbDeviceMscDiskConfig;
extern void USDHC1_DriverIRQHandler(void);

extern usb_cdc_shell_state_t s_cdcState;
extern usb_cdc_acm_info_t s_cdcAcmInfo;

extern uint8_t s_lineCoding[LINE_CODING_SIZE];
extern uint8_t s_abstractState[COMM_FEATURE_DATA_SIZE];
extern uint8_t s_countryCode[COMM_FEATURE_DATA_SIZE];
extern uint8_t s_recvBuffer[DATA_BUFF_SIZE];
extern uint8_t s_sendBuffer[DATA_BUFF_SIZE];
extern uint8_t s_gsCanOutBuffer[GS_USB_MAX_PACKET_SIZE];
extern gs_host_frame_classic_t s_gsCanInFrames[GS_USB_MAX_FRAMES_PER_TRANSFER];

extern volatile uint32_t s_recvSize;
extern volatile uint8_t s_txBusy;
extern volatile uint8_t s_shellSessionReady;
extern volatile led_mode_t s_ledMode;
extern volatile TickType_t s_ledActivityLastTick;
extern volatile bool s_ledActivitySeen;
extern volatile bool s_cdcOutPrimed;
extern volatile uint32_t s_gsCanOutSize;
extern volatile bool s_rtcInitialized;
extern volatile bool s_bootloaderRebootPending;
extern volatile TickType_t s_bootloaderRebootDeadline;
extern bool s_shellCliReady;
extern TickType_t s_canUtilLastTick;
extern bool s_canUtilInitialized;
extern uint32_t s_canUtilLastTxBits[2];
extern uint32_t s_canUtilLastRxBits[2];

extern FATFS s_fsObject;
extern bool s_fsConfigured;
extern bool s_fsMounted;
extern const TCHAR s_fsDrive[];
extern FIL s_canLogFile;
extern volatile bool s_canLogFileOpen;
extern char s_canLogActivePath[CAN_LOG_PATH_MAX];
extern uint32_t s_canLogNextIndex;
extern mf4_log_state_t s_mf4LogState;
extern sd_usage_cache_t s_sdUsageCache;

extern const mf4_channel_desc_t s_mf4Channels[MF4_CHANNEL_COUNT];
extern can_bus_context_t s_canBuses[GS_USB_CHANNEL_COUNT];
extern volatile bool s_canReady;
extern volatile bool s_canHeartbeatEnabled;
extern volatile bool s_canDumpEnabled;
extern volatile uint32_t s_canBitRate;
extern volatile uint32_t s_canModeFlags;
extern volatile bool s_canAutoStartEnabled;
extern volatile bool s_logEnabled;
extern volatile bool s_logAllowWhenUsbAttached;
extern volatile bool s_mscHostActive;

extern QueueHandle_t s_canLogQueue;
extern SemaphoreHandle_t s_shellTxMutex;

extern volatile bool s_gsCanConfigured;
extern volatile bool s_gsCanHostLe;
extern volatile bool s_gsCanInBusy;
extern volatile bool s_gsCanOutPrimed;
extern volatile bool s_gsCanEpReady;
extern volatile uint16_t s_gsCanTxHead;
extern volatile uint16_t s_gsCanTxTail;
extern volatile uint16_t s_gsCanTxCount;
extern volatile bool s_gsCanOverflowPending[GS_USB_CHANNEL_COUNT];
extern gs_host_frame_classic_t s_gsCanTxQueue[GS_USB_FRAME_QUEUE_LENGTH];
extern gs_can_deferred_request_t s_gsCanDeferredReq[GS_USB_CHANNEL_COUNT];

extern volatile uint16_t s_canRxFifoHead[GS_USB_CHANNEL_COUNT];
extern volatile uint16_t s_canRxFifoTail[GS_USB_CHANNEL_COUNT];
extern volatile bool s_canRxFifoOverflow[GS_USB_CHANNEL_COUNT];
extern can_isr_rx_entry_t s_canRxFifo[GS_USB_CHANNEL_COUNT][CAN_ISR_RX_FIFO_LENGTH];
extern volatile uint16_t s_canGsTxHead[GS_USB_CHANNEL_COUNT];
extern volatile uint16_t s_canGsTxTail[GS_USB_CHANNEL_COUNT];
extern volatile uint16_t s_canGsTxCount[GS_USB_CHANNEL_COUNT];
extern volatile uint8_t s_canGsTxMbCursor[GS_USB_CHANNEL_COUNT];
extern can_gs_tx_entry_t s_canGsTxQueue[GS_USB_CHANNEL_COUNT][CAN_GS_TX_QUEUE_LENGTH];

extern gs_host_config_t s_gsHostConfig;
extern gs_device_bittiming_t s_gsBitTiming;
extern gs_device_mode_t s_gsModeRequest;
extern uint32_t s_gsBerrRequest;
extern volatile uint32_t s_gsCanBtClockHz;
extern gs_device_config_t s_gsDeviceConfig;
extern gs_device_bt_const_t s_gsBtConst;
extern gs_device_bittiming_t s_gsBitTimingByChannel[GS_USB_CHANNEL_COUNT];
extern bool s_gsBitTimingValid[GS_USB_CHANNEL_COUNT];

extern usb_device_class_config_list_struct_t s_cdcAcmConfigList;

#endif /* APP_SHARED_H */
