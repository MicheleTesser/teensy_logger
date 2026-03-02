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

#include "FreeRTOS.h"
#include "task.h"

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

#define APP_TASK_STACK_WORDS 1024U
#define APP_TASK_PRIORITY    (tskIDLE_PRIORITY + 3U)
#define LED_BLINK_PERIOD_MS  300U

#define SHELL_PROMPT           "usb-shell> "
#define SHELL_LINE_BUFFER_SIZE 256U
#define SHELL_TX_LINE_SIZE     128U

#define FS_BENCH_FILE_PATH    "/bench.bin"
#define FS_BENCH_BUFFER_SIZE  4096U
#define FS_BENCH_DEFAULT_KIB  1024U
#define FS_BENCH_DEFAULT_RUNS 1U

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
    uint8_t currentInterfaceAlternateSetting[USB_CDC_VCOM_INTERFACE_COUNT];
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

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void AppTask(void *pvParameters);
static void USB_DeviceApplicationInit(void);
static void USB_DeviceClockInit(void);
static void USB_DeviceIsrEnable(void);
static void ShellHandlePacket(const uint8_t *data, uint32_t length);
static void ShellHandleLine(char *line);
static void ShellSendPrompt(void);
static void ShellWrite(const char *text);
static void ShellWriteBytes(const uint8_t *data, uint32_t length);
static void ShellWriteLinef(const char *fmt, ...);
static void ShellPrintHelp(void);

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

static usb_status_t USB_DeviceCdcVcomCallback(class_handle_t handle, uint32_t event, void *param);
static usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param);

/*******************************************************************************
 * Globals
 ******************************************************************************/
extern usb_device_endpoint_struct_t g_UsbDeviceCdcVcomDicEndpoints[];
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

static volatile uint32_t s_recvSize = 0U;
static volatile uint8_t s_txBusy = 0U;
static volatile uint8_t s_shellSessionReady = 0U;
static led_mode_t s_ledMode = kLedModeBlink;

static FATFS s_fsObject;
static bool s_fsConfigured = false;
static bool s_fsMounted = false;
static const TCHAR s_fsDrive[] = {SDDISK + '0', ':', '/', '\0'};

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
    if ((data == NULL) || (length == 0U))
    {
        return;
    }

    if ((s_cdcState.attach == 0U) || (s_cdcState.startTransactions == 0U) || (s_cdcState.cdcAcmHandle == NULL))
    {
        return;
    }

    while (length > 0U)
    {
        usb_status_t status;
        uint32_t chunk = (length > DATA_BUFF_SIZE) ? DATA_BUFF_SIZE : length;

        while (s_txBusy != 0U)
        {
            vTaskDelay(pdMS_TO_TICKS(1U));
        }

        (void)memcpy(s_sendBuffer, data, chunk);
        s_txBusy = 1U;
        status   = USB_DeviceCdcAcmSend(s_cdcState.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, s_sendBuffer, chunk);
        if (status != kStatus_USB_Success)
        {
            s_txBusy = 0U;
            return;
        }

        data += chunk;
        length -= chunk;
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

static void ShellPrintHelp(void)
{
    ShellWrite("Comandi disponibili:\r\n");
    ShellWrite("  help            - mostra questo help\r\n");
    ShellWrite("  led blink       - lampeggio continuo (default)\r\n");
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
}

static void ShellHandleLine(char *line)
{
    char rawLine[SHELL_LINE_BUFFER_SIZE];
    char *argv[8];
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
            ShellWrite("LED in lampeggio continuo\r\n");
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

    if (strcmp(argv[0], "fs") == 0)
    {
        FsCommand(argc, argv, rawLine);
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
                (void)USB_DeviceCdcAcmSend(handle, USB_CDC_VCOM_BULK_IN_ENDPOINT, NULL, 0U);
            }
            else
            {
                s_txBusy = 0U;
            }
            error = kStatus_USB_Success;
            break;

        case kUSB_DeviceCdcEventRecvResponse:
            if ((s_cdcState.attach == 1U) && (s_cdcState.startTransactions == 1U))
            {
                s_recvSize = epCbParam->length;
                error      = kStatus_USB_Success;
                if (s_recvSize == 0U)
                {
                    error = USB_DeviceCdcAcmRecv(
                        handle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_recvBuffer, g_UsbDeviceCdcVcomDicEndpoints[1].maxPacketSize);
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
            if ((s_cdcAcmInfo.dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION) != 0U)
            {
                s_cdcAcmInfo.uartState |= USB_DEVICE_CDC_UART_STATE_TX_CARRIER;
            }
            else
            {
                s_cdcAcmInfo.uartState &= (uint16_t)(~USB_DEVICE_CDC_UART_STATE_TX_CARRIER);
            }
            if ((s_cdcAcmInfo.dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE) != 0U)
            {
                s_cdcAcmInfo.uartState |= USB_DEVICE_CDC_UART_STATE_RX_CARRIER;
            }
            else
            {
                s_cdcAcmInfo.uartState &= (uint16_t)(~USB_DEVICE_CDC_UART_STATE_RX_CARRIER);
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

            if ((s_cdcState.attach == 1U) && s_cdcAcmInfo.dtePresent)
            {
                s_cdcState.startTransactions = 1U;
                if (wasStarted == 0U)
                {
                    s_shellSessionReady = 0U;
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
            s_cdcState.attach               = 0U;
            s_cdcState.startTransactions    = 0U;
            s_cdcState.currentConfiguration = 0U;
            s_shellSessionReady             = 0U;
            s_recvSize                      = 0U;
            s_txBusy                        = 0U;
            error                           = kStatus_USB_Success;
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U))
            if (kStatus_USB_Success == USB_DeviceClassGetSpeed(CONTROLLER_ID, &s_cdcState.speed))
            {
                (void)USB_DeviceSetSpeed(handle, s_cdcState.speed);
            }
#endif
            break;

        case kUSB_DeviceEventSetConfiguration:
            if (0U == (*temp8))
            {
                s_cdcState.attach               = 0U;
                s_cdcState.startTransactions    = 0U;
                s_cdcState.currentConfiguration = 0U;
                s_shellSessionReady             = 0U;
                error                           = kStatus_USB_Success;
            }
            else if (USB_CDC_VCOM_CONFIGURE_INDEX == (*temp8))
            {
                s_cdcState.attach               = 1U;
                s_cdcState.currentConfiguration = *temp8;
                s_recvSize                      = 0U;
                error = USB_DeviceCdcAcmRecv(
                    s_cdcState.cdcAcmHandle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_recvBuffer,
                    g_UsbDeviceCdcVcomDicEndpoints[1].maxPacketSize);
            }
            break;

        case kUSB_DeviceEventSetInterface:
            if (0U != s_cdcState.attach)
            {
                uint8_t interface        = (uint8_t)((*temp16 & 0xFF00U) >> 8U);
                uint8_t alternateSetting = (uint8_t)(*temp16 & 0x00FFU);
                if ((interface == USB_CDC_VCOM_COMM_INTERFACE_INDEX) &&
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
                if (interface < USB_CDC_VCOM_INTERFACE_COUNT)
                {
                    *temp16 = (*temp16 & 0xFF00U) | s_cdcState.currentInterfaceAlternateSetting[interface];
                    error   = kStatus_USB_Success;
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
    USB_DeviceClockInit();
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
    SYSMPU_Enable(SYSMPU, 0U);
#endif

    s_cdcState.speed        = USB_SPEED_FULL;
    s_cdcState.attach       = 0U;
    s_cdcState.cdcAcmHandle = NULL;
    s_cdcState.deviceHandle = NULL;

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
static void AppTask(void *pvParameters)
{
    uint8_t packet[DATA_BUFF_SIZE];
    TickType_t lastBlinkTick = xTaskGetTickCount();
    led_mode_t appliedMode   = kLedModeBlink;

    (void)pvParameters;
    USB_DeviceApplicationInit();

    for (;;)
    {
        if ((s_cdcState.attach == 1U) && (s_cdcState.startTransactions == 1U))
        {
            if (s_shellSessionReady == 0U)
            {
                s_shellSessionReady = 1U;
                ShellWrite("\r\nFreeRTOS USB shell pronta. Digita 'help'.\r\n");
                ShellSendPrompt();
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

                ShellHandlePacket(packet, rxSize);

                (void)USB_DeviceCdcAcmRecv(
                    s_cdcState.cdcAcmHandle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_recvBuffer,
                    g_UsbDeviceCdcVcomDicEndpoints[1].maxPacketSize);
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(2U));
            }
        }
        else
        {
            s_shellSessionReady = 0U;
            vTaskDelay(pdMS_TO_TICKS(10U));
        }

        if (s_ledMode == kLedModeBlink)
        {
            TickType_t now = xTaskGetTickCount();
            if ((now - lastBlinkTick) >= pdMS_TO_TICKS(LED_BLINK_PERIOD_MS))
            {
                USER_LED_TOGGLE();
                lastBlinkTick = now;
            }
            appliedMode = kLedModeBlink;
        }
        else if ((s_ledMode == kLedModeOn) && (appliedMode != kLedModeOn))
        {
            USER_LED_ON();
            appliedMode = kLedModeOn;
        }
        else if ((s_ledMode == kLedModeOff) && (appliedMode != kLedModeOff))
        {
            USER_LED_OFF();
            appliedMode = kLedModeOff;
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
