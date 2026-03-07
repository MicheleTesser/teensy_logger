#include "app/app_shared.h"
#include "build_epoch.h"

#ifndef APP_BUILD_UNIX_EPOCH
#define APP_BUILD_UNIX_EPOCH 0UL
#endif

#define RTC_MIN_VALID_EPOCH 1577836800UL

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

static const CLI_Command_Definition_t s_cliLogCommand = {
    .pcCommand = "log",
    .pcHelpString = "log <status|on|off|usb <on|off>|autocan <on|off>|export ...>\r\n",
    .pxCommandInterpreter = ShellCliLogCommand,
    .cExpectedNumberOfParameters = -1,
};

static const CLI_Command_Definition_t s_cliSdCommand = {
    .pcCommand = "sd",
    .pcHelpString = "sd <status|refresh|ls [path] [max]>: stato spazio SD + listing\r\n",
    .pxCommandInterpreter = ShellCliSdCommand,
    .cExpectedNumberOfParameters = -1,
};

static const CLI_Command_Definition_t s_cliRtcCommand = {
    .pcCommand = "rtc",
    .pcHelpString = "rtc <status|set <unix_epoch>>\r\n",
    .pxCommandInterpreter = ShellCliRtcCommand,
    .cExpectedNumberOfParameters = -1,
};

static const CLI_Command_Definition_t s_cliTimeCommand = {
    .pcCommand = "time",
    .pcHelpString = "time [status|set [unix_epoch]|<unix_epoch>] (set senza arg: epoch build host)\r\n",
    .pxCommandInterpreter = ShellCliTimeCommand,
    .cExpectedNumberOfParameters = -1,
};

static const CLI_Command_Definition_t s_cliBootCommand = {
    .pcCommand = "bootloader",
    .pcHelpString = "bootloader <status|go|cancel>\r\n",
    .pxCommandInterpreter = ShellCliBootCommand,
    .cExpectedNumberOfParameters = -1,
};

/*
 * Module: USB IRQ glue + CDC shell I/O + CLI production commands + gs_usb frame queue helpers.
 */
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
void ShellWriteBytes(const uint8_t *data, uint32_t length)
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

void ShellWrite(const char *text)
{
    if (text == NULL)
    {
        return;
    }
    ShellWriteBytes((const uint8_t *)text, (uint32_t)strlen(text));
}

bool ShellWriteBytesTry(const uint8_t *data, uint32_t length)
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

bool ShellWriteTry(const char *text)
{
    if (text == NULL)
    {
        return false;
    }
    return ShellWriteBytesTry((const uint8_t *)text, (uint32_t)strlen(text));
}

void ShellSendPrompt(void)
{
    ShellWrite(SHELL_PROMPT);
}

void ShellWriteLinef(const char *fmt, ...)
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

bool ShellTokenEquals(const char *token, BaseType_t tokenLen, const char *value)
{
    size_t valueLen;

    if ((token == NULL) || (value == NULL) || (tokenLen < 0))
    {
        return false;
    }

    valueLen = strlen(value);
    return ((size_t)tokenLen == valueLen) && (strncmp(token, value, valueLen) == 0);
}

static bool ShellCopyParameter(const char *pcCommandString, UBaseType_t index, char *buffer, size_t bufferSize)
{
    const char *param;
    BaseType_t paramLen = 0;

    if ((pcCommandString == NULL) || (buffer == NULL) || (bufferSize < 2U))
    {
        return false;
    }

    param = FreeRTOS_CLIGetParameter(pcCommandString, index, &paramLen);
    if ((param == NULL) || (paramLen <= 0) || ((size_t)paramLen >= bufferSize))
    {
        return false;
    }

    (void)memcpy(buffer, param, (size_t)paramLen);
    buffer[paramLen] = '\0';
    return true;
}

static bool ShellPathHasMf4Extension(const char *path)
{
    const char *dot;
    const char *slash;

    if (path == NULL)
    {
        return false;
    }

    dot = strrchr(path, '.');
    if (dot == NULL)
    {
        return false;
    }

    slash = strrchr(path, '/');
    if ((slash != NULL) && (dot < slash))
    {
        return false;
    }

    if (dot[1] == '\0')
    {
        return false;
    }

    return (tolower((unsigned char)dot[1]) == 'm') && (tolower((unsigned char)dot[2]) == 'f') &&
           (tolower((unsigned char)dot[3]) == '4') && (dot[4] == '\0');
}

static FRESULT ShellEnsureSdMountedForCli(void)
{
    FRESULT fr;

    if (s_mscHostActive && (s_cdcState.attach == 1U))
    {
        return FR_LOCKED;
    }

    if (s_fsMounted)
    {
        return FR_OK;
    }

    fr = FsMount();
    if (fr == FR_OK)
    {
        return FR_OK;
    }

    /* Recovery path: if mount is busy/timed out, pause logger and retry once. */
    if (fr == FR_TIMEOUT)
    {
        bool resumeLogging = s_logEnabled;
        TickType_t startTick = xTaskGetTickCount();
        s_logEnabled = false;

        while (s_canLogFileOpen && ((xTaskGetTickCount() - startTick) < pdMS_TO_TICKS(600U)))
        {
            vTaskDelay(pdMS_TO_TICKS(20U));
        }

        (void)FsUnmount();
        vTaskDelay(pdMS_TO_TICKS(30U));
        fr = FsMount();
        s_logEnabled = resumeLogging;
    }

    return fr;
}

static bool ShellBuildDefaultCsvPath(const char *srcPath, char *dstPath, size_t dstPathSize)
{
    size_t baseLen;

    if ((srcPath == NULL) || (dstPath == NULL) || (dstPathSize < 6U))
    {
        return false;
    }

    baseLen = strlen(srcPath);
    if (ShellPathHasMf4Extension(srcPath))
    {
        const char *dot = strrchr(srcPath, '.');
        if (dot != NULL)
        {
            baseLen = (size_t)(dot - srcPath);
        }
    }

    if ((baseLen + 5U) > dstPathSize)
    {
        return false;
    }

    (void)memcpy(dstPath, srcPath, baseLen);
    (void)memcpy(&dstPath[baseLen], ".csv", 5U);
    return true;
}

static bool ShellPauseLoggingForExport(uint32_t timeoutMs)
{
    TickType_t startTick = xTaskGetTickCount();
    TickType_t timeoutTicks = pdMS_TO_TICKS(timeoutMs);
    uint32_t guard = 0U;
    const uint32_t guardMax = 1000U;

    s_logEnabled = false;
    while (s_canLogFileOpen)
    {
        if ((xTaskGetTickCount() - startTick) > timeoutTicks)
        {
            return false;
        }
        if (guard++ > guardMax)
        {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(20U));
    }
    return true;
}

static void ShellFormatBytesHuman(uint64_t bytes, char *out, size_t outSize)
{
    static const char *kUnits[] = {"B", "KiB", "MiB", "GiB", "TiB"};
    uint64_t scaled10;
    size_t unit = 0U;
    uint32_t whole;
    uint32_t frac;

    if ((out == NULL) || (outSize == 0U))
    {
        return;
    }

    scaled10 = bytes * 10ULL;
    while ((scaled10 >= 10240ULL) && (unit < (ARRAY_SIZE(kUnits) - 1U)))
    {
        scaled10 = (scaled10 + 512ULL) / 1024ULL;
        unit++;
    }

    whole = (uint32_t)(scaled10 / 10ULL);
    frac  = (uint32_t)(scaled10 % 10ULL);

    if (unit == 0U)
    {
        (void)snprintf(out, outSize, "%lu %s", (unsigned long)whole, kUnits[unit]);
    }
    else
    {
        (void)snprintf(out, outSize, "%lu.%lu %s", (unsigned long)whole, (unsigned long)frac, kUnits[unit]);
    }
}

typedef struct
{
    uint32_t scanned;
    uint32_t exported;
    uint32_t skippedExistingCsv;
    uint32_t skippedByAge;
    uint32_t failed;
    uint32_t recordsExported;
} shell_export_stats_t;

static bool ShellJoinPath(const char *dirPath, const char *fileName, char *fullPath, size_t fullPathSize)
{
    size_t dirLen;
    bool addSlash;
    int32_t n;

    if ((dirPath == NULL) || (fileName == NULL) || (fullPath == NULL) || (fullPathSize == 0U))
    {
        return false;
    }

    dirLen = strlen(dirPath);
    addSlash = (dirLen > 0U) && (dirPath[dirLen - 1U] != '/');

    if ((dirLen == 0U) || ((dirLen == 1U) && (dirPath[0] == '.')))
    {
        n = snprintf(fullPath, fullPathSize, "%s", fileName);
    }
    else if ((dirLen == 1U) && (dirPath[0] == '/'))
    {
        n = snprintf(fullPath, fullPathSize, "/%s", fileName);
    }
    else if (addSlash)
    {
        n = snprintf(fullPath, fullPathSize, "%s/%s", dirPath, fileName);
    }
    else
    {
        n = snprintf(fullPath, fullPathSize, "%s%s", dirPath, fileName);
    }

    return (n > 0) && ((size_t)n < fullPathSize);
}

static bool ShellIsLeapYear(uint32_t year)
{
    return ((year % 4U) == 0U) && (((year % 100U) != 0U) || ((year % 400U) == 0U));
}

static bool ShellFatDateTimeToUnix(uint16_t fatDate, uint16_t fatTime, uint32_t *unixSeconds)
{
    static const uint8_t kDaysInMonth[12] = {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};
    uint32_t year;
    uint32_t month;
    uint32_t day;
    uint32_t hour;
    uint32_t minute;
    uint32_t second;
    uint64_t days = 0U;
    uint32_t y;
    uint32_t m;

    if (unixSeconds == NULL)
    {
        return false;
    }

    year   = 1980U + ((uint32_t)((fatDate >> 9U) & 0x7FU));
    month  = (uint32_t)((fatDate >> 5U) & 0x0FU);
    day    = (uint32_t)(fatDate & 0x1FU);
    hour   = (uint32_t)((fatTime >> 11U) & 0x1FU);
    minute = (uint32_t)((fatTime >> 5U) & 0x3FU);
    second = (uint32_t)(fatTime & 0x1FU) * 2U;

    if ((month < 1U) || (month > 12U) || (day < 1U) || (hour > 23U) || (minute > 59U) || (second > 59U))
    {
        return false;
    }

    if (day > kDaysInMonth[month - 1U] + ((month == 2U && ShellIsLeapYear(year)) ? 1U : 0U))
    {
        return false;
    }

    for (y = 1970U; y < year; y++)
    {
        days += ShellIsLeapYear(y) ? 366ULL : 365ULL;
    }
    for (m = 1U; m < month; m++)
    {
        days += kDaysInMonth[m - 1U];
        if ((m == 2U) && ShellIsLeapYear(year))
        {
            days++;
        }
    }
    days += (uint64_t)(day - 1U);

    *unixSeconds = (uint32_t)(days * 86400ULL + ((uint64_t)hour * 3600ULL) + ((uint64_t)minute * 60ULL) + second);
    return true;
}

static FRESULT ShellExportIncremental(const char *dirPath, bool filterRecent, uint32_t recentMinutes,
                                      shell_export_stats_t *stats)
{
    DIR dir;
    FILINFO info;
    FRESULT fr;
    FRESULT firstError = FR_OK;
    const char *effectiveDirPath = dirPath;
    uint32_t nowSeconds = 0U;
    uint32_t ageLimitSeconds = 0U;

    if ((dirPath == NULL) || (stats == NULL))
    {
        return FR_INVALID_PARAMETER;
    }

    (void)memset(stats, 0, sizeof(*stats));

    if (filterRecent)
    {
        if (!RtcIsValid())
        {
            return FR_INVALID_PARAMETER;
        }
        nowSeconds = RtcGetUnixSeconds();
        ageLimitSeconds = recentMinutes * 60U;
    }

    fr = f_opendir(&dir, dirPath);
    if ((fr != FR_OK) && (strcmp(dirPath, ".") == 0))
    {
        fr = f_opendir(&dir, "/");
        if (fr == FR_OK)
        {
            effectiveDirPath = "/";
        }
    }
    if (fr != FR_OK)
    {
        return fr;
    }

    while (true)
    {
        const char *name;
        char srcPath[CAN_LOG_EXPORT_PATH_MAX];
        char dstPath[CAN_LOG_EXPORT_PATH_MAX];
        uint32_t exportedRecords = 0U;
        FILINFO csvInfo;
        uint32_t fileSeconds = 0U;

        fr = f_readdir(&dir, &info);
        if (fr != FR_OK)
        {
            firstError = (firstError == FR_OK) ? fr : firstError;
            break;
        }
        if (info.fname[0] == '\0')
        {
            break;
        }
        if ((info.fattrib & AM_DIR) != 0U)
        {
            continue;
        }

        name = info.fname;
        if (!ShellPathHasMf4Extension(name))
        {
            continue;
        }

        stats->scanned++;

        if (filterRecent)
        {
            if (!ShellFatDateTimeToUnix(info.fdate, info.ftime, &fileSeconds))
            {
                stats->skippedByAge++;
                continue;
            }
            if ((nowSeconds > fileSeconds) && ((nowSeconds - fileSeconds) > ageLimitSeconds))
            {
                stats->skippedByAge++;
                continue;
            }
        }

        if (!ShellJoinPath(effectiveDirPath, name, srcPath, sizeof(srcPath)))
        {
            stats->failed++;
            firstError = (firstError == FR_OK) ? FR_INVALID_NAME : firstError;
            continue;
        }
        if (!ShellBuildDefaultCsvPath(srcPath, dstPath, sizeof(dstPath)))
        {
            stats->failed++;
            firstError = (firstError == FR_OK) ? FR_INVALID_NAME : firstError;
            continue;
        }

        fr = f_stat(dstPath, &csvInfo);
        if (fr == FR_OK)
        {
            stats->skippedExistingCsv++;
            continue;
        }
        if ((fr != FR_NO_FILE) && (fr != FR_NO_PATH))
        {
            stats->failed++;
            firstError = (firstError == FR_OK) ? fr : firstError;
            continue;
        }

        fr = CanLogExportCsv(srcPath, dstPath, &exportedRecords);
        if (fr != FR_OK)
        {
            stats->failed++;
            firstError = (firstError == FR_OK) ? fr : firstError;
            continue;
        }

        stats->exported++;
        stats->recordsExported += exportedRecords;
    }

    (void)f_closedir(&dir);
    return firstError;
}

void ShellCliInit(void)
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
    if (FreeRTOS_CLIRegisterCommand(&s_cliLogCommand) != pdTRUE)
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
    if (FreeRTOS_CLIRegisterCommand(&s_cliTimeCommand) != pdTRUE)
    {
        return;
    }
    if (FreeRTOS_CLIRegisterCommand(&s_cliBootCommand) != pdTRUE)
    {
        return;
    }

    s_shellCliReady = true;
}

BaseType_t ShellCliHelpCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                   "Production CLI:\r\n"
                   "  help                 - mostra questo help\r\n"
                   "  led <status|activity|on|off>\r\n"
                   "  can <status|util|clear>\r\n"
                   "  log <status|on|off|usb <on|off>|autocan <on|off>|export ...>\r\n"
                   "    log export <src|active> [dst.csv]\r\n"
                   "    log export inc [dir]             - solo file .mf4 senza .csv\r\n"
                   "    log export recent [min] [dir]    - default 60 min\r\n"
                   "  sd <status|refresh|ls [path] [max]>  - spazio SD + list file\r\n"
                   "  rtc status           - stato RTC SNVS (UTC)\r\n"
                   "  rtc set <epoch>      - imposta RTC con Unix epoch\r\n"
                   "  time [epoch]          - alias rapido per leggere/impostare RTC\r\n"
                   "  time set              - usa epoch host al momento della build\r\n"
                   "  bootloader <status|go|cancel> - entra in bootloader per flash\r\n"
                   "Nota: CAN init/send solo via SocketCAN.\r\n");
    return pdFALSE;
}

BaseType_t ShellCliLedCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
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

BaseType_t ShellCliCanCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
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

BaseType_t ShellCliLogCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    const char *subcmd;
    BaseType_t subcmdLen = 0;

    subcmd = FreeRTOS_CLIGetParameter(pcCommandString, 1U, &subcmdLen);
    if ((subcmd == NULL) || ShellTokenEquals(subcmd, subcmdLen, "status"))
    {
        bool usbAttached = (s_cdcState.attach == 1U);
        bool active = s_logEnabled && (s_logAllowWhenUsbAttached || !usbAttached);
        (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                       "log enabled=%u active=%u usb_attached=%u allow_usb=%u autocan=%u file=%s\r\n",
                       s_logEnabled ? 1U : 0U, active ? 1U : 0U, usbAttached ? 1U : 0U,
                       s_logAllowWhenUsbAttached ? 1U : 0U, s_canAutoStartEnabled ? 1U : 0U,
                       (s_canLogFileOpen && (s_canLogActivePath[0] != '\0')) ? s_canLogActivePath : "(n/a)");
        return pdFALSE;
    }

    if (ShellTokenEquals(subcmd, subcmdLen, "on"))
    {
        s_logEnabled = true;
        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Logging ON\r\n");
        return pdFALSE;
    }

    if (ShellTokenEquals(subcmd, subcmdLen, "off"))
    {
        s_logEnabled = false;
        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Logging OFF\r\n");
        return pdFALSE;
    }

    if (ShellTokenEquals(subcmd, subcmdLen, "usb"))
    {
        const char *arg;
        BaseType_t argLen = 0;
        arg = FreeRTOS_CLIGetParameter(pcCommandString, 2U, &argLen);
        if ((arg == NULL) || (argLen <= 0))
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: log usb <on|off>\r\n");
            return pdFALSE;
        }
        if (ShellTokenEquals(arg, argLen, "on"))
        {
            s_logAllowWhenUsbAttached = true;
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Logging con USB collegato: ON\r\n");
            return pdFALSE;
        }
        if (ShellTokenEquals(arg, argLen, "off"))
        {
            s_logAllowWhenUsbAttached = false;
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Logging con USB collegato: OFF\r\n");
            return pdFALSE;
        }
        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: log usb <on|off>\r\n");
        return pdFALSE;
    }

    if (ShellTokenEquals(subcmd, subcmdLen, "autocan"))
    {
        const char *arg;
        BaseType_t argLen = 0;
        arg = FreeRTOS_CLIGetParameter(pcCommandString, 2U, &argLen);
        if ((arg == NULL) || (argLen <= 0))
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: log autocan <on|off>\r\n");
            return pdFALSE;
        }
        if (ShellTokenEquals(arg, argLen, "on"))
        {
            s_canAutoStartEnabled = true;
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Auto CAN start: ON\r\n");
            return pdFALSE;
        }
        if (ShellTokenEquals(arg, argLen, "off"))
        {
            s_canAutoStartEnabled = false;
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Auto CAN start: OFF\r\n");
            return pdFALSE;
        }
        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: log autocan <on|off>\r\n");
        return pdFALSE;
    }

    if (ShellTokenEquals(subcmd, subcmdLen, "export"))
    {
        char srcPath[CAN_LOG_EXPORT_PATH_MAX];
        char dstPath[CAN_LOG_EXPORT_PATH_MAX];
        char argPath[CAN_LOG_EXPORT_PATH_MAX];
        char dirPath[CAN_LOG_EXPORT_PATH_MAX];
        shell_export_stats_t stats;
        bool resumeLogging = s_logEnabled;
        uint32_t recentMinutes = 60U;
        uint32_t exported = 0U;
        FRESULT fr;

        if (!ShellCopyParameter(pcCommandString, 2U, argPath, sizeof(argPath)))
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                           "Uso: log export <src|active> [dst.csv]\r\n"
                           "     log export inc [dir]\r\n"
                           "     log export recent [min] [dir]\r\n");
            return pdFALSE;
        }

        if (strcmp(argPath, "inc") == 0)
        {
            if (!ShellCopyParameter(pcCommandString, 3U, dirPath, sizeof(dirPath)))
            {
                (void)strncpy(dirPath, ".", sizeof(dirPath) - 1U);
                dirPath[sizeof(dirPath) - 1U] = '\0';
            }

            if (!ShellPauseLoggingForExport(4000U))
            {
                s_logEnabled = resumeLogging;
                (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Timeout chiusura log attivo\r\n");
                return pdFALSE;
            }

            fr = ShellExportIncremental(dirPath, false, 0U, &stats);
            s_logEnabled = resumeLogging;

            if ((fr != FR_OK) && (stats.exported == 0U))
            {
                (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Export incrementale fallito: %s (%d)\r\n",
                               FsResultToString(fr), (int)fr);
                return pdFALSE;
            }

            (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                           "Export inc dir=%s scanned=%lu exported=%lu skipped_csv=%lu failed=%lu records=%lu\r\n",
                           dirPath, (unsigned long)stats.scanned, (unsigned long)stats.exported,
                           (unsigned long)stats.skippedExistingCsv, (unsigned long)stats.failed,
                           (unsigned long)stats.recordsExported);
            return pdFALSE;
        }

        if (strcmp(argPath, "recent") == 0)
        {
            char token[24];

            if (ShellCopyParameter(pcCommandString, 3U, token, sizeof(token)))
            {
                if (ShellParseU32Auto(token, &recentMinutes))
                {
                    if (!ShellCopyParameter(pcCommandString, 4U, dirPath, sizeof(dirPath)))
                    {
                        (void)strncpy(dirPath, ".", sizeof(dirPath) - 1U);
                        dirPath[sizeof(dirPath) - 1U] = '\0';
                    }
                }
                else
                {
                    (void)strncpy(dirPath, token, sizeof(dirPath) - 1U);
                    dirPath[sizeof(dirPath) - 1U] = '\0';
                }
            }
            else
            {
                (void)strncpy(dirPath, ".", sizeof(dirPath) - 1U);
                dirPath[sizeof(dirPath) - 1U] = '\0';
            }

            if ((recentMinutes == 0U) || (recentMinutes > (24U * 60U * 30U)))
            {
                (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Minuti non validi (1..43200)\r\n");
                return pdFALSE;
            }

            if (!RtcIsValid())
            {
                (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                               "RTC non valido: usa 'time set <epoch>' prima di 'log export recent'\r\n");
                return pdFALSE;
            }

            if (!ShellPauseLoggingForExport(4000U))
            {
                s_logEnabled = resumeLogging;
                (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Timeout chiusura log attivo\r\n");
                return pdFALSE;
            }

            fr = ShellExportIncremental(dirPath, true, recentMinutes, &stats);
            s_logEnabled = resumeLogging;

            if ((fr != FR_OK) && (stats.exported == 0U))
            {
                (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Export recent fallito: %s (%d)\r\n",
                               FsResultToString(fr), (int)fr);
                return pdFALSE;
            }

            (void)snprintf(
                pcWriteBuffer, xWriteBufferLen,
                "Export recent=%lu min dir=%s scanned=%lu exported=%lu skipped_csv=%lu skipped_age=%lu failed=%lu records=%lu\r\n",
                (unsigned long)recentMinutes, dirPath, (unsigned long)stats.scanned, (unsigned long)stats.exported,
                (unsigned long)stats.skippedExistingCsv, (unsigned long)stats.skippedByAge,
                (unsigned long)stats.failed, (unsigned long)stats.recordsExported);
            return pdFALSE;
        }

        if (strcmp(argPath, "active") == 0)
        {
            if (s_canLogActivePath[0] == '\0')
            {
                (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Nessun log attivo. Specifica un file sorgente.\r\n");
                return pdFALSE;
            }
            (void)strncpy(srcPath, s_canLogActivePath, sizeof(srcPath) - 1U);
            srcPath[sizeof(srcPath) - 1U] = '\0';
        }
        else
        {
            (void)strncpy(srcPath, argPath, sizeof(srcPath) - 1U);
            srcPath[sizeof(srcPath) - 1U] = '\0';
        }

        if (!ShellCopyParameter(pcCommandString, 3U, dstPath, sizeof(dstPath)))
        {
            if (!ShellBuildDefaultCsvPath(srcPath, dstPath, sizeof(dstPath)))
            {
                (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Percorso destinazione non valido\r\n");
                return pdFALSE;
            }
        }

        if (!ShellPauseLoggingForExport(4000U))
        {
            s_logEnabled = resumeLogging;
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Timeout chiusura log attivo\r\n");
            return pdFALSE;
        }

        fr = CanLogExportCsv(srcPath, dstPath, &exported);
        s_logEnabled = resumeLogging;

        if (fr != FR_OK)
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Export fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
            return pdFALSE;
        }

        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Export completato: %lu record -> %s\r\n",
                       (unsigned long)exported, dstPath);
        return pdFALSE;
    }

    (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                   "Uso: log <status|on|off|usb <on|off>|autocan <on|off>|export ...>\r\n");
    return pdFALSE;
}

BaseType_t ShellCliSdCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    const char *subcmd;
    const char *param;
    BaseType_t subcmdLen = 0;
    BaseType_t paramLen = 0;
    bool forceRefresh = false;
    sd_usage_cache_t usage;
    TickType_t now;
    uint32_t ageMs = 0U;
    FRESULT fr;
    bool hasSnapshot;
    char path[CAN_LOG_EXPORT_PATH_MAX];
    DIR dir;
    FILINFO info;
    uint32_t totalMiB;
    uint32_t freeMiB;
    uint32_t usedMiB;
    char totalHuman[24];
    char freeHuman[24];
    char usedHuman[24];

    subcmd = FreeRTOS_CLIGetParameter(pcCommandString, 1U, &subcmdLen);
    if (subcmd != NULL)
    {
        if (ShellTokenEquals(subcmd, subcmdLen, "refresh"))
        {
            forceRefresh = true;
        }
        else if (ShellTokenEquals(subcmd, subcmdLen, "ls"))
        {
            char token[24];
            uint32_t maxLines = 32U;
            uint32_t shown = 0U;
            bool truncated = false;

            fr = ShellEnsureSdMountedForCli();
            if (fr != FR_OK)
            {
                if (fr == FR_LOCKED)
                {
                    (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                                   "SD bloccata: esportata come disco USB (MSC). Usa il PC per leggere i file.\r\n");
                }
                else
                {
                    (void)snprintf(pcWriteBuffer, xWriteBufferLen, "SD non montata: %s (%d)\r\n",
                                   FsResultToString(fr), (int)fr);
                }
                return pdFALSE;
            }

            param = FreeRTOS_CLIGetParameter(pcCommandString, 2U, &paramLen);
            if ((param != NULL) && (paramLen > 0) && ((size_t)paramLen < sizeof(path)))
            {
                (void)memcpy(path, param, (size_t)paramLen);
                path[paramLen] = '\0';

                if (ShellParseU32Auto(path, &maxLines))
                {
                    (void)strncpy(path, "/", sizeof(path) - 1U);
                    path[sizeof(path) - 1U] = '\0';
                }
            }
            else
            {
                (void)strncpy(path, "/", sizeof(path) - 1U);
                path[sizeof(path) - 1U] = '\0';
            }

            param = FreeRTOS_CLIGetParameter(pcCommandString, 3U, &paramLen);
            if ((param != NULL) && (paramLen > 0) && ((size_t)paramLen < sizeof(token)))
            {
                (void)memcpy(token, param, (size_t)paramLen);
                token[paramLen] = '\0';
                if (!ShellParseU32Auto(token, &maxLines))
                {
                    (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: sd ls [path] [max]\r\n");
                    return pdFALSE;
                }
            }

            if (maxLines == 0U)
            {
                maxLines = 1U;
            }
            if (maxLines > 512U)
            {
                maxLines = 512U;
            }

            fr = f_opendir(&dir, path);
            if (fr != FR_OK)
            {
                (void)snprintf(pcWriteBuffer, xWriteBufferLen, "ls fallito: %s (%d)\r\n",
                               FsResultToString(fr), (int)fr);
                return pdFALSE;
            }

            ShellWriteLinef("Listing %s\r\n", path);
            while (true)
            {
                const char *name;

                fr = f_readdir(&dir, &info);
                if (fr != FR_OK)
                {
                    (void)f_closedir(&dir);
                    (void)snprintf(pcWriteBuffer, xWriteBufferLen, "readdir fallito: %s (%d)\r\n",
                                   FsResultToString(fr), (int)fr);
                    return pdFALSE;
                }
                if (info.fname[0] == '\0')
                {
                    break;
                }

                name = info.fname;
                if (shown >= maxLines)
                {
                    truncated = true;
                    break;
                }
                if ((info.fattrib & AM_DIR) != 0U)
                {
                    ShellWriteLinef("d %10s %s\r\n", "-", name);
                }
                else
                {
                    char sizeHuman[24];
                    ShellFormatBytesHuman((uint64_t)info.fsize, sizeHuman, sizeof(sizeHuman));
                    ShellWriteLinef("f %10s %s\r\n", sizeHuman, name);
                }
                shown++;
            }
            (void)f_closedir(&dir);

            if (truncated)
            {
                (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                               "ls completato: mostrati %lu file (limite=%lu, output troncato)\r\n",
                               (unsigned long)shown, (unsigned long)maxLines);
            }
            else
            {
                (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                               "ls completato: mostrati %lu file\r\n",
                               (unsigned long)shown);
            }
            return pdFALSE;
        }
        else if (!ShellTokenEquals(subcmd, subcmdLen, "status"))
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: sd <status|refresh|ls [path] [max]>\r\n");
            return pdFALSE;
        }
    }

    fr = ShellEnsureSdMountedForCli();
    if (fr != FR_OK)
    {
        if (fr == FR_LOCKED)
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                           "SD bloccata: esportata come disco USB (MSC). Usa il PC per leggere i file.\r\n");
        }
        else
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "SD non montata: %s (%d)\r\n", FsResultToString(fr), (int)fr);
        }
        return pdFALSE;
    }
    if (!s_sdUsageCache.valid)
    {
        forceRefresh = true;
    }

    hasSnapshot = SdUsageSnapshot(&usage);
    if (forceRefresh || !hasSnapshot)
    {
        fr = SdUsageRefresh(true);
    }
    else
    {
        fr = FR_OK;
    }

    if ((fr != FR_OK) && !hasSnapshot)
    {
        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "sd status fallito: %s (%d)\r\n", FsResultToString(fr), (int)fr);
        return pdFALSE;
    }

    if (fr == FR_OK)
    {
        hasSnapshot = SdUsageSnapshot(&usage);
    }
    if (!hasSnapshot)
    {
        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "SD cache non disponibile\r\n");
        return pdFALSE;
    }

    now = xTaskGetTickCount();
    ageMs = (uint32_t)((now - usage.tick) * portTICK_PERIOD_MS);
    totalMiB = (uint32_t)(usage.totalBytes / (1024ULL * 1024ULL));
    freeMiB  = (uint32_t)(usage.freeBytes / (1024ULL * 1024ULL));
    usedMiB  = (uint32_t)(usage.usedBytes / (1024ULL * 1024ULL));
    ShellFormatBytesHuman(usage.totalBytes, totalHuman, sizeof(totalHuman));
    ShellFormatBytesHuman(usage.freeBytes, freeHuman, sizeof(freeHuman));
    ShellFormatBytesHuman(usage.usedBytes, usedHuman, sizeof(usedHuman));

    (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                   "SD totale=%s libero=%s usato=%s (%u.%u%%) cache_age=%lu ms\r\n"
                   "SD (raw) totale=%lu MiB libero=%lu MiB usato=%lu MiB\r\n"
                   "log=%s\r\n",
                   totalHuman, freeHuman, usedHuman,
                   (unsigned int)(usage.usedPermille / 10U), (unsigned int)(usage.usedPermille % 10U),
                   (unsigned long)ageMs, (unsigned long)totalMiB, (unsigned long)freeMiB, (unsigned long)usedMiB,
                   (s_canLogFileOpen && (s_canLogActivePath[0] != '\0')) ? s_canLogActivePath : "(n/a)");
    return pdFALSE;
}

BaseType_t ShellCliRtcCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    const char *subcmd;
    const char *param = NULL;
    BaseType_t subcmdLen = 0;
    BaseType_t paramLen = 0;
    char token[24];
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

    /* Shorthand: rtc <unix_epoch> */
    if ((subcmdLen > 0) && ((size_t)subcmdLen < sizeof(token)))
    {
        (void)memcpy(token, subcmd, (size_t)subcmdLen);
        token[subcmdLen] = '\0';
        if (ShellParseU32Auto(token, &unixSeconds))
        {
            if (!RtcSetUnixSeconds(unixSeconds))
            {
                (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Errore impostazione RTC\r\n");
                return pdFALSE;
            }

            (void)RtcUnixSecondsToDateTime(unixSeconds, &dt);
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "RTC impostato: %lu (%04u-%02u-%02u %02u:%02u:%02u UTC)\r\n",
                           (unsigned long)unixSeconds, (unsigned int)dt.year, (unsigned int)dt.month,
                           (unsigned int)dt.day, (unsigned int)dt.hour, (unsigned int)dt.minute,
                           (unsigned int)dt.second);
            return pdFALSE;
        }
    }

    (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: rtc <status|set <unix_epoch>>\r\n");
    return pdFALSE;
}

BaseType_t ShellCliTimeCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    const char *subcmd;
    BaseType_t subcmdLen = 0;
    uint32_t unixSeconds;
    rtc_datetime_t dt;
    char token[24];

    subcmd = FreeRTOS_CLIGetParameter(pcCommandString, 1U, &subcmdLen);
    if ((subcmd == NULL) || ShellTokenEquals(subcmd, subcmdLen, "status"))
    {
        unixSeconds = RtcGetUnixSeconds();
        (void)RtcUnixSecondsToDateTime(unixSeconds, &dt);
        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "epoch=%lu UTC=%04u-%02u-%02u %02u:%02u:%02u valid=%u\r\n",
                       (unsigned long)unixSeconds, (unsigned int)dt.year, (unsigned int)dt.month,
                       (unsigned int)dt.day, (unsigned int)dt.hour, (unsigned int)dt.minute, (unsigned int)dt.second,
                       RtcIsValid() ? 1U : 0U);
        return pdFALSE;
    }

    if (ShellTokenEquals(subcmd, subcmdLen, "set"))
    {
        const char *param = NULL;
        BaseType_t paramLen = 0;

        param = FreeRTOS_CLIGetParameter(pcCommandString, 2U, &paramLen);
        if ((param == NULL) || (paramLen <= 0))
        {
            unixSeconds = (uint32_t)APP_BUILD_UNIX_EPOCH;
            if (unixSeconds < RTC_MIN_VALID_EPOCH)
            {
                (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                               "Epoch build non valido. Usa: time set <unix_epoch>\r\n");
                return pdFALSE;
            }
            goto apply_time;
        }

        if ((size_t)paramLen >= sizeof(token))
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: time [<unix_epoch>|status|set [unix_epoch]]\r\n");
            return pdFALSE;
        }

        (void)memcpy(token, param, (size_t)paramLen);
        token[paramLen] = '\0';
    }
    else
    {
        if ((subcmdLen <= 0) || ((size_t)subcmdLen >= sizeof(token)))
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: time [<unix_epoch>|status|set [unix_epoch]]\r\n");
            return pdFALSE;
        }
        (void)memcpy(token, subcmd, (size_t)subcmdLen);
        token[subcmdLen] = '\0';
    }

    if (!ShellParseU32Auto(token, &unixSeconds))
    {
        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Epoch non valido\r\n");
        return pdFALSE;
    }

apply_time:
    if (!RtcSetUnixSeconds(unixSeconds))
    {
        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Errore impostazione RTC\r\n");
        return pdFALSE;
    }

    (void)RtcUnixSecondsToDateTime(unixSeconds, &dt);
    (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                   "RTC impostato: %lu (%04u-%02u-%02u %02u:%02u:%02u UTC)\r\n",
                   (unsigned long)unixSeconds, (unsigned int)dt.year, (unsigned int)dt.month, (unsigned int)dt.day,
                   (unsigned int)dt.hour, (unsigned int)dt.minute, (unsigned int)dt.second);
    return pdFALSE;
}

BaseType_t ShellCliBootCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    const char *subcmd;
    BaseType_t subcmdLen = 0;

    subcmd = FreeRTOS_CLIGetParameter(pcCommandString, 1U, &subcmdLen);
    if ((subcmd == NULL) || ShellTokenEquals(subcmd, subcmdLen, "status"))
    {
        if (s_bootloaderRebootPending)
        {
            TickType_t now = xTaskGetTickCount();
            uint32_t remainingMs = 0U;

            if (s_bootloaderRebootDeadline > now)
            {
                remainingMs = (uint32_t)((s_bootloaderRebootDeadline - now) * portTICK_PERIOD_MS);
            }

            (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                           "bootloader pending=1 eta=%lu ms\r\n",
                           (unsigned long)remainingMs);
        }
        else
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "bootloader pending=0\r\n");
        }
        return pdFALSE;
    }

    if (ShellTokenEquals(subcmd, subcmdLen, "go") || ShellTokenEquals(subcmd, subcmdLen, "now"))
    {
        AppRequestBootloaderReboot(BOOTLOADER_REBOOT_DELAY_CLI_MS);
        (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                       "Riavvio in bootloader richiesto (%lu ms)\r\n",
                       (unsigned long)BOOTLOADER_REBOOT_DELAY_CLI_MS);
        return pdFALSE;
    }

    if (ShellTokenEquals(subcmd, subcmdLen, "cancel"))
    {
        AppCancelBootloaderReboot();
        (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Richiesta bootloader annullata\r\n");
        return pdFALSE;
    }

    (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: bootloader <status|go|cancel>\r\n");
    return pdFALSE;
}

void LedMarkCanActivity(void)
{
    s_ledActivityLastTick = xTaskGetTickCount();
    s_ledActivitySeen = true;
}

uint32_t GsBswap32(uint32_t value)
{
    return ((value & 0x000000FFUL) << 24U) | ((value & 0x0000FF00UL) << 8U) | ((value & 0x00FF0000UL) >> 8U) |
           ((value & 0xFF000000UL) >> 24U);
}

bool GsIsLittleEndianHost(void)
{
    const uint16_t marker = 0x0001U;
    return (*(const uint8_t *)&marker) == 0x01U;
}

uint32_t GsToLe32(uint32_t value)
{
    if (GsIsLittleEndianHost())
    {
        return value;
    }
    return GsBswap32(value);
}

uint32_t GsFromLe32(uint32_t value)
{
    if (GsIsLittleEndianHost())
    {
        return value;
    }
    return GsBswap32(value);
}

uint32_t GsWireToCpu32(uint32_t value)
{
    if (s_gsCanHostLe)
    {
        return GsFromLe32(value);
    }
    return value;
}

uint32_t GsCpuToWire32(uint32_t value)
{
    if (s_gsCanHostLe)
    {
        return GsToLe32(value);
    }
    return value;
}

bool GsCanQueueFrame(const gs_host_frame_classic_t *frame)
{
    if (APP_UNLIKELY(frame == NULL))
    {
        return false;
    }

    taskENTER_CRITICAL();
    if (APP_UNLIKELY(s_gsCanTxCount >= GS_USB_FRAME_QUEUE_LENGTH))
    {
        taskEXIT_CRITICAL();
        return false;
    }

    s_gsCanTxQueue[s_gsCanTxHead] = *frame;
    s_gsCanTxHead++;
    if (s_gsCanTxHead >= GS_USB_FRAME_QUEUE_LENGTH)
    {
        s_gsCanTxHead = 0U;
    }
    s_gsCanTxCount++;
    taskEXIT_CRITICAL();

    return true;
}

void GsCanTrySendNext(void)
{
    gs_host_frame_classic_t nextFrame;
    usb_status_t status;

    if (APP_UNLIKELY(!s_gsCanConfigured || !s_gsCanEpReady || s_gsCanInBusy))
    {
        return;
    }

    if (APP_UNLIKELY((s_cdcState.attach == 0U) || (s_cdcState.deviceHandle == NULL)))
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
    s_gsCanTxTail++;
    if (s_gsCanTxTail >= GS_USB_FRAME_QUEUE_LENGTH)
    {
        s_gsCanTxTail = 0U;
    }
    s_gsCanTxCount--;
    taskEXIT_CRITICAL();

    s_gsCanInFrames[0] = nextFrame;

    s_gsCanInBusy  = true;
    status = USB_DeviceSendRequest(s_cdcState.deviceHandle, (uint8_t)(USB_GS_CAN_BULK_IN_ENDPOINT | (USB_IN << 7U)),
                                   (uint8_t *)&s_gsCanInFrames[0], sizeof(gs_host_frame_classic_t));
    if (status != kStatus_USB_Success)
    {
        s_gsCanInBusy = false;
        taskENTER_CRITICAL();
        if (s_gsCanTxCount < GS_USB_FRAME_QUEUE_LENGTH)
        {
            if (s_gsCanTxTail == 0U)
            {
                s_gsCanTxTail = (uint16_t)(GS_USB_FRAME_QUEUE_LENGTH - 1U);
            }
            else
            {
                s_gsCanTxTail--;
            }
            s_gsCanTxQueue[s_gsCanTxTail] = nextFrame;
            s_gsCanTxCount++;
        }
        taskEXIT_CRITICAL();
    }
}

void GsCanSetOverflowFlag(size_t busIndex)
{
    if (busIndex < GS_USB_CHANNEL_COUNT)
    {
        s_gsCanOverflowPending[busIndex] = true;
    }
}

bool GsCanRxPublish(size_t busIndex, const flexcan_frame_t *frame, status_t status)
{
    gs_host_frame_classic_t hostFrame;
    uint32_t canId;
    uint8_t dlc;

    if (APP_UNLIKELY((frame == NULL) || (busIndex >= GS_USB_CHANNEL_COUNT) || !s_gsCanConfigured ||
                     !s_canBuses[busIndex].started))
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

    (void)memset(&hostFrame, 0, sizeof(hostFrame));
    hostFrame.echoId  = GsCpuToWire32(GS_HOST_FRAME_ECHO_ID_RX);
    hostFrame.canId   = GsCpuToWire32(canId);
    hostFrame.canDlc  = dlc;
    hostFrame.channel = (uint8_t)busIndex;
    hostFrame.flags   = 0U;

    if (APP_UNLIKELY((status == kStatus_FLEXCAN_RxOverflow) || s_gsCanOverflowPending[busIndex]))
    {
        hostFrame.flags |= GS_CAN_FLAG_OVERFLOW;
        s_gsCanOverflowPending[busIndex] = false;
    }

    hostFrame.data[0] = frame->dataByte0;
    hostFrame.data[1] = frame->dataByte1;
    hostFrame.data[2] = frame->dataByte2;
    hostFrame.data[3] = frame->dataByte3;
    hostFrame.data[4] = frame->dataByte4;
    hostFrame.data[5] = frame->dataByte5;
    hostFrame.data[6] = frame->dataByte6;
    hostFrame.data[7] = frame->dataByte7;

    if (APP_UNLIKELY(!GsCanQueueFrame(&hostFrame)))
    {
        GsCanSetOverflowFlag(busIndex);
        return false;
    }
    return true;
}

bool GsCanTxEchoPublish(size_t busIndex, uint32_t echoId, const flexcan_frame_t *frame)
{
    gs_host_frame_classic_t hostFrame;
    uint32_t canId;
    uint8_t dlc;

    if (APP_UNLIKELY((frame == NULL) || (busIndex >= GS_USB_CHANNEL_COUNT) || !s_gsCanConfigured))
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

    (void)memset(&hostFrame, 0, sizeof(hostFrame));
    hostFrame.echoId  = GsCpuToWire32(echoId);
    hostFrame.canId   = GsCpuToWire32(canId);
    hostFrame.canDlc  = dlc;
    hostFrame.channel = (uint8_t)busIndex;
    hostFrame.data[0] = frame->dataByte0;
    hostFrame.data[1] = frame->dataByte1;
    hostFrame.data[2] = frame->dataByte2;
    hostFrame.data[3] = frame->dataByte3;
    hostFrame.data[4] = frame->dataByte4;
    hostFrame.data[5] = frame->dataByte5;
    hostFrame.data[6] = frame->dataByte6;
    hostFrame.data[7] = frame->dataByte7;

    return GsCanQueueFrame(&hostFrame);
}

bool GsCanFillBtConst(uint8_t channel, gs_device_bt_const_t *btConst)
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

    /* Keep advertised CAN clock deterministic before host computes bit timing. */
    CanApplyGsClockRoot();
    canClock = CLOCK_GetClockRootFreq(kCLOCK_CanClkRoot);
    if (canClock == 0U)
    {
        canClock = 20000000U;
    }
    s_gsCanBtClockHz = canClock;

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

uint8_t GsCanDecodeChannelFromSetup(const usb_setup_struct_t *setup)
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

    /* Prefer explicit non-zero channel values first (handles wValue-hi/lo variants). */
    if ((channelLo < GS_USB_CHANNEL_COUNT) && (channelLo != 0U))
    {
        return channelLo;
    }
    if ((channelHi < GS_USB_CHANNEL_COUNT) && (channelHi != 0U))
    {
        return channelHi;
    }
    if ((indexHi < GS_USB_CHANNEL_COUNT) && (indexHi != 0U))
    {
        return indexHi;
    }

    /* Linux gs_usb canonical path: channel in wIndex low byte. */
    if (indexLo < GS_USB_CHANNEL_COUNT)
    {
        return indexLo;
    }

    /* Fallbacks for channel 0 encodings. */
    if (channelLo < GS_USB_CHANNEL_COUNT)
    {
        return channelLo;
    }
    if (channelHi < GS_USB_CHANNEL_COUNT)
    {
        return channelHi;
    }
    if (indexHi < GS_USB_CHANNEL_COUNT)
    {
        return indexHi;
    }

    return 0xFFU;
}
