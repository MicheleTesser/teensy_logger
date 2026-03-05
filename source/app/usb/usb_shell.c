#include "app/app_shared.h"

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
    .pcHelpString = "log <status|on|off|usb <on|off>|autocan <on|off>>\r\n",
    .pxCommandInterpreter = ShellCliLogCommand,
    .cExpectedNumberOfParameters = -1,
};

static const CLI_Command_Definition_t s_cliSdCommand = {
    .pcCommand = "sd",
    .pcHelpString = "sd <status|refresh>: stato spazio SD (cache)\r\n",
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
    .pcHelpString = "time [<unix_epoch>|status|set <unix_epoch>]\r\n",
    .pxCommandInterpreter = ShellCliTimeCommand,
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
                   "  log <status|on|off|usb <on|off>|autocan <on|off>>\r\n"
                   "  sd <status|refresh>  - spazio SD totale/libero/usato\r\n"
                   "  rtc status           - stato RTC SNVS (UTC)\r\n"
                   "  rtc set <epoch>      - imposta RTC con Unix epoch\r\n"
                   "  time [epoch]          - alias rapido per leggere/impostare RTC\r\n"
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

    (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: log <status|on|off|usb <on|off>|autocan <on|off>>\r\n");
    return pdFALSE;
}

BaseType_t ShellCliSdCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    const char *subcmd;
    BaseType_t subcmdLen = 0;
    bool forceRefresh = false;
    sd_usage_cache_t usage;
    TickType_t now;
    uint32_t ageMs = 0U;
    FRESULT fr;
    bool hasSnapshot;

    subcmd = FreeRTOS_CLIGetParameter(pcCommandString, 1U, &subcmdLen);
    if (subcmd != NULL)
    {
        if (ShellTokenEquals(subcmd, subcmdLen, "refresh"))
        {
            forceRefresh = true;
        }
        else if (!ShellTokenEquals(subcmd, subcmdLen, "status"))
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: sd <status|refresh>\r\n");
            return pdFALSE;
        }
    }

    if (!s_fsMounted)
    {
        fr = FsMount();
        if (fr != FR_OK)
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "SD non montata: %s (%d)\r\n", FsResultToString(fr), (int)fr);
            return pdFALSE;
        }
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

    (void)snprintf(pcWriteBuffer, xWriteBufferLen,
                   "SD totale=%llu MiB libero=%llu MiB usato=%llu MiB (%u.%u%%) cache_age=%lu ms\r\n"
                   "log=%s\r\n",
                   (unsigned long long)(usage.totalBytes / (1024ULL * 1024ULL)),
                   (unsigned long long)(usage.freeBytes / (1024ULL * 1024ULL)),
                   (unsigned long long)(usage.usedBytes / (1024ULL * 1024ULL)),
                   (unsigned int)(usage.usedPermille / 10U), (unsigned int)(usage.usedPermille % 10U),
                   (unsigned long)ageMs,
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
        if ((param == NULL) || (paramLen <= 0) || ((size_t)paramLen >= sizeof(token)))
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: time [<unix_epoch>|status|set <unix_epoch>]\r\n");
            return pdFALSE;
        }

        (void)memcpy(token, param, (size_t)paramLen);
        token[paramLen] = '\0';
    }
    else
    {
        if ((subcmdLen <= 0) || ((size_t)subcmdLen >= sizeof(token)))
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Uso: time [<unix_epoch>|status|set <unix_epoch>]\r\n");
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

void GsCanTrySendNext(void)
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

bool GsCanTxEchoPublish(size_t busIndex, uint32_t echoId, const flexcan_frame_t *frame)
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
