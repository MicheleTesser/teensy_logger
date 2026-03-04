#include "app/app_shared.h"

/*
 * Module: CAN runtime/polling + shell command handlers + gs_usb protocol control path.
 * Focus area when host sees only one channel, deferred start/reset, or queue stalls.
 */
void CanPollRxMb(can_bus_context_t *bus, size_t busIndex, uint8_t mbIdx)
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

void CanPollRxDump(void)
{
    size_t i;

    if (!s_canReady)
    {
        return;
    }

    for (i = 0U; i < ARRAY_SIZE(s_canBuses); i++)
    {
      if (s_canBuses[i].started)
      {
        CanPollRxMb(&s_canBuses[i], i, s_canBuses[i].rxStdMb);
        CanPollRxMb(&s_canBuses[i], i, s_canBuses[i].rxExtMb);
      }
    }
}

status_t CanInitAll(uint32_t bitRate, uint32_t modeFlags)
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

status_t CanSendFromShell(uint32_t busIndex, uint32_t id, const uint8_t *payload, uint8_t length)
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

void CanCommand(int32_t argc, char *argv[])
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

void ShellPrintHelp(void)
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

void ShellHandleLine(char *line)
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

void ShellHandlePacket(const uint8_t *data, uint32_t length)
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

void CanRefreshReadyState(void)
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

bool CanAnyControllerStarted(void)
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

void CanApplyGsClockRoot(void)
{
    /* CAN1/CAN2 share the same root; never touch it while any controller is running. */
    if (CanAnyControllerStarted())
    {
        return;
    }
    CLOCK_SetMux(kCLOCK_CanMux, 0U);
    CLOCK_SetDiv(kCLOCK_CanDiv, 2U);
}

void GsCanProcessDeferredRequests(void)
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

void GsCanHandleBulkOutPacket(const uint8_t *data, uint32_t length)
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

bool GsCanHandleVendorRequest(usb_device_control_request_struct_t *controlRequest)
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

                    /* Convert timing using the same clock advertised via BT_CONST. */
                    canClock  = s_gsCanBtClockHz;
                    if (canClock == 0U)
                    {
                        canClock = CLOCK_GetClockRootFreq(kCLOCK_CanClkRoot);
                    }
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

usb_status_t USB_GsCanBulkOutCallback(usb_device_handle handle,
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

usb_status_t USB_GsCanBulkInCallback(usb_device_handle handle,
                                            usb_device_endpoint_callback_message_struct_t *message,
                                            void *callbackParam)
{
    (void)handle;
    (void)message;
    (void)callbackParam;

    s_gsCanInBusy = false;
    return kStatus_USB_Success;
}
