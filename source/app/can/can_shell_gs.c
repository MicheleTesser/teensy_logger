#include "app/app_shared.h"

/*
 * Module: CAN runtime/polling + shell command handlers + gs_usb protocol control path.
 * Focus area when host sees only one channel, deferred start/reset, or queue stalls.
 */
static const uint8_t s_gsCanTxMbList[] = {
    12U, 13U, 14U, 15U, 16U, 17U, 18U, 19U, 20U, 21U,
    22U, 23U, 24U, 25U, 26U, 27U, 28U, 29U, 30U, 31U,
};

static void CanRxFifoPushFromIsr(size_t busIndex, const flexcan_frame_t *frame, status_t status)
{
    uint16_t head;
    uint16_t nextHead;

    if ((frame == NULL) || (busIndex >= GS_USB_CHANNEL_COUNT))
    {
        return;
    }

    head = s_canRxFifoHead[busIndex];
    nextHead = (uint16_t)((head + 1U) & CAN_ISR_RX_FIFO_MASK);
    if (nextHead == s_canRxFifoTail[busIndex])
    {
        s_canRxFifoOverflow[busIndex] = true;
        return;
    }

    s_canRxFifo[busIndex][head].frame = *frame;
    s_canRxFifo[busIndex][head].status = status;
    s_canRxFifoHead[busIndex] = nextHead;
}

static void CanDrainRxMbIsr(can_bus_context_t *bus, size_t busIndex, uint8_t mbIdx)
{
    uint64_t mbMask;
    uint32_t drainGuard = 0U;

    if ((bus == NULL) || !bus->started)
    {
        return;
    }

    mbMask = (uint64_t)1U << mbIdx;
    while (drainGuard < 64U)
    {
        flexcan_frame_t frame;
        status_t status;

        if (FLEXCAN_GetMbStatusFlags(bus->base, mbMask) == 0U)
        {
            break;
        }

        status = FLEXCAN_ReadRxMb(bus->base, mbIdx, &frame);
        FLEXCAN_ClearMbStatusFlags(bus->base, mbMask);
        if ((status == kStatus_Success) || (status == kStatus_FLEXCAN_RxOverflow))
        {
            CanRxFifoPushFromIsr(busIndex, &frame, status);
        }
        drainGuard++;
    }
}

static void CanServiceIsr(size_t busIndex)
{
    if (busIndex >= ARRAY_SIZE(s_canBuses))
    {
        return;
    }

    CanDrainRxMbIsr(&s_canBuses[busIndex], busIndex, s_canBuses[busIndex].rxStdMb);
    CanDrainRxMbIsr(&s_canBuses[busIndex], busIndex, s_canBuses[busIndex].rxExtMb);
}

void CAN1_IRQHandler(void)
{
    CanServiceIsr(0U);
    SDK_ISR_EXIT_BARRIER;
}

void CAN2_IRQHandler(void)
{
    CanServiceIsr(1U);
    SDK_ISR_EXIT_BARRIER;
}

void CanRxFifoReset(size_t busIndex)
{
    if (busIndex >= GS_USB_CHANNEL_COUNT)
    {
        return;
    }

    taskENTER_CRITICAL();
    s_canRxFifoHead[busIndex] = 0U;
    s_canRxFifoTail[busIndex] = 0U;
    s_canRxFifoOverflow[busIndex] = false;
    taskEXIT_CRITICAL();
}

bool CanRxFifoPop(size_t busIndex, flexcan_frame_t *frame, status_t *status)
{
    bool hasFrame = false;

    if ((busIndex >= GS_USB_CHANNEL_COUNT) || (frame == NULL) || (status == NULL))
    {
        return false;
    }

    taskENTER_CRITICAL();
    if (s_canRxFifoTail[busIndex] != s_canRxFifoHead[busIndex])
    {
        uint16_t tail = s_canRxFifoTail[busIndex];
        *frame = s_canRxFifo[busIndex][tail].frame;
        *status = s_canRxFifo[busIndex][tail].status;
        s_canRxFifoTail[busIndex] = (uint16_t)((tail + 1U) & CAN_ISR_RX_FIFO_MASK);
        hasFrame = true;
    }
    taskEXIT_CRITICAL();

    return hasFrame;
}

bool CanProcessRxFifo(uint32_t budgetPerBus)
{
    bool processedAny = false;
    size_t i;

    if (budgetPerBus == 0U)
    {
        budgetPerBus = 1U;
    }

    for (i = 0U; i < ARRAY_SIZE(s_canBuses); i++)
    {
        uint32_t budget = budgetPerBus;
        flexcan_frame_t frame;
        status_t status;

        while ((budget > 0U) && CanRxFifoPop(i, &frame, &status))
        {
            CanHandleRxFrame(i, &frame, status);
            processedAny = true;
            budget--;
        }

        if (s_canRxFifoOverflow[i])
        {
            s_canRxFifoOverflow[i] = false;
            s_canBuses[i].rxOverflowCount++;
            GsCanSetOverflowFlag(i);
            processedAny = true;
        }
    }

    return processedAny;
}

void GsCanTxQueueReset(size_t busIndex)
{
    if (busIndex >= GS_USB_CHANNEL_COUNT)
    {
        return;
    }

    taskENTER_CRITICAL();
    s_canGsTxHead[busIndex] = 0U;
    s_canGsTxTail[busIndex] = 0U;
    s_canGsTxCount[busIndex] = 0U;
    s_canGsTxMbCursor[busIndex] = 0U;
    taskEXIT_CRITICAL();
}

bool GsCanTxQueuePush(size_t busIndex, uint32_t echoId, const flexcan_frame_t *frame)
{
    uint16_t head;
    uint16_t nextHead;

    if ((busIndex >= GS_USB_CHANNEL_COUNT) || (frame == NULL))
    {
        return false;
    }

    taskENTER_CRITICAL();
    head = s_canGsTxHead[busIndex];
    nextHead = (uint16_t)((head + 1U) & CAN_GS_TX_QUEUE_MASK);
    if (nextHead == s_canGsTxTail[busIndex])
    {
        taskEXIT_CRITICAL();
        return false;
    }

    s_canGsTxQueue[busIndex][head].frame = *frame;
    s_canGsTxQueue[busIndex][head].echoId = echoId;
    s_canGsTxHead[busIndex] = nextHead;
    s_canGsTxCount[busIndex]++;
    taskEXIT_CRITICAL();
    return true;
}

static bool GsCanTxQueuePeek(size_t busIndex, can_gs_tx_entry_t *entry)
{
    bool hasEntry = false;

    if ((busIndex >= GS_USB_CHANNEL_COUNT) || (entry == NULL))
    {
        return false;
    }

    taskENTER_CRITICAL();
    if (s_canGsTxTail[busIndex] != s_canGsTxHead[busIndex])
    {
        *entry = s_canGsTxQueue[busIndex][s_canGsTxTail[busIndex]];
        hasEntry = true;
    }
    taskEXIT_CRITICAL();
    return hasEntry;
}

static void GsCanTxQueueDropFront(size_t busIndex)
{
    if (busIndex >= GS_USB_CHANNEL_COUNT)
    {
        return;
    }

    taskENTER_CRITICAL();
    if (s_canGsTxTail[busIndex] != s_canGsTxHead[busIndex])
    {
        s_canGsTxTail[busIndex] = (uint16_t)((s_canGsTxTail[busIndex] + 1U) & CAN_GS_TX_QUEUE_MASK);
        if (s_canGsTxCount[busIndex] > 0U)
        {
            s_canGsTxCount[busIndex]--;
        }
    }
    taskEXIT_CRITICAL();
}

bool GsCanTxQueueHasPending(void)
{
    return (s_canGsTxCount[0] != 0U) || (s_canGsTxCount[1] != 0U);
}

static bool GsCanTrySendQueuedFrame(size_t busIndex, const can_gs_tx_entry_t *entry)
{
    can_bus_context_t *bus;
    size_t attempt;

    if ((entry == NULL) || (busIndex >= ARRAY_SIZE(s_canBuses)))
    {
        return false;
    }

    bus = &s_canBuses[busIndex];
    for (attempt = 0U; attempt < ARRAY_SIZE(s_gsCanTxMbList); attempt++)
    {
        uint8_t cursor = s_canGsTxMbCursor[busIndex];
        uint8_t mbIdx = s_gsCanTxMbList[cursor];
        status_t status = CanWriteTxMbWithRecovery(bus, mbIdx, &entry->frame);
        cursor++;
        if (cursor >= ARRAY_SIZE(s_gsCanTxMbList))
        {
            cursor = 0U;
        }
        s_canGsTxMbCursor[busIndex] = cursor;

        if (status == kStatus_Success)
        {
            bus->txOkCount++;
            bus->txBitCount += CanEstimateFrameBits(&entry->frame);
            LedMarkCanActivity();
            return true;
        }
        if (status != kStatus_FLEXCAN_TxBusy)
        {
            bus->txErrCount++;
        }
    }

    return false;
}

bool GsCanServiceTxQueues(uint32_t budgetPerBus)
{
    bool sentAny = false;
    size_t busIndex;

    if (budgetPerBus == 0U)
    {
        budgetPerBus = 1U;
    }

    for (busIndex = 0U; busIndex < GS_USB_CHANNEL_COUNT; busIndex++)
    {
        uint32_t budget = budgetPerBus;

        if (!s_canBuses[busIndex].started)
        {
            continue;
        }

        while (budget > 0U)
        {
            can_gs_tx_entry_t entry;

            if (!GsCanTxQueuePeek(busIndex, &entry))
            {
                break;
            }

            if (!GsCanTrySendQueuedFrame(busIndex, &entry))
            {
                break;
            }

            GsCanTxQueueDropFront(busIndex);
            if (!GsCanTxEchoPublish(busIndex, entry.echoId, &entry.frame))
            {
                GsCanSetOverflowFlag(busIndex);
            }
            sentAny = true;
            budget--;
        }
    }

    return sentAny;
}

void CanPollRxMb(can_bus_context_t *bus, size_t busIndex, uint8_t mbIdx)
{
    uint64_t mbMask;
    flexcan_frame_t frame;
    status_t status;
    uint32_t drainGuard = 0U;

    if ((bus == NULL) || !bus->started)
    {
        return;
    }

    mbMask = (uint64_t)1U << mbIdx;
    while (drainGuard < 64U)
    {
        if (APP_LIKELY(FLEXCAN_GetMbStatusFlags(bus->base, mbMask) == 0U))
        {
            break;
        }

        status = FLEXCAN_ReadRxMb(bus->base, mbIdx, &frame);
        FLEXCAN_ClearMbStatusFlags(bus->base, mbMask);
        if (APP_LIKELY(status == kStatus_Success))
        {
            CanHandleRxFrame(busIndex, &frame, status);
        }
        else if (status == kStatus_FLEXCAN_RxOverflow)
        {
            CanHandleRxFrame(busIndex, &frame, status);
        }
        drainGuard++;
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
        if (!s_canBuses[i].started)
        {
            continue;
        }

        CanPollRxMb(&s_canBuses[i], i, s_canBuses[i].rxStdMb);
        CanPollRxMb(&s_canBuses[i], i, s_canBuses[i].rxExtMb);
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

void ShellHandleLine(char *line)
{
    char rawLine[SHELL_LINE_BUFFER_SIZE];
    char *start;
    size_t len;
    char cliOutput[SHELL_CLI_OUTPUT_SIZE];
    BaseType_t moreData;
    const char *commandInput;

    if (line == NULL)
    {
        return;
    }

    (void)strncpy(rawLine, line, sizeof(rawLine) - 1U);
    rawLine[sizeof(rawLine) - 1U] = '\0';

    /* Normalize: trim leading/trailing spaces/tabs to avoid false "unknown command". */
    start = rawLine;
    while ((*start == ' ') || (*start == '\t'))
    {
        start++;
    }
    if (*start == '\0')
    {
        return;
    }
    len = strlen(start);
    while ((len > 0U) && ((start[len - 1U] == ' ') || (start[len - 1U] == '\t')))
    {
        start[len - 1U] = '\0';
        len--;
    }
    if (start[0] == '\0')
    {
        return;
    }

    if (!s_shellCliReady)
    {
        ShellWrite("CLI non pronta\r\n");
        return;
    }

    commandInput = start;
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
}

void ShellHandlePacket(const uint8_t *data, uint32_t length)
{
    static char lineBuffer[SHELL_LINE_BUFFER_SIZE];
    static size_t lineLength = 0U;
    static bool skipNextLf   = false;
    static bool inAnsiEscape = false;
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

        /* Ignore simple ANSI escape sequences often emitted by terminal arrow keys. */
        if (inAnsiEscape)
        {
            if (((ch >= (uint8_t)'A') && (ch <= (uint8_t)'Z')) ||
                ((ch >= (uint8_t)'a') && (ch <= (uint8_t)'z')) || (ch == (uint8_t)'~'))
            {
                inAnsiEscape = false;
            }
            continue;
        }
        if (ch == 0x1BU)
        {
            inAnsiEscape = true;
            continue;
        }

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

    for (i = 0U; i < ARRAY_SIZE(s_canBuses); i++)
    {
        if (s_canBuses[i].started)
        {
            s_canReady = true;
            return;
        }
    }
    s_canReady = false;
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

        if (APP_LIKELY(s_gsCanDeferredReq[channel].pending == 0U))
        {
            continue;
        }

        op = s_gsCanDeferredReq[channel].op;
        bitRate = s_gsCanDeferredReq[channel].bitRate;
        flags = s_gsCanDeferredReq[channel].flags & GS_CAN_MODE_SUPPORTED_MASK;
        s_gsCanDeferredReq[channel].pending = 0U;
        s_gsCanDeferredReq[channel].op = GS_CAN_DEFERRED_OP_NONE;

        switch (op)
        {
            case GS_CAN_DEFERRED_OP_START:
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
                break;
            }
            case GS_CAN_DEFERRED_OP_RESET:
            {
                if (s_canBuses[channel].started)
                {
                    (void)CanDeinitOne(&s_canBuses[channel]);
                }
                break;
            }
            default:
                break;
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
            txFrame.dataByte0 = hostFrame->data[0];
            txFrame.dataByte1 = hostFrame->data[1];
            txFrame.dataByte2 = hostFrame->data[2];
            txFrame.dataByte3 = hostFrame->data[3];
            txFrame.dataByte4 = hostFrame->data[4];
            txFrame.dataByte5 = hostFrame->data[5];
            txFrame.dataByte6 = hostFrame->data[6];
            txFrame.dataByte7 = hostFrame->data[7];
        }

        if (!GsCanTxQueuePush(channel, echoId, &txFrame))
        {
            s_canBuses[channel].txErrCount++;
            /* Never leave host TX frames without completion echo: otherwise SocketCAN can stall. */
            if (!GsCanTxEchoPublish(channel, echoId, &txFrame))
            {
                GsCanSetOverflowFlag(channel);
            }
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
