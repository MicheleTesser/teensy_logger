#include "app/app_shared.h"

/*
 * Module: CAN controller low-level primitives + MF4 block writer internals.
 * Focus area when debugging CAN bring-up, TX mailbox recovery, or log file format.
 */
status_t CanReleaseStopMode(CAN_Type *base)
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

status_t CanInitOneWithFlags(can_bus_context_t *bus, uint32_t bitRate, uint32_t modeFlags)
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

status_t CanDeinitOne(can_bus_context_t *bus)
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

uint32_t CanEstimateFrameBits(const flexcan_frame_t *frame)
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

void CanUpdateUtilization(TickType_t now)
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

void CanResetRuntimeStats(void)
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

status_t CanWriteTxMbWithRecovery(can_bus_context_t *bus, uint8_t mbIdx, const flexcan_frame_t *frame)
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

status_t CanSendFrame(can_bus_context_t *bus, uint8_t mbIdx, uint32_t id, const uint8_t *payload, uint8_t length)
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

status_t CanSendFrameRaw(can_bus_context_t *bus, uint8_t mbIdx, const flexcan_frame_t *frame)
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

void CanDumpPrintFrame(size_t busIndex, const flexcan_frame_t *frame, status_t status)
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

uint32_t Mf4Align8(uint32_t value)
{
    return (value + 7U) & ~7U;
}

void Mf4PutU16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

void Mf4PutU32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16U) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

void Mf4PutU64(uint8_t *dst, uint64_t value)
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

void Mf4PutS16(uint8_t *dst, int16_t value)
{
    Mf4PutU16(dst, (uint16_t)value);
}

void Mf4PutDouble(uint8_t *dst, double value)
{
    union
    {
        double d;
        uint64_t u;
    } v;
    v.d = value;
    Mf4PutU64(dst, v.u);
}

bool Mf4WriteAt(FIL *file, FSIZE_t offset, const void *data, UINT size)
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

bool Mf4ReadAt(FIL *file, FSIZE_t offset, void *data, UINT size)
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

uint64_t Mf4UnixSecondsToNano(uint32_t unixSeconds)
{
    return ((uint64_t)unixSeconds) * 1000000000ULL;
}

void Mf4ComputeLayout(uint32_t *txOffsets, uint32_t *dtBlockOffset, uint32_t *dtDataOffset)
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

bool Mf4WriteTextBlock(FIL *file, uint32_t offset, const char *text)
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

bool Mf4WriteFreshFile(FIL *file, mf4_log_state_t *state)
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

bool Mf4PatchCounters(FIL *file, const mf4_log_state_t *state)
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

bool Mf4Sync(FIL *file, const mf4_log_state_t *state)
{
    if (!Mf4PatchCounters(file, state))
    {
        return false;
    }
    return f_sync(file) == FR_OK;
}

bool Mf4RecoverFile(FIL *file, mf4_log_state_t *state)
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

bool CanLogEnqueue(size_t busIndex, const flexcan_frame_t *frame, status_t status)
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

void CanHandleRxFrame(size_t busIndex, const flexcan_frame_t *frame, status_t status)
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
