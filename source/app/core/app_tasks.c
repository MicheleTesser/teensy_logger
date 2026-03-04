#include "app/app_shared.h"

/*******************************************************************************
 * Runtime tasks
 ******************************************************************************/

void LedTask(void *pvParameters)
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

void CanTask(void *pvParameters)
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

void CanLogTask(void *pvParameters)
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

void AppTask(void *pvParameters)
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
