#include "led_blinky_shared.h"

/*
 * Module: USB callbacks/init sequence + runtime tasks + system entry point.
 * Focus area when endpoints are not primed or shell/gs traffic does not progress.
 */
/*******************************************************************************
 * USB callbacks
 ******************************************************************************/
usb_status_t USB_DeviceCdcVcomCallback(class_handle_t handle, uint32_t event, void *param)
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

usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param)
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
void USB_DeviceClockInit(void)
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

void USB_DeviceIsrEnable(void)
{
    uint8_t irqNumber;
    uint8_t usbDeviceEhciIrq[] = USBHS_IRQS;

    irqNumber = usbDeviceEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
    EnableIRQ((IRQn_Type)irqNumber);
}

void USB_DeviceApplicationInit(void)
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
