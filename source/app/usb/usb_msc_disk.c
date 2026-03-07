#include "app/app_shared.h"

#define MSC_RW_BUFFER_SIZE_BYTES 4096U
#define MSC_DEFAULT_SECTOR_SIZE  512U
#define MSC_LOGICAL_UNITS        1U

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static usb_device_inquiry_data_fromat_struct_t s_mscInquiryInfo = {
    (USB_DEVICE_MSC_UFI_PERIPHERAL_QUALIFIER << USB_DEVICE_MSC_UFI_PERIPHERAL_QUALIFIER_SHIFT) |
        USB_DEVICE_MSC_UFI_PERIPHERAL_DEVICE_TYPE,
    (uint8_t)(USB_DEVICE_MSC_UFI_REMOVABLE_MEDIUM_BIT << USB_DEVICE_MSC_UFI_REMOVABLE_MEDIUM_BIT_SHIFT),
    USB_DEVICE_MSC_UFI_VERSIONS,
    0x02U,
    USB_DEVICE_MSC_UFI_ADDITIONAL_LENGTH,
    {0x00U, 0x00U, 0x00U},
    {'R', 'A', 'C', 'E', 'U', 'P', ' ', ' '},
    {'T', 'E', 'E', 'N', 'S', 'Y', ' ', 'L', 'O', 'G', 'G', 'E', 'R', ' ', ' ', ' '},
    {'0', '0', '0', '1'},
};

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static usb_device_mode_parameters_header_struct_t s_mscModeParametersHeader = {
    0x0000U,
    0x00U,
    0x00U,
    {0x00U, 0x00U, 0x00U, 0x00U},
};

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_mscReadBuffer[MSC_RW_BUFFER_SIZE_BYTES];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_mscWriteBuffer[MSC_RW_BUFFER_SIZE_BYTES];

static uint32_t s_mscSectorSize  = MSC_DEFAULT_SECTOR_SIZE;
static uint32_t s_mscSectorCount = 0U;
static bool s_mscDiskInitialized = false;

static void MscSetNotReadySense(usb_device_ufi_app_struct_t *ufi)
{
    if ((ufi == NULL) || (ufi->requestSense == NULL))
    {
        return;
    }

    ufi->requestSense->senseKey = USB_DEVICE_MSC_UFI_NOT_READY;
    ufi->requestSense->additionalSenseCode = USB_DEVICE_MSC_UFI_ASC_MEDIUM_NOT_PRESENT;
    ufi->requestSense->additionalSenseQualifer = USB_DEVICE_MSC_UFI_NO_SENSE;
}

static bool MscEnsureGeometry(void)
{
    DSTATUS status;
    DRESULT result;
    DWORD sectorCount = 0U;
    WORD sectorSize = 0U;

    /* fsl_sd_disk status is not reliable before first init; force init once. */
    if (!s_mscDiskInitialized)
    {
        status = disk_initialize((BYTE)SDDISK);
        if ((status & STA_NOINIT) != 0U)
        {
            return false;
        }
        s_mscDiskInitialized = true;
    }

    result = disk_ioctl((BYTE)SDDISK, GET_SECTOR_COUNT, &sectorCount);
    if ((result != RES_OK) || (sectorCount == 0U))
    {
        /* Retry one full reinit if geometry looks invalid (card reinsert/reset). */
        status = disk_initialize((BYTE)SDDISK);
        if ((status & STA_NOINIT) != 0U)
        {
            s_mscDiskInitialized = false;
            return false;
        }
        result = disk_ioctl((BYTE)SDDISK, GET_SECTOR_COUNT, &sectorCount);
        if ((result != RES_OK) || (sectorCount == 0U))
        {
            s_mscDiskInitialized = false;
            return false;
        }
    }

    result = disk_ioctl((BYTE)SDDISK, GET_SECTOR_SIZE, &sectorSize);
    if ((result != RES_OK) || (sectorSize == 0U))
    {
        sectorSize = (WORD)MSC_DEFAULT_SECTOR_SIZE;
    }

    s_mscSectorCount = (uint32_t)sectorCount;
    s_mscSectorSize = (uint32_t)sectorSize;
    return true;
}

static bool MscValidateLba(const usb_device_lba_app_struct_t *lba, uint32_t *sectorCount)
{
    uint32_t sectors;

    if ((lba == NULL) || (sectorCount == NULL) || (s_mscSectorSize == 0U))
    {
        return false;
    }

    if ((lba->size % s_mscSectorSize) != 0U)
    {
        return false;
    }

    sectors = lba->size / s_mscSectorSize;
    if (sectors > (MSC_RW_BUFFER_SIZE_BYTES / s_mscSectorSize))
    {
        return false;
    }

    if ((lba->offset > s_mscSectorCount) || (sectors > (s_mscSectorCount - lba->offset)))
    {
        return false;
    }

    *sectorCount = sectors;
    return true;
}

usb_status_t USB_DeviceMscCallback(class_handle_t handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Success;
    usb_device_lba_information_struct_t *lbaInfo;
    usb_device_lba_app_struct_t *lba;
    usb_device_ufi_app_struct_t *ufi;
    usb_device_capacity_information_struct_t *capacity;
    uint32_t sectorCount;

    (void)handle;

    switch (event)
    {
        case kUSB_DeviceMscEventReadResponse:
            break;

        case kUSB_DeviceMscEventWriteResponse:
            lba = (usb_device_lba_app_struct_t *)param;
            if ((lba == NULL) || (lba->size == 0U))
            {
                break;
            }
            if (!MscEnsureGeometry() || !MscValidateLba(lba, &sectorCount))
            {
                return kStatus_USB_Error;
            }
            if (disk_write((BYTE)SDDISK, lba->buffer, (LBA_t)lba->offset, (UINT)sectorCount) != RES_OK)
            {
                return kStatus_USB_Error;
            }
            (void)disk_ioctl((BYTE)SDDISK, CTRL_SYNC, NULL);
            break;

        case kUSB_DeviceMscEventWriteRequest:
            lba = (usb_device_lba_app_struct_t *)param;
            if (lba == NULL)
            {
                return kStatus_USB_InvalidRequest;
            }
            lba->buffer = s_mscWriteBuffer;
            break;

        case kUSB_DeviceMscEventReadRequest:
            lba = (usb_device_lba_app_struct_t *)param;
            if (lba == NULL)
            {
                return kStatus_USB_InvalidRequest;
            }
            if (!MscEnsureGeometry() || !MscValidateLba(lba, &sectorCount))
            {
                return kStatus_USB_Error;
            }
            lba->buffer = s_mscReadBuffer;
            if (disk_read((BYTE)SDDISK, lba->buffer, (LBA_t)lba->offset, (UINT)sectorCount) != RES_OK)
            {
                return kStatus_USB_Error;
            }
            break;

        case kUSB_DeviceMscEventGetLbaInformation:
            lbaInfo = (usb_device_lba_information_struct_t *)param;
            if (lbaInfo == NULL)
            {
                return kStatus_USB_InvalidRequest;
            }
            if (!MscEnsureGeometry())
            {
                return kStatus_USB_Error;
            }
            lbaInfo->logicalUnitNumberSupported = MSC_LOGICAL_UNITS;
            lbaInfo->logicalUnitInformations[0].lengthOfEachLba = s_mscSectorSize;
            lbaInfo->logicalUnitInformations[0].totalLbaNumberSupports = s_mscSectorCount;
            lbaInfo->logicalUnitInformations[0].bulkInBufferSize = MSC_RW_BUFFER_SIZE_BYTES;
            lbaInfo->logicalUnitInformations[0].bulkOutBufferSize = MSC_RW_BUFFER_SIZE_BYTES;
            break;

        case kUSB_DeviceMscEventTestUnitReady:
            ufi = (usb_device_ufi_app_struct_t *)param;
            if (!MscEnsureGeometry())
            {
                MscSetNotReadySense(ufi);
            }
            break;

        case kUSB_DeviceMscEventInquiry:
            ufi = (usb_device_ufi_app_struct_t *)param;
            if (ufi == NULL)
            {
                return kStatus_USB_InvalidRequest;
            }
            ufi->size = sizeof(s_mscInquiryInfo);
            ufi->buffer = (uint8_t *)&s_mscInquiryInfo;
            break;

        case kUSB_DeviceMscEventModeSense:
            ufi = (usb_device_ufi_app_struct_t *)param;
            if (ufi == NULL)
            {
                return kStatus_USB_InvalidRequest;
            }
            ufi->size = sizeof(s_mscModeParametersHeader);
            ufi->buffer = (uint8_t *)&s_mscModeParametersHeader;
            break;

        case kUSB_DeviceMscEventReadCapacity:
        case kUSB_DeviceMscEventReadFormatCapacity:
            capacity = (usb_device_capacity_information_struct_t *)param;
            if (capacity == NULL)
            {
                return kStatus_USB_InvalidRequest;
            }
            if (!MscEnsureGeometry())
            {
                return kStatus_USB_Error;
            }
            capacity->lengthOfEachLba = s_mscSectorSize;
            capacity->totalLbaNumberSupports = s_mscSectorCount;
            break;

        case kUSB_DeviceMscEventModeSelect:
        case kUSB_DeviceMscEventModeSelectResponse:
        case kUSB_DeviceMscEventFormatComplete:
        case kUSB_DeviceMscEventRemovalRequest:
        case kUSB_DeviceMscEventRequestSense:
        case kUSB_DeviceMscEventSendDiagnostic:
        case kUSB_DeviceMscEventStopEjectMedia:
            break;

        default:
            error = kStatus_USB_InvalidRequest;
            break;
    }

    return error;
}
