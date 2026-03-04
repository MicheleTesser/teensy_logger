#include "app/app_shared.h"

/*
 * Module: RTC + filesystem shell commands.
 * Focus area when checking time validity, mount state, and SD I/O behavior.
 */
bool RtcIsLeapYear(uint16_t year)
{
    if ((year % 4U) != 0U)
    {
        return false;
    }
    if ((year % 100U) != 0U)
    {
        return true;
    }
    return (year % 400U) == 0U;
}

uint64_t RtcReadRawCounter(void)
{
    uint32_t msbA;
    uint32_t lsb;
    uint32_t msbB;
    uint32_t retry = RTC_COUNTER_SYNC_RETRY;

    do
    {
        msbA = SNVS->LPSRTCMR & SNVS_LPSRTCMR_SRTC_MASK;
        lsb  = SNVS->LPSRTCLR;
        msbB = SNVS->LPSRTCMR & SNVS_LPSRTCMR_SRTC_MASK;
        retry--;
    } while ((msbA != msbB) && (retry > 0U));

    return (((uint64_t)msbA << 32U) | (uint64_t)lsb);
}

bool RtcEnableCounter(bool enable)
{
    uint32_t timeout = RTC_ENABLE_TIMEOUT_LOOPS;

    if (enable)
    {
        SNVS->LPCR |= SNVS_LPCR_SRTC_ENV_MASK;
    }
    else
    {
        SNVS->LPCR &= ~SNVS_LPCR_SRTC_ENV_MASK;
    }

    while (timeout > 0U)
    {
        bool isEnabled = (SNVS->LPCR & SNVS_LPCR_SRTC_ENV_MASK) != 0U;
        if (isEnabled == enable)
        {
            return true;
        }
        timeout--;
    }
    return false;
}

bool RtcInit(void)
{
    if (s_rtcInitialized)
    {
        return true;
    }

    /* Clear sticky LP status flags. */
    SNVS->LPSR |= SNVS->LPSR;

    if ((SNVS->LPCR & SNVS_LPCR_SRTC_ENV_MASK) == 0U)
    {
        if (!RtcEnableCounter(true))
        {
            return false;
        }
    }

    s_rtcInitialized = true;
    return true;
}

bool RtcIsValid(void)
{
    if (!RtcInit())
    {
        return false;
    }

    if ((SNVS->LPCR & SNVS_LPCR_SRTC_ENV_MASK) == 0U)
    {
        return false;
    }

    return (SNVS->LPGPR[0] == RTC_VALID_GPR_MAGIC);
}

uint32_t RtcGetUnixSeconds(void)
{
    uint64_t counter;

    if (!RtcInit())
    {
        return 0U;
    }

    counter = RtcReadRawCounter();
    return (uint32_t)(counter >> RTC_COUNTER_SHIFT);
}

bool RtcSetUnixSeconds(uint32_t unixSeconds)
{
    if (!RtcInit())
    {
        return false;
    }

    if (!RtcEnableCounter(false))
    {
        return false;
    }

    SNVS->LPSRTCLR = (uint32_t)(unixSeconds << RTC_COUNTER_SHIFT);
    SNVS->LPSRTCMR = (uint32_t)(unixSeconds >> (32U - RTC_COUNTER_SHIFT));

    if (!RtcEnableCounter(true))
    {
        return false;
    }

    SNVS->LPGPR[0] = RTC_VALID_GPR_MAGIC;
    SNVS->LPGPR[1] = unixSeconds;
    return true;
}

bool RtcUnixSecondsToDateTime(uint32_t unixSeconds, rtc_datetime_t *dateTime)
{
    static const uint8_t kMonthDays[12] = {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};
    uint32_t days;
    uint32_t secOfDay;
    uint16_t year = 1970U;
    uint8_t month = 1U;
    uint8_t i;

    if (dateTime == NULL)
    {
        return false;
    }

    days    = unixSeconds / RTC_SECONDS_PER_DAY;
    secOfDay = unixSeconds % RTC_SECONDS_PER_DAY;

    while (true)
    {
        uint32_t daysThisYear = RtcIsLeapYear(year) ? RTC_DAYS_PER_LEAP_YEAR : RTC_DAYS_PER_NORMAL_YEAR;
        if (days < daysThisYear)
        {
            break;
        }
        days -= daysThisYear;
        year++;
    }

    for (i = 0U; i < 12U; i++)
    {
        uint32_t daysThisMonth = kMonthDays[i];
        if ((i == 1U) && RtcIsLeapYear(year))
        {
            daysThisMonth++;
        }
        if (days < daysThisMonth)
        {
            month = (uint8_t)(i + 1U);
            break;
        }
        days -= daysThisMonth;
    }

    dateTime->year   = year;
    dateTime->month  = month;
    dateTime->day    = (uint8_t)(days + 1U);
    dateTime->hour   = (uint8_t)(secOfDay / RTC_SECONDS_PER_HOUR);
    secOfDay        %= RTC_SECONDS_PER_HOUR;
    dateTime->minute = (uint8_t)(secOfDay / RTC_SECONDS_PER_MINUTE);
    dateTime->second = (uint8_t)(secOfDay % RTC_SECONDS_PER_MINUTE);
    return true;
}

void RtcCommand(int32_t argc, char *argv[])
{
    uint32_t unixSeconds;
    rtc_datetime_t dt;

    if (argc < 2)
    {
        ShellWrite("Uso: rtc <status|set>\r\n");
        return;
    }

    if (strcmp(argv[1], "status") == 0)
    {
        unixSeconds = RtcGetUnixSeconds();
        (void)RtcUnixSecondsToDateTime(unixSeconds, &dt);
        ShellWriteLinef("RTC init=%u valid=%u unix=%lu %04u-%02u-%02u %02u:%02u:%02u UTC\r\n",
                        s_rtcInitialized ? 1U : 0U, RtcIsValid() ? 1U : 0U, (unsigned long)unixSeconds,
                        (unsigned int)dt.year, (unsigned int)dt.month, (unsigned int)dt.day,
                        (unsigned int)dt.hour, (unsigned int)dt.minute, (unsigned int)dt.second);
        return;
    }

    if (strcmp(argv[1], "set") == 0)
    {
        if ((argc < 3) || !ShellParseU32Auto(argv[2], &unixSeconds))
        {
            ShellWrite("Uso: rtc set <unix_epoch>\r\n");
            return;
        }

        if (!RtcSetUnixSeconds(unixSeconds))
        {
            ShellWrite("RTC set fallito\r\n");
            return;
        }

        (void)RtcUnixSecondsToDateTime(unixSeconds, &dt);
        ShellWriteLinef("RTC impostato: %lu (%04u-%02u-%02u %02u:%02u:%02u UTC)\r\n",
                        (unsigned long)unixSeconds, (unsigned int)dt.year, (unsigned int)dt.month,
                        (unsigned int)dt.day, (unsigned int)dt.hour, (unsigned int)dt.minute,
                        (unsigned int)dt.second);
        return;
    }

    ShellWrite("Uso: rtc <status|set>\r\n");
}

DWORD get_fattime(void)
{
    rtc_datetime_t dt;
    uint16_t year;

    if (RtcIsValid())
    {
        (void)RtcUnixSecondsToDateTime(RtcGetUnixSeconds(), &dt);
    }
    else
    {
        dt.year   = RTC_DEFAULT_FAT_YEAR;
        dt.month  = 1U;
        dt.day    = 1U;
        dt.hour   = 0U;
        dt.minute = 0U;
        dt.second = 0U;
    }

    year = dt.year;
    if (year < 1980U)
    {
        year = 1980U;
    }
    else if (year > 2107U)
    {
        year = 2107U;
    }

    return ((DWORD)(year - 1980U) << 25U) | ((DWORD)dt.month << 21U) | ((DWORD)dt.day << 16U) |
           ((DWORD)dt.hour << 11U) | ((DWORD)dt.minute << 5U) | ((DWORD)(dt.second / 2U));
}

const char *FsResultToString(FRESULT result)
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

FRESULT FsMount(void)
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

FRESULT FsUnmount(void)
{
    if (s_canLogFileOpen)
    {
        if (s_mf4LogState.ready)
        {
            (void)Mf4Sync(&s_canLogFile, &s_mf4LogState);
        }
        else
        {
            (void)f_sync(&s_canLogFile);
        }
        (void)f_close(&s_canLogFile);
        s_canLogFileOpen = false;
    }
    s_mf4LogState.ready = false;

    FRESULT fr = f_mount(NULL, s_fsDrive, 0U);
    if (fr == FR_OK)
    {
        s_fsMounted = false;
    }
    return fr;
}

bool FsEnsureMounted(void)
{
    if (!s_fsMounted)
    {
        ShellWrite("Filesystem non montato. Usa: fs mount\r\n");
        return false;
    }
    return true;
}

bool FsParseU32(const char *text, uint32_t *value)
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

bool ShellParseU32Auto(const char *text, uint32_t *value)
{
    char *end = NULL;
    unsigned long parsed;

    if ((text == NULL) || (value == NULL) || (text[0] == '\0'))
    {
        return false;
    }

    parsed = strtoul(text, &end, 0);
    if ((end == text) || ((end != NULL) && (*end != '\0')) || (parsed > UINT32_MAX))
    {
        return false;
    }

    *value = (uint32_t)parsed;
    return true;
}

bool ShellParseCanBus(const char *text, uint32_t *busIndex)
{
    uint32_t rawBus;

    if ((text == NULL) || (busIndex == NULL))
    {
        return false;
    }

    if ((strcmp(text, "can0") == 0) || (strcmp(text, "CAN0") == 0))
    {
        *busIndex = 1U;
        return true;
    }
    if ((strcmp(text, "can1") == 0) || (strcmp(text, "CAN1") == 0))
    {
        *busIndex = 2U;
        return true;
    }

    if (!ShellParseU32Auto(text, &rawBus))
    {
        return false;
    }
    if ((rawBus == 0U) || (rawBus > ARRAY_SIZE(s_canBuses)))
    {
        return false;
    }

    *busIndex = rawBus;
    return true;
}

const char *ShellSkipToken(const char *cursor)
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

void FsCommandList(const char *path)
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

void FsCommandCat(const char *path)
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

void FsCommandWrite(const char *path, const char *payload, bool append)
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

void FsCommandMkDir(const char *path)
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

void FsCommandRemove(const char *path)
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

void FsCommandMkfs(void)
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

void FsCommandBench(uint32_t totalKiB, uint32_t runs)
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

void FsCommand(int32_t argc, char *argv[], const char *rawLine)
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
