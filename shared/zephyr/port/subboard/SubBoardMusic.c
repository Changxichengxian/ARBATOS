/* SD 卡歌曲播放和 PD14/PD15 切歌，专供 SENTINEL-M 副板 V2。 */
#include "SubBoardMusic.h"
#include "SubBoardMusicButton.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "BspBuzzer.h"
#include "BspBuzzerM.h"
#include "SdCard.h"
#include "fatfs/ff.h"

LOG_MODULE_REGISTER(sub_music, LOG_LEVEL_INF);

#define SUB_BOARD_MUSIC_ROOT             "0:/"
#define SUB_BOARD_MUSIC_PATH_MAX         384u
#define SUB_BOARD_MUSIC_NAME_MAX         96u
#define SUB_BOARD_MUSIC_TRACK_MAX        64u
#define SUB_BOARD_MUSIC_DIR_DEPTH        4u
#define SUB_BOARD_MUSIC_IO_BYTES         512u
#define SUB_BOARD_MUSIC_PCM_BYTES        256u
#define SUB_BOARD_MUSIC_STACK_BYTES      4096u
#define SUB_BOARD_MUSIC_BUTTON_PERIOD_MS 10u
#define SUB_BOARD_MUSIC_RETRY_MS         2000u
#define SUB_BOARD_MUSIC_DEFAULT_HZ       12000u
#define SUB_BOARD_MUSIC_MAX_HZ           48000u
#define SUB_BOARD_MUSIC_VOLUME           210u
#define SUB_BOARD_MUSIC_PREV_PIN         14u
#define SUB_BOARD_MUSIC_NEXT_PIN         15u

typedef enum
{
    SubBoardMusicU8 = 0,
    SubBoardMusicWavPcm = 1,
} SubBoardMusicFormat;

typedef struct
{
    char path[SUB_BOARD_MUSIC_PATH_MAX];
    char name[SUB_BOARD_MUSIC_NAME_MAX];
    uint32_t sampleRateHz;
    uint32_t dataOffset;
    uint32_t dataBytes;
    uint16_t blockAlign;
    uint8_t channels;
    uint8_t bitsPerSample;
    uint8_t format;
} SubBoardMusicTrack;

/* 显式保存目录遍历状态，避免子目录和长路径耗尽音乐线程栈。 */
typedef struct
{
    DIR dir;
    char path[SUB_BOARD_MUSIC_PATH_MAX];
} SubBoardMusicScanFrame;

static SubBoardMusicScanFrame SubBoardMusicScanFrames[SUB_BOARD_MUSIC_DIR_DEPTH + 1u];
static SubBoardMusicTrack SubBoardMusicTracks[SUB_BOARD_MUSIC_TRACK_MAX];
static uint16_t SubBoardMusicTrackCount;
static SubBoardMusicDiag SubBoardMusicDiagData;
static SubBoardMusicButton SubBoardMusicPrev;
static SubBoardMusicButton SubBoardMusicNext;
static uint8_t SubBoardMusicIo[SUB_BOARD_MUSIC_IO_BYTES];
static uint8_t SubBoardMusicPcm[SUB_BOARD_MUSIC_PCM_BYTES];
static struct k_thread SubBoardMusicThread;
K_THREAD_STACK_DEFINE(SubBoardMusicStack, SUB_BOARD_MUSIC_STACK_BYTES);
static struct k_mutex SubBoardMusicDiagMutex;
static volatile uint8_t SubBoardMusicStarted;
static volatile uint8_t SubBoardMusicStopRequested;
static volatile int8_t SubBoardMusicChange;
static uint8_t SubBoardMusicPlayEnabled;

static const struct device *const SubBoardMusicGpioD = DEVICE_DT_GET(DT_NODELABEL(gpiod));

static uint16_t SubBoardMusicLe16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8u));
}

static uint32_t SubBoardMusicLe32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8u) | ((uint32_t)p[2] << 16u) | ((uint32_t)p[3] << 24u);
}

static uint8_t SubBoardMusicEqNoCase(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0')
    {
        char ca = *a++;
        char cb = *b++;
        if (ca >= 'A' && ca <= 'Z')
        {
            ca = (char)(ca + ('a' - 'A'));
        }
        if (cb >= 'A' && cb <= 'Z')
        {
            cb = (char)(cb + ('a' - 'A'));
        }
        if (ca != cb)
        {
            return 0u;
        }
    }
    return (*a == '\0' && *b == '\0') ? 1u : 0u;
}

static const char *SubBoardMusicExtension(const char *name)
{
    const char *dot = NULL;

    while (*name != '\0')
    {
        if (*name == '.')
        {
            dot = name;
        }
        name++;
    }
    return dot;
}

static void SubBoardMusicDiagLock(void)
{
    k_mutex_lock(&SubBoardMusicDiagMutex, K_FOREVER);
}

static void SubBoardMusicDiagUnlock(void)
{
    k_mutex_unlock(&SubBoardMusicDiagMutex);
}

static void SubBoardMusicSetError(int32_t error)
{
    SubBoardMusicDiagLock();
    SubBoardMusicDiagData.lastError = error;
    SubBoardMusicDiagUnlock();
}

static int SubBoardMusicMount(void)
{
    const int ret = SdcardMount();

    SubBoardMusicDiagLock();
    SubBoardMusicDiagData.mounted = (ret == 0) ? 1u : 0u;
    SubBoardMusicDiagData.lastError = (int32_t)ret;
    SubBoardMusicDiagUnlock();
    return ret;
}

static int SubBoardMusicParseWav(SubBoardMusicTrack *track)
{
    FIL fp;
    UINT got = 0u;
    uint8_t hdr[12];
    uint8_t haveFmt = 0u;
    uint8_t haveData = 0u;
    FSIZE_t fileSize;
    FSIZE_t riffEnd;
    FRESULT fr;

    fr = f_open(&fp, track->path, FA_READ);
    if (fr != FR_OK)
    {
        return -(int)fr;
    }
    fileSize = f_size(&fp);
    fr = f_read(&fp, hdr, sizeof(hdr), &got);
    if (fr != FR_OK || got != sizeof(hdr) || memcmp(hdr, "RIFF", 4u) != 0 || memcmp(&hdr[8], "WAVE", 4u) != 0)
    {
        (void)f_close(&fp);
        return -EINVAL;
    }
    riffEnd = (FSIZE_t)SubBoardMusicLe32(&hdr[4]) + 8u;
    if (riffEnd < sizeof(hdr) || riffEnd > fileSize)
    {
        (void)f_close(&fp);
        return -EINVAL;
    }

    while (haveData == 0u)
    {
        uint8_t chunk[8];
        uint32_t bytes;

        fr = f_read(&fp, chunk, sizeof(chunk), &got);
        if (fr != FR_OK || got != sizeof(chunk))
        {
            (void)f_close(&fp);
            return -EINVAL;
        }
        bytes = SubBoardMusicLe32(&chunk[4]);
        if (f_tell(&fp) > riffEnd || (FSIZE_t)bytes > riffEnd - f_tell(&fp))
        {
            (void)f_close(&fp);
            return -EINVAL;
        }
        if (memcmp(chunk, "fmt ", 4u) == 0)
        {
            uint8_t fmt[16];
            if (bytes < sizeof(fmt) || f_read(&fp, fmt, sizeof(fmt), &got) != FR_OK || got != sizeof(fmt))
            {
                (void)f_close(&fp);
                return -EINVAL;
            }
            if (SubBoardMusicLe16(&fmt[0]) != 1u)
            {
                (void)f_close(&fp);
                return -ENOTSUP;
            }
            const uint16_t channels = SubBoardMusicLe16(&fmt[2]);
            const uint16_t bitsPerSample = SubBoardMusicLe16(&fmt[14]);

            if (channels > UINT8_MAX || bitsPerSample > UINT8_MAX)
            {
                (void)f_close(&fp);
                return -ENOTSUP;
            }
            track->channels = (uint8_t)channels;
            track->sampleRateHz = SubBoardMusicLe32(&fmt[4]);
            track->blockAlign = SubBoardMusicLe16(&fmt[12]);
            track->bitsPerSample = (uint8_t)bitsPerSample;
            if ((track->channels != 1u && track->channels != 2u) ||
                (track->bitsPerSample != 8u && track->bitsPerSample != 16u) || track->sampleRateHz == 0u ||
                track->sampleRateHz > SUB_BOARD_MUSIC_MAX_HZ ||
                track->blockAlign != (uint16_t)(track->channels * (track->bitsPerSample / 8u)))
            {
                (void)f_close(&fp);
                return -ENOTSUP;
            }
            if (bytes > sizeof(fmt) && f_lseek(&fp, f_tell(&fp) + bytes - sizeof(fmt)) != FR_OK)
            {
                (void)f_close(&fp);
                return -EINVAL;
            }
            haveFmt = 1u;
        }
        else if (memcmp(chunk, "data", 4u) == 0)
        {
            if (haveFmt == 0u)
            {
                (void)f_close(&fp);
                return -EINVAL;
            }
            track->dataOffset = f_tell(&fp);
            track->dataBytes = bytes;
            if ((track->dataBytes % track->blockAlign) != 0u)
            {
                (void)f_close(&fp);
                return -EINVAL;
            }
            haveData = 1u;
        }
        else if (f_lseek(&fp, f_tell(&fp) + bytes) != FR_OK)
        {
            (void)f_close(&fp);
            return -EINVAL;
        }
        if ((bytes & 1u) != 0u && haveData == 0u && f_lseek(&fp, f_tell(&fp) + 1u) != FR_OK)
        {
            (void)f_close(&fp);
            return -EINVAL;
        }
    }
    (void)f_close(&fp);
    track->format = SubBoardMusicWavPcm;
    return 0;
}

static int SubBoardMusicAddFile(const char *root, const FILINFO *info)
{
    SubBoardMusicTrack *track;
    const char *ext;
    int n;
    int ret;

    SubBoardMusicDiagLock();
    SubBoardMusicDiagData.entryCount++;
    if (SubBoardMusicDiagData.firstFile[0] == '\0')
    {
        const size_t nameLen = strnlen(info->fname, sizeof(SubBoardMusicDiagData.firstFile) - 1u);

        memcpy(SubBoardMusicDiagData.firstFile, info->fname, nameLen);
        SubBoardMusicDiagData.firstFile[nameLen] = '\0';
    }
    SubBoardMusicDiagUnlock();
    ext = SubBoardMusicExtension(info->fname);
    if (ext == NULL || (SubBoardMusicEqNoCase(ext, ".u8") == 0u && SubBoardMusicEqNoCase(ext, ".wav") == 0u))
    {
        SubBoardMusicDiagLock();
        SubBoardMusicDiagData.unsupportedCount++;
        SubBoardMusicDiagUnlock();
        return 0;
    }
    if (SubBoardMusicTrackCount >= SUB_BOARD_MUSIC_TRACK_MAX)
    {
        return -ENOSPC;
    }
    track = &SubBoardMusicTracks[SubBoardMusicTrackCount];
    memset(track, 0, sizeof(*track));
    n = snprintk(track->path, sizeof(track->path), "%s%s%s", root,
                 root[strlen(root) - 1u] == '/' ? "" : "/", info->fname);
    if (n <= 0 || (size_t)n >= sizeof(track->path))
    {
        return -ENAMETOOLONG;
    }
    /* 显示名可以缩短，打开文件始终使用完整路径；不能因显示名过长丢歌。 */
    n = snprintk(track->name, sizeof(track->name), "%s", info->fname);
    if (n >= (int)sizeof(track->name))
    {
        size_t end = sizeof(track->name) - 1u;
        while (end != 0u && ((uint8_t)info->fname[end] & 0xc0u) == 0x80u)
        {
            end--;
        }
        track->name[end] = '\0';
    }
    if (SubBoardMusicEqNoCase(ext, ".u8") != 0u)
    {
        if (info->fsize > UINT32_MAX)
        {
            return -EFBIG;
        }
        track->format = SubBoardMusicU8;
        track->sampleRateHz = SUB_BOARD_MUSIC_DEFAULT_HZ;
        track->channels = 1u;
        track->bitsPerSample = 8u;
        track->blockAlign = 1u;
        track->dataBytes = info->fsize;
        ret = 0;
    }
    else
    {
        ret = SubBoardMusicParseWav(track);
    }
    if (ret != 0)
    {
        return ret;
    }
    SubBoardMusicTrackCount++;
    /* 普通日志只报告总数，避免长歌名单在启动时挤满日志缓冲。 */
    LOG_DBG("Track %u: %s", SubBoardMusicTrackCount, track->path);
    return 0;
}

static int SubBoardMusicScanDir(const char *root)
{
    FILINFO info;
    FRESULT fr;
    uint32_t depth = 0u;
    int ret = 0;
    SubBoardMusicScanFrame *frame = &SubBoardMusicScanFrames[0];

    (void)snprintk(frame->path, sizeof(frame->path), "%s", root);
    fr = f_opendir(&frame->dir, frame->path);
    if (fr != FR_OK)
    {
        return -(int)fr;
    }
    for (;;)
    {
        frame = &SubBoardMusicScanFrames[depth];
        fr = f_readdir(&frame->dir, &info);
        if (fr != FR_OK)
        {
            ret = -(int)fr;
            break;
        }
        if (info.fname[0] == '\0')
        {
            (void)f_closedir(&frame->dir);
            if (depth == 0u)
            {
                return 0;
            }
            depth--;
            continue;
        }
        if ((info.fattrib & AM_DIR) == 0u)
        {
            int fileRet = SubBoardMusicAddFile(frame->path, &info);
            if (fileRet != 0)
            {
                SubBoardMusicDiagLock();
                SubBoardMusicDiagData.unsupportedCount++;
                SubBoardMusicDiagUnlock();
                LOG_WRN("Skip %s/%s: %d", frame->path, info.fname, fileRet);
            }
        }
        else if (info.fname[0] != '.' && (info.fattrib & (AM_HID | AM_SYS)) == 0u)
        {
            if (depth >= SUB_BOARD_MUSIC_DIR_DEPTH)
            {
                LOG_WRN("Directory depth limit: %s/%s", frame->path, info.fname);
                continue;
            }
            SubBoardMusicScanFrame *next = &SubBoardMusicScanFrames[depth + 1u];
            size_t parentLen = strlen(frame->path);
            size_t nameLen = strlen(info.fname);
            size_t separator = frame->path[parentLen - 1u] == '/' ? 0u : 1u;
            if (parentLen + separator + nameLen >= sizeof(next->path))
            {
                LOG_WRN("Directory path too long: %s", info.fname);
                continue;
            }
            memmove(next->path, frame->path, parentLen);
            if (separator != 0u)
            {
                next->path[parentLen] = '/';
            }
            memcpy(&next->path[parentLen + separator], info.fname, nameLen + 1u);
            fr = f_opendir(&next->dir, next->path);
            if (fr == FR_OK)
            {
                depth++;
            }
            else
            {
                LOG_WRN("Cannot scan %s: %d", next->path, fr);
            }
        }
    }
    do
    {
        (void)f_closedir(&SubBoardMusicScanFrames[depth].dir);
    } while (depth-- != 0u);
    return ret;
}

static int SubBoardMusicScan(void)
{
    int ret;

    SubBoardMusicDiagLock();
    memset(SubBoardMusicTracks, 0, sizeof(SubBoardMusicTracks));
    SubBoardMusicTrackCount = 0u;
    SubBoardMusicDiagData.trackCount = 0u;
    SubBoardMusicDiagData.scanCount++;
    SubBoardMusicDiagData.entryCount = 0u;
    SubBoardMusicDiagData.unsupportedCount = 0u;
    SubBoardMusicDiagData.currentIndex = 0u;
    SubBoardMusicDiagData.currentName[0] = '\0';
    SubBoardMusicDiagData.firstFile[0] = '\0';
    SubBoardMusicDiagUnlock();

    ret = SubBoardMusicScanDir(SUB_BOARD_MUSIC_ROOT);
    SubBoardMusicDiagLock();
    (void)snprintk(SubBoardMusicDiagData.directory, sizeof(SubBoardMusicDiagData.directory), "%s", SUB_BOARD_MUSIC_ROOT);
    SubBoardMusicDiagUnlock();
    if (ret != 0)
    {
        SubBoardMusicSetError(ret);
        return ret;
    }
    SubBoardMusicDiagLock();
    SubBoardMusicDiagData.trackCount = SubBoardMusicTrackCount;
    SubBoardMusicDiagData.lastError = (SubBoardMusicTrackCount != 0u) ? 0 : -ENOENT;
    SubBoardMusicDiagUnlock();
    LOG_INF("Scan complete: %u tracks", SubBoardMusicTrackCount);
    return (SubBoardMusicTrackCount != 0u) ? 0 : -ENOENT;
}

static uint8_t SubBoardMusicReadPin(uint32_t pin)
{
    const int value = gpio_pin_get(SubBoardMusicGpioD, pin);

    return (value > 0) ? 1u : 0u;
}

static void SubBoardMusicPollButton(SubBoardMusicButton *button, uint32_t pin, int8_t direction, uint32_t now)
{
    const uint8_t raw = SubBoardMusicReadPin(pin);

    const uint8_t wasStable = button->stable;
    const uint8_t event = SubBoardMusicButtonUpdate(button, raw, now);

    if (event == 2u)
    {
        SubBoardMusicPlayEnabled = !SubBoardMusicPlayEnabled;
        SubBoardMusicDiagLock();
        SubBoardMusicDiagData.toggleCount++;
        SubBoardMusicDiagUnlock();
    }
    else if (event == 1u)
    {
        SubBoardMusicChange = direction;
    }
    if (wasStable != button->stable)
    {
        if (button->stable == 0u)
        {
            SubBoardMusicDiagLock();
            if (direction < 0)
            {
                SubBoardMusicDiagData.pd14PressedCount++;
            }
            else
            {
                SubBoardMusicDiagData.pd15PressedCount++;
            }
            SubBoardMusicDiagUnlock();
        }
    }
}

static void SubBoardMusicPollButtons(void)
{
    const uint32_t now = k_uptime_get_32();

    SubBoardMusicPollButton(&SubBoardMusicPrev, SUB_BOARD_MUSIC_PREV_PIN, -1, now);
    SubBoardMusicPollButton(&SubBoardMusicNext, SUB_BOARD_MUSIC_NEXT_PIN, 1, now);
    SubBoardMusicDiagLock();
    SubBoardMusicDiagData.pd14Raw = SubBoardMusicPrev.raw;
    SubBoardMusicDiagData.pd15Raw = SubBoardMusicNext.raw;
    SubBoardMusicDiagData.pd14Stable = SubBoardMusicPrev.stable;
    SubBoardMusicDiagData.pd15Stable = SubBoardMusicNext.stable;
    SubBoardMusicDiagData.playEnabled = SubBoardMusicPlayEnabled;
    SubBoardMusicDiagUnlock();
}

static uint32_t SubBoardMusicConvert(const SubBoardMusicTrack *track, uint32_t inputBytes)
{
    uint32_t out = 0u;

    if (track->format == SubBoardMusicU8)
    {
        if (inputBytes > sizeof(SubBoardMusicPcm))
        {
            inputBytes = sizeof(SubBoardMusicPcm);
        }
        memcpy(SubBoardMusicPcm, SubBoardMusicIo, inputBytes);
        return inputBytes;
    }
    for (uint32_t pos = 0u; pos + track->blockAlign <= inputBytes && out < sizeof(SubBoardMusicPcm); pos += track->blockAlign)
    {
        int32_t sample;

        if (track->bitsPerSample == 8u)
        {
            sample = (int32_t)SubBoardMusicIo[pos] - 128;
            if (track->channels == 2u)
            {
                sample += (int32_t)SubBoardMusicIo[pos + 1u] - 128;
                sample /= 2;
            }
            sample += 128;
        }
        else
        {
            sample = (int16_t)SubBoardMusicLe16(&SubBoardMusicIo[pos]);
            if (track->channels == 2u)
            {
                sample += (int16_t)SubBoardMusicLe16(&SubBoardMusicIo[pos + 2u]);
                sample /= 2;
            }
            sample = (sample >> 8) + 128;
        }
        if (sample < 0)
        {
            sample = 0;
        }
        else if (sample > 255)
        {
            sample = 255;
        }
        SubBoardMusicPcm[out++] = (uint8_t)sample;
    }
    return out;
}

static int SubBoardMusicPlay(uint16_t index)
{
    const SubBoardMusicTrack *track = &SubBoardMusicTracks[index];
    FIL fp;
    uint32_t remain = track->dataBytes;
    FRESULT fr;
    int ret = 0;

    fr = f_open(&fp, track->path, FA_READ);
    if (fr != FR_OK)
    {
        return -(int)fr;
    }
    if (track->dataOffset != 0u && f_lseek(&fp, track->dataOffset) != FR_OK)
    {
        (void)f_close(&fp);
        return -EIO;
    }
    BuzzerPcmStop();
    BuzzerSetEnable(1u);
    ret = BuzzerPcmStartStreamU8(track->sampleRateHz, SUB_BOARD_MUSIC_VOLUME);
    if (ret != 0)
    {
        (void)f_close(&fp);
        return ret;
    }
    SubBoardMusicDiagLock();
    SubBoardMusicDiagData.currentIndex = index;
    SubBoardMusicDiagData.sampleRateHz = track->sampleRateHz;
    SubBoardMusicDiagData.channels = track->channels;
    SubBoardMusicDiagData.bitsPerSample = track->bitsPerSample;
    SubBoardMusicDiagData.playing = 1u;
    (void)snprintk(SubBoardMusicDiagData.currentName, sizeof(SubBoardMusicDiagData.currentName), "%s", track->name);
    SubBoardMusicDiagUnlock();

    LOG_INF("Playing %s: %u Hz, %u bit, %u ch", track->name,
            track->sampleRateHz, track->bitsPerSample, track->channels);
    while (SubBoardMusicStopRequested == 0u && SubBoardMusicPlayEnabled != 0u)
    {
        UINT got = 0u;
        uint32_t want;
        uint32_t pcm;
        uint32_t wrote = 0u;

        SubBoardMusicPollButtons();
        if (SubBoardMusicChange != 0 || SubBoardMusicPlayEnabled == 0u)
        {
            break;
        }
        if (BuzzerPcmIsRunning() == 0u || BuzzerPcmIsStreamMode() == 0u)
        {
            ret = -EIO;
            break;
        }
        if (BuzzerPcmStreamGetFree() < sizeof(SubBoardMusicPcm))
        {
            k_msleep(1);
            continue;
        }
        if (remain == 0u)
        {
            BuzzerPcmStreamFinish();
            while (BuzzerPcmStreamGetUsed() != 0u && SubBoardMusicChange == 0 &&
                   SubBoardMusicStopRequested == 0u && SubBoardMusicPlayEnabled != 0u)
            {
                SubBoardMusicPollButtons();
                k_msleep(2);
            }
            break;
        }
        want = remain;
        if (want > sizeof(SubBoardMusicPcm) * track->blockAlign)
        {
            want = sizeof(SubBoardMusicPcm) * track->blockAlign;
        }
        if (want > sizeof(SubBoardMusicIo))
        {
            want = sizeof(SubBoardMusicIo);
        }
        if (track->format == SubBoardMusicWavPcm)
        {
            want -= want % track->blockAlign;
        }
        fr = f_read(&fp, SubBoardMusicIo, want, &got);
        if (fr != FR_OK)
        {
            ret = -(int)fr;
            break;
        }
        if (got == 0u)
        {
            remain = 0u;
            continue;
        }
        remain -= got;
        pcm = SubBoardMusicConvert(track, got);
        while (wrote < pcm && SubBoardMusicStopRequested == 0u &&
               SubBoardMusicChange == 0 && SubBoardMusicPlayEnabled != 0u)
        {
            wrote += BuzzerPcmStreamWriteU8(&SubBoardMusicPcm[wrote], pcm - wrote);
            if (wrote < pcm)
            {
                SubBoardMusicPollButtons();
                k_msleep(1);
            }
        }
        SubBoardMusicDiagLock();
        SubBoardMusicDiagData.pcmQueueUsed = BuzzerPcmStreamGetUsed();
        SubBoardMusicDiagData.pcmQueueFree = BuzzerPcmStreamGetFree();
        SubBoardMusicDiagUnlock();
    }
    (void)f_close(&fp);
    BuzzerPcmStop();
    SubBoardMusicDiagLock();
    SubBoardMusicDiagData.playing = 0u;
    SubBoardMusicDiagData.pcmQueueUsed = 0u;
    SubBoardMusicDiagData.pcmQueueFree = BuzzerPcmStreamGetFree();
    SubBoardMusicDiagUnlock();
    return ret;
}

static void SubBoardMusicThreadEntry(void *a, void *b, void *c)
{
    uint16_t index = 0u;

    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);
    while (SubBoardMusicStopRequested == 0u)
    {
        int ret = SubBoardMusicMount();

        if (ret == 0)
        {
            ret = SubBoardMusicScan();
        }
        if (ret == 0)
        {
            while (SubBoardMusicStopRequested == 0u)
            {
                int8_t change;

                SubBoardMusicPollButtons();
                change = SubBoardMusicChange;
                SubBoardMusicChange = 0;
                if (change < 0)
                {
                    index = (index == 0u) ? (uint16_t)(SubBoardMusicTrackCount - 1u) : (uint16_t)(index - 1u);
                }
                else if (change > 0)
                {
                    index = (uint16_t)((index + 1u) % SubBoardMusicTrackCount);
                }
                SubBoardMusicDiagLock();
                SubBoardMusicDiagData.currentIndex = index;
                (void)snprintk(SubBoardMusicDiagData.currentName, sizeof(SubBoardMusicDiagData.currentName),
                               "%s", SubBoardMusicTracks[index].name);
                SubBoardMusicDiagUnlock();
                if (SubBoardMusicPlayEnabled == 0u)
                {
                    k_msleep(SUB_BOARD_MUSIC_BUTTON_PERIOD_MS);
                    continue;
                }
                ret = SubBoardMusicPlay(index);
                if (SubBoardMusicStopRequested != 0u)
                {
                    break;
                }
                /* 正常播完才自动下一首；手动停止保留当前选择。 */
                if (ret == 0 && SubBoardMusicPlayEnabled != 0u && SubBoardMusicChange == 0)
                {
                    index = (uint16_t)((index + 1u) % SubBoardMusicTrackCount);
                }
                if (ret != 0)
                {
                    LOG_WRN("Playback error: %d", ret);
                    SubBoardMusicSetError(ret);
                    SubBoardMusicPlayEnabled = 0u;
                    k_msleep(100u);
                }
            }
        }
        else
        {
            k_msleep(SUB_BOARD_MUSIC_RETRY_MS);
        }
    }
    BuzzerPcmStop();
    SubBoardMusicDiagLock();
    SubBoardMusicDiagData.running = 0u;
    SubBoardMusicDiagData.playing = 0u;
    SubBoardMusicDiagUnlock();
    SubBoardMusicStarted = 0u;
}

int SubBoardMusicStart(void)
{
    int ret;

    if (SubBoardMusicStarted != 0u)
    {
        return 0;
    }
    if (!device_is_ready(SubBoardMusicGpioD))
    {
        return -ENODEV;
    }
    ret = gpio_pin_configure(SubBoardMusicGpioD, SUB_BOARD_MUSIC_PREV_PIN, GPIO_INPUT | GPIO_PULL_UP);
    if (ret == 0)
    {
        ret = gpio_pin_configure(SubBoardMusicGpioD, SUB_BOARD_MUSIC_NEXT_PIN, GPIO_INPUT | GPIO_PULL_UP);
    }
    if (ret != 0)
    {
        return ret;
    }
    k_mutex_init(&SubBoardMusicDiagMutex);
    memset(&SubBoardMusicDiagData, 0, sizeof(SubBoardMusicDiagData));
    memset(&SubBoardMusicPrev, 0, sizeof(SubBoardMusicPrev));
    memset(&SubBoardMusicNext, 0, sizeof(SubBoardMusicNext));
    SubBoardMusicPrev.raw = SubBoardMusicReadPin(SUB_BOARD_MUSIC_PREV_PIN);
    SubBoardMusicPrev.stable = SubBoardMusicPrev.raw;
    SubBoardMusicNext.raw = SubBoardMusicReadPin(SUB_BOARD_MUSIC_NEXT_PIN);
    SubBoardMusicNext.stable = SubBoardMusicNext.raw;
    SubBoardMusicPrev.changedMs = k_uptime_get_32();
    SubBoardMusicNext.changedMs = SubBoardMusicPrev.changedMs;
    SubBoardMusicStopRequested = 0u;
    SubBoardMusicChange = 0;
    SubBoardMusicPlayEnabled = 0u;
    SubBoardMusicStarted = 1u;
    SubBoardMusicDiagLock();
    SubBoardMusicDiagData.running = 1u;
    SubBoardMusicDiagUnlock();
    (void)k_thread_create(&SubBoardMusicThread, SubBoardMusicStack, K_THREAD_STACK_SIZEOF(SubBoardMusicStack),
                          SubBoardMusicThreadEntry, NULL, NULL, NULL, K_PRIO_PREEMPT(12), 0, K_NO_WAIT);
    k_thread_name_set(&SubBoardMusicThread, "sub_music");
    return 0;
}

void SubBoardMusicStop(void)
{
    SubBoardMusicStopRequested = 1u;
    BuzzerPcmStop();
}

void SubBoardMusicGetDiag(SubBoardMusicDiag *out)
{
    if (out != NULL)
    {
        SubBoardMusicDiagLock();
        *out = SubBoardMusicDiagData;
        SubBoardMusicDiagUnlock();
    }
}
