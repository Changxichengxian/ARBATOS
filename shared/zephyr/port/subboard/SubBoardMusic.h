/* SD 卡歌曲播放和副板按键切歌。 */
#ifndef ARB_SUB_BOARD_MUSIC_H
#define ARB_SUB_BOARD_MUSIC_H

#include <stdint.h>

typedef struct
{
    int32_t lastError;
    uint32_t scanCount;
    uint32_t entryCount;
    uint32_t unsupportedCount;
    uint32_t pd14PressedCount;
    uint32_t pd15PressedCount;
    uint32_t pcmQueueUsed;
    uint32_t pcmQueueFree;
    uint32_t sampleRateHz;
    uint16_t currentIndex;
    uint16_t trackCount;
    uint8_t running;
    uint8_t mounted;
    uint8_t playing;
    uint8_t pd14Raw;
    uint8_t pd15Raw;
    uint8_t pd14Stable;
    uint8_t pd15Stable;
    uint8_t channels;
    uint8_t bitsPerSample;
    char directory[16];
    char firstFile[96];
    char currentName[96];
    uint32_t toggleCount;
    uint8_t playEnabled;
} SubBoardMusicDiag;

/*
 * 启动独立的低优先级音乐线程。需在 BspBuzzerPlatformInit() 与 SD SPI
 * 协议层就绪后调用；可重复调用。
 */
int SubBoardMusicStart(void);
void SubBoardMusicStop(void);
void SubBoardMusicGetDiag(SubBoardMusicDiag *out);

#endif
