#ifndef SUB_BOARD_MUSIC_BUTTON_H
#define SUB_BOARD_MUSIC_BUTTON_H

#include <stdint.h>

typedef struct
{
    uint8_t raw;
    uint8_t stable;
    uint8_t pending;
    uint32_t changedMs;
    uint32_t clickedMs;
} SubBoardMusicButton;

/* 0 无动作，1 单击，2 双击。等双击窗口结束才选歌，避免双击先切歌。 */
static inline uint8_t SubBoardMusicButtonUpdate(SubBoardMusicButton *button, uint8_t raw, uint32_t now)
{
    uint8_t event = 0u;
    if (button->pending && (uint32_t)(now - button->clickedMs) > 350u) {
        button->pending = 0u;
        event = 1u;
    }
    if (raw != button->raw) {
        button->raw = raw;
        button->changedMs = now;
    }
    if (button->stable != raw && (uint32_t)(now - button->changedMs) >= 20u) {
        button->stable = raw;
        if (raw == 0u) {
            if (button->pending) {
                button->pending = 0u;
                event = 2u;
            } else {
                button->pending = 1u;
                button->clickedMs = now;
            }
        }
    }
    return event;
}

#endif
