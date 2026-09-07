#include <assert.h>
#include <stdio.h>
#include "BatteryAlarmPolicy.h"
#include "SubBoardMusicButton.h"

static void CheckAlarm(void)
{
    BatteryAlarmState s = {0};
    /* 两次短时压降之间恢复正常，不能拼接成连续低压。 */
    for (unsigned repeat = 0; repeat < 2; repeat++) {
        for (unsigned i = 0; i < 29; i++) {
            assert(!BatteryAlarmUpdate(&s, 1, 1, 21.0f, 21.6f, 3000, 100));
        }
        assert(!BatteryAlarmUpdate(&s, 1, 1, 22.0f, 21.6f, 3000, 100));
    }
    for (unsigned i = 0; i < 30; i++) {
        assert(!BatteryAlarmUpdate(&s, 1, 1, 21.0f, 21.6f, 3000, 100));
    }
    assert(BatteryAlarmUpdate(&s, 1, 1, 21.0f, 21.6f, 3000, 100));
    assert(BatteryAlarmUpdate(&s, 1, 1, 22.0f, 21.6f, 3000, 100));
    assert(!BatteryAlarmUpdate(&s, 1, 1, 22.2f, 21.6f, 3000, 100));
    for (unsigned i = 0; i < 100; i++) {
        assert(!BatteryAlarmUpdate(&s, 0, 0, 0.0f, 21.6f, 3000, 100));
    }
    for (unsigned i = 0; i < 30; i++) {
        assert(!BatteryAlarmUpdate(&s, 1, 0, 0.0f, 21.6f, 3000, 100));
    }
    assert(BatteryAlarmUpdate(&s, 1, 0, 0.0f, 21.6f, 3000, 100));
    assert(!BatteryAlarmUpdate(&s, 0, 0, 0.0f, 21.6f, 3000, 100));
}

static void CheckButton(void)
{
    SubBoardMusicButton b = {.raw = 1, .stable = 1};
    assert(!SubBoardMusicButtonUpdate(&b, 0, 10));
    assert(!SubBoardMusicButtonUpdate(&b, 1, 15));
    assert(!SubBoardMusicButtonUpdate(&b, 1, 40));
    assert(!SubBoardMusicButtonUpdate(&b, 0, 100));
    assert(!SubBoardMusicButtonUpdate(&b, 0, 120));
    assert(!SubBoardMusicButtonUpdate(&b, 1, 150));
    assert(!SubBoardMusicButtonUpdate(&b, 1, 170));
    assert(!SubBoardMusicButtonUpdate(&b, 0, 250));
    assert(SubBoardMusicButtonUpdate(&b, 0, 270) == 2);
    assert(!SubBoardMusicButtonUpdate(&b, 0, 1000));
    assert(!SubBoardMusicButtonUpdate(&b, 1, 1010));
    assert(!SubBoardMusicButtonUpdate(&b, 1, 1030));
    assert(!SubBoardMusicButtonUpdate(&b, 0, 1100));
    assert(!SubBoardMusicButtonUpdate(&b, 0, 1120));
    assert(!SubBoardMusicButtonUpdate(&b, 1, 1150));
    assert(!SubBoardMusicButtonUpdate(&b, 1, 1170));
    assert(!SubBoardMusicButtonUpdate(&b, 1, 1470));
    assert(SubBoardMusicButtonUpdate(&b, 1, 1471) == 1);
    assert(!SubBoardMusicButtonUpdate(&b, 1, 2000));
    b = (SubBoardMusicButton){.raw = 0, .stable = 0, .pending = 1, .clickedMs = 0xffffff00u};
    assert(SubBoardMusicButtonUpdate(&b, 0, 100) == 1);
    assert(!SubBoardMusicButtonUpdate(&b, 0, 1000));
}

int main(void)
{
    CheckAlarm();
    CheckButton();
    puts("PASS: low voltage dwell/hysteresis/disable/invalid input; button bounce/single/double/hold/timer wrap");
    return 0;
}
