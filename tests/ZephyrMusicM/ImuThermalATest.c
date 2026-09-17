#include <assert.h>
#include "ImuThermalA.h"

int main(void)
{
    ImuThermalA s = {0};
    for (uint32_t ms = 0; ms <= 60000; ms += 100) {
        ImuThermalAUpdate(&s, 25.0f, ms, 5000.0f);
        assert(s.pwm <= 3000 && !s.stable);
    }
    ImuThermalAUpdate(&s, 49.1f, 60001, 5000.0f);
    assert(!s.pwm && !s.stable);
    ImuThermalAUpdate(&s, NAN, 60002, 5000.0f);
    assert(!s.pwm && !s.initialized && !s.stable);
    for (uint32_t ms = 0; ms <= 19900; ms += 100) {
        ImuThermalAUpdate(&s, 48.0f, ms, 5000.0f);
        assert(!s.stable);
    }
    ImuThermalAUpdate(&s, 48.0f, 20000, 5000.0f);
    assert(s.stable);
    /* 刚离开稳温范围就取消资格，不能沿用上个窗口结果。 */
    ImuThermalAUpdate(&s, 47.0f, 20100, 5000.0f);
    assert(!s.stable);
    s = (ImuThermalA){0};
    for (uint32_t ms = 0; ms <= 60000; ms += 100) {
        ImuThermalAUpdate(&s, 30.0f, ms, 100.0f);
        assert(s.pwm <= 100);
    }
    return 0;
}
