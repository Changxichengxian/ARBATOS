/* 独立 CAN 测试固件。上电只接收；明确写入测试命令后才操作 ID 1..6。 */
#include <errno.h>
#include <string.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#if defined(CONFIG_UART_CONSOLE)
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#endif

#define BENCH_MAGIC 0x43424d31u
#define BENCH_RETRY_US 5000u
#define BENCH_TIMEOUT_US 100000u
#define BENCH_ROUNDS 10u
#define BENCH_TRIALS (6u * BENCH_ROUNDS * 2u)

#if defined(CONFIG_ARM_ON_ENTER_CPU_IDLE_HOOK)
bool z_arm_on_enter_cpu_idle(void)
{
    /* 仅本测试固件保持 CPU 唤醒；电机计时仍由硬件周期计数完成。 */
    return false;
}
#endif

typedef struct
{
    uint32_t count, cycle, state, raw0, raw1;
} BenchMotor;

typedef struct
{
    uint32_t id, round, command, attempts, accepted, completed, txErrors;
    uint32_t elapsedUs, state, outcome;
} BenchTrial;

typedef struct
{
    uint32_t rate, durationUs, txAttempts, txAccepted, txComplete, txErrors;
    uint32_t rxFrames, rxBitsMin, rxBitsMax, overruns, rxErrors, txBusErrors, busState;
} BenchWindow;

typedef struct
{
    uint32_t magic, ready;
    int32_t initError;
    uint32_t busy, result, trialCount, windowCount, tickMs;
    uint32_t txAttempts, txAccepted, txComplete, txErrors, rxFrames, rxBitsMin, rxBitsMax;
    BenchMotor motors[6];
    BenchTrial trials[BENCH_TRIALS];
    BenchWindow windows[6];
} BenchStatus;

volatile BenchStatus CanBenchDiag = {.magic = BENCH_MAGIC};
BUILD_ASSERT(sizeof(BenchStatus) == 5292u);
const volatile char CanBenchIdentity[] = "ARBATOS_CAN_BENCH_V3_CAN1_1M_ID1_6";
volatile uint32_t CanBenchCommand;
volatile uint32_t CanBenchStop;
static const struct device *const BenchCan = DEVICE_DT_GET(DT_ALIAS(can_primary));
static uint32_t BenchReportRequested;

static void BenchSerialCommand(const char *line)
{
    if (strcmp(line, "CB1 stop") == 0) {
        CanBenchStop = 1u;
    } else if (strcmp(line, "CB1 status") == 0) {
        BenchReportRequested = 1u;
    } else if (CanBenchDiag.ready != 0u && CanBenchDiag.busy == 0u && CanBenchCommand == 0u) {
        uint32_t command = strcmp(line, "CB1 passive") == 0 ? 1u :
                           strcmp(line, "CB1 transitions") == 0 ? 2u :
                           strcmp(line, "CB1 load") == 0 ? 3u :
                           strcmp(line, "CB1 sample") == 0 ? 4u :
                           strcmp(line, "CB1 left") == 0 ? 5u :
                           strcmp(line, "CB1 right") == 0 ? 6u : 0u;
        if (command != 0u) {
            CanBenchStop = 0u;
            CanBenchCommand = command;
        }
    }
}

#if defined(CONFIG_UART_CONSOLE)
static const struct device *const BenchUart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static BenchStatus BenchSerialSnapshot;
static uint8_t BenchRxQueue[128];
static volatile uint32_t BenchRxRead, BenchRxWrite, BenchRxOverflow;

static void BenchUartReceive(const struct device *dev, void *arg)
{
    ARG_UNUSED(arg);
    if (!uart_irq_update(dev)) return;
    while (uart_irq_rx_ready(dev)) {
        uint8_t ch;
        if (uart_fifo_read(dev, &ch, 1) != 1) break;
        if (ch == '!') CanBenchStop = 1u;
        uint32_t next = (BenchRxWrite + 1u) % sizeof(BenchRxQueue);
        if (next == BenchRxRead) {
            BenchRxOverflow = 1u;
        } else {
            BenchRxQueue[BenchRxWrite] = ch;
            BenchRxWrite = next;
        }
    }
}

static void BenchSerialInit(void)
{
    if (device_is_ready(BenchUart)) {
        uart_irq_callback_user_data_set(BenchUart, BenchUartReceive, NULL);
        uart_irq_rx_enable(BenchUart);
    }
}

static void BenchPollSerial(void)
{
    static char line[32];
    static uint32_t used;
    static bool discard;
    for (uint32_t i = 0u; i < 32u; i++) {
        unsigned key = irq_lock();
        if (BenchRxOverflow != 0u) {
            BenchRxRead = BenchRxWrite;
            BenchRxOverflow = 0u;
            used = 0u;
            discard = true;
        }
        if (BenchRxRead == BenchRxWrite) {
            irq_unlock(key);
            break;
        }
        uint8_t ch = BenchRxQueue[BenchRxRead];
        BenchRxRead = (BenchRxRead + 1u) % sizeof(BenchRxQueue);
        irq_unlock(key);
        if (ch == '!') {
            CanBenchStop = 1u;
            used = 0u;
            discard = true;
        } else if (ch == '\n') {
            if (!discard) {
                line[used] = '\0';
                BenchSerialCommand(line);
            }
            used = 0u;
            discard = false;
        } else if (ch != '\r') {
            if (ch < 32u || ch > 126u || used + 1u >= sizeof(line)) discard = true;
            if (!discard) line[used++] = (char)ch;
        }
    }
}

static void BenchSerialReport(uint32_t command)
{
    /* 测试结束后才发送完整报告，避免日志发送时间进入电机响应和负载计时。 */
    unsigned key = irq_lock();
    memcpy(&BenchSerialSnapshot, (const void *)&CanBenchDiag, sizeof(BenchSerialSnapshot));
    irq_unlock(key);
    const uint8_t *raw = (const uint8_t *)&BenchSerialSnapshot;
    uint32_t crc = 0xffffffffu;
    for (uint32_t i = 0u; i < sizeof(BenchSerialSnapshot); i++) {
        crc ^= raw[i];
        for (uint32_t bit = 0u; bit < 8u; bit++) crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
    }
    printk("CB1_BEGIN %u %u %08x\n", command, (unsigned)sizeof(BenchSerialSnapshot), (unsigned)~crc);
    static const char digits[] = "0123456789abcdef";
    for (uint32_t i = 0u; i < sizeof(BenchSerialSnapshot); i++) {
        uart_poll_out(BenchUart, digits[raw[i] >> 4]);
        uart_poll_out(BenchUart, digits[raw[i] & 15u]);
        if (i % 32u == 31u) uart_poll_out(BenchUart, '\n');
    }
    printk("\nCB1_END\n");
}

static void BenchSerialStatus(void)
{
    printk("CB1_STATUS ARBATOS_CAN_BENCH_V3_CAN1_1M_ID1_6 ready=%u init=%d busy=%u tx=%u rx=%u tick=%u\n",
           (unsigned)CanBenchDiag.ready, (int)CanBenchDiag.initError, (unsigned)CanBenchDiag.busy,
           (unsigned)CanBenchDiag.txComplete, (unsigned)CanBenchDiag.rxFrames, (unsigned)CanBenchDiag.tickMs);
}
#else
static void BenchSerialInit(void) {}
static void BenchPollSerial(void) {}
static void BenchSerialReport(uint32_t command) { ARG_UNUSED(command); }
static void BenchSerialStatus(void) {}
#endif

static uint32_t BenchElapsed(uint32_t start)
{
    return (uint32_t)k_cyc_to_us_floor64((uint32_t)(k_cycle_get_32() - start));
}

static void BenchReceive(const struct device *dev, struct can_frame *frame, void *arg)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(arg);
    if ((frame->flags & (CAN_FRAME_IDE | CAN_FRAME_RTR | CAN_FRAME_FDF)) != 0u || frame->dlc > 8u) {
        return;
    }
    CanBenchDiag.rxFrames++;
    CanBenchDiag.rxBitsMin += 47u + frame->dlc * 8u;
    CanBenchDiag.rxBitsMax += 47u + frame->dlc * 8u + (33u + frame->dlc * 8u) / 4u;
    uint32_t id = frame->data[0] & 15u;
    if (frame->dlc != 8u || id < 1u || id > 6u || frame->id != id) {
        return;
    }
    volatile BenchMotor *motor = &CanBenchDiag.motors[id - 1u];
    motor->cycle = k_cycle_get_32();
    motor->state = frame->data[0] >> 4;
    memcpy((void *)&motor->raw0, frame->data, 8u);
    __sync_synchronize();
    motor->count++;
}

static void BenchTxDone(const struct device *dev, int error, void *arg)
{
    ARG_UNUSED(dev);
    volatile BenchTrial *trial = arg;
    if (error == 0) {
        CanBenchDiag.txComplete++;
        if (trial != NULL) trial->completed++;
    } else {
        CanBenchDiag.txErrors++;
        if (trial != NULL) trial->txErrors++;
    }
}

static int BenchSend(uint32_t id, uint8_t command, volatile BenchTrial *trial)
{
    if (id < 1u || id > 6u || (command != 0xfcu && command != 0xfdu && command != 0u)) {
        return -EINVAL;
    }
    struct can_frame frame = {.id = id, .dlc = 8u};
    if (command != 0u) {
        memset(frame.data, 0xff, 8u);
        frame.data[7] = command;
    } else {
        /* 清除旧目标：MIT 的位置/速度/力矩编码零值，kp=kd=0；不写电机零位。 */
        static const uint8_t neutral[8] = {0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00};
        memcpy(frame.data, neutral, sizeof(neutral));
    }
    CanBenchDiag.txAttempts++;
    if (trial != NULL) trial->attempts++;
    int ret = can_send(BenchCan, &frame, K_NO_WAIT, BenchTxDone, (void *)trial);
    if (ret == 0) {
        CanBenchDiag.txAccepted++;
        if (trial != NULL) trial->accepted++;
    } else {
        CanBenchDiag.txErrors++;
        if (trial != NULL) trial->txErrors++;
    }
    return ret;
}

static uint32_t BenchTransition(uint32_t id, uint8_t command, volatile BenchTrial *trial, bool stopping)
{
    volatile BenchMotor *motor = &CanBenchDiag.motors[id - 1u];
    uint32_t before = motor->count;
    uint32_t start = k_cycle_get_32();
    uint32_t nextSend = 0u;
    uint32_t accepted = 0u;
    uint32_t expected = command == 0xfcu ? 1u : 0u;
    while (BenchElapsed(start) < BENCH_TIMEOUT_US) {
        BenchPollSerial();
        uint32_t elapsed = BenchElapsed(start);
        if (CanBenchStop != 0u && !stopping) break;
        if (elapsed >= nextSend) {
            if (BenchSend(id, command, trial) == 0) accepted++;
            nextSend = elapsed + BENCH_RETRY_US;
        }
        unsigned key = irq_lock();
        BenchMotor sample = *motor;
        irq_unlock(key);
        /* 只认命令之后的目标 ID 状态反馈；CAN ACK 不等于电机状态成功。 */
        if (accepted != 0u && sample.count != before && sample.state == expected &&
            (int32_t)(sample.cycle - start) >= 0) {
            if (trial != NULL) {
                trial->elapsedUs = (uint32_t)k_cyc_to_us_floor64((uint32_t)(sample.cycle - start));
                trial->state = sample.state;
                trial->outcome = 1u;
            }
            return 1u;
        }
        if (sample.count != before && sample.state >= 8u && command == 0xfcu) break;
        k_usleep(200);
    }
    if (trial != NULL) {
        trial->elapsedUs = BenchElapsed(start);
        trial->state = motor->state;
        trial->outcome = 2u;
    }
    return 0u;
}

static uint32_t BenchDisableAll(void)
{
    uint32_t confirmed = 0u;
    for (uint32_t id = 1u; id <= 6u; id++) {
        confirmed += BenchTransition(id, 0xfdu, NULL, true);
    }
    return confirmed == 6u;
}

static int BenchWheelVelocity(uint32_t id, bool moving)
{
    if (id != 5u && id != 6u) {
        return -EINVAL;
    }
    /* 仅用于已核对的 H3510：V=280 rad/s，T=1 Nm。kp=0、kd 约 0.03。 */
    uint32_t velocity = moving ? (id == 6u ? 2053u : 2042u) : 2048u;
    uint32_t damping = moving ? 25u : 0u;
    struct can_frame frame = {.id = id, .dlc = 8u};
    frame.data[0] = 0x80u;
    frame.data[1] = 0u;
    frame.data[2] = (uint8_t)(velocity >> 4);
    frame.data[3] = (uint8_t)(velocity << 4);
    frame.data[4] = 0u;
    frame.data[5] = (uint8_t)(damping >> 4);
    frame.data[6] = (uint8_t)((damping << 4) | 8u);
    frame.data[7] = 0u;
    CanBenchDiag.txAttempts++;
    int ret = can_send(BenchCan, &frame, K_NO_WAIT, BenchTxDone, NULL);
    if (ret == 0) {
        CanBenchDiag.txAccepted++;
    } else {
        CanBenchDiag.txErrors++;
    }
    return ret;
}

static void BenchJogWheel(uint32_t id)
{
    /* 所有退出路径由 BenchRun 失能收尾；关节 ID 不允许进入此函数。 */
    if ((id != 5u && id != 6u) || CanBenchStop != 0u) {
        CanBenchStop = 1u;
        return;
    }
    if (BenchWheelVelocity(id, false) != 0) {
        CanBenchStop = 1u;
        return;
    }
    k_msleep(5);
    if (!BenchTransition(id, 0xfcu, NULL, false)) {
        CanBenchStop = 1u;
        return;
    }
    unsigned key = irq_lock();
    BenchMotor initial = CanBenchDiag.motors[id - 1u];
    irq_unlock(key);
    uint8_t raw[8];
    memcpy(raw, &initial.raw0, sizeof(raw));
    int32_t startPosition = ((uint32_t)raw[1] << 8) | raw[2];
    uint32_t start = k_cycle_get_32();
    uint32_t next = 0u;
    while (BenchElapsed(start) < 1000000u && CanBenchStop == 0u) {
        BenchPollSerial();
        key = irq_lock();
        BenchMotor sample = CanBenchDiag.motors[id - 1u];
        bool othersDisabled = true;
        for (uint32_t i = 1u; i <= 6u; i++) {
            if (i != id && CanBenchDiag.motors[i - 1u].state != 0u) othersDisabled = false;
        }
        irq_unlock(key);
        memcpy(raw, &sample.raw0, sizeof(raw));
        int32_t delta = (int32_t)(((uint32_t)raw[1] << 8) | raw[2]) - startPosition;
        if (delta > 32767) delta -= 65536;
        if (delta < -32768) delta += 65536;
        uint32_t speed = ((uint32_t)raw[3] << 4) | (raw[4] >> 4);
        /* 反馈超过 50 ms、速度约超过 2.5 rad/s、位移超过约 1 rad 均立即退出。 */
        if (!othersDisabled || sample.state != 1u || BenchElapsed(sample.cycle) > 50000u ||
            speed < 2030u || speed > 2065u || delta < -2621 || delta > 2621) {
            CanBenchStop = 1u;
            break;
        }
        uint32_t elapsed = BenchElapsed(start);
        if (CanBenchStop != 0u) break;
        if (elapsed >= next) {
            if (BenchWheelVelocity(id, true) != 0) {
                CanBenchStop = 1u;
                break;
            }
            next = elapsed + 5000u;
        }
        k_usleep(200);
    }
    BenchWheelVelocity(id, false);
    if (!BenchTransition(id, 0xfdu, NULL, true)) {
        CanBenchStop = 1u;
    }
}

static void BenchMeasureWindow(uint32_t rate, uint32_t durationMs)
{
    if (CanBenchDiag.windowCount >= 6u) return;
    volatile BenchWindow *window = &CanBenchDiag.windows[CanBenchDiag.windowCount];
    unsigned key = irq_lock();
    uint32_t before[] = {CanBenchDiag.txAttempts, CanBenchDiag.txAccepted, CanBenchDiag.txComplete,
                        CanBenchDiag.txErrors, CanBenchDiag.rxFrames,
                        CanBenchDiag.rxBitsMin, CanBenchDiag.rxBitsMax};
    irq_unlock(key);
    uint32_t overruns = can_stats_get_rx_overruns(BenchCan);
    uint32_t start = k_cycle_get_32();
    uint32_t next = 0u, id = 1u;
    while (BenchElapsed(start) < durationMs * 1000u && CanBenchStop == 0u) {
        BenchPollSerial();
        uint32_t elapsed = BenchElapsed(start);
        if (rate != 0u && elapsed >= next) {
            BenchSend(id, 0xfdu, NULL);
            id = id == 6u ? 1u : id + 1u;
            next = elapsed + 1000000u / rate;
        }
        k_usleep(rate > 1000u ? 100 : 200);
    }
    window->rate = rate;
    window->durationUs = BenchElapsed(start);
    key = irq_lock();
    window->txAttempts = CanBenchDiag.txAttempts - before[0];
    window->txAccepted = CanBenchDiag.txAccepted - before[1];
    window->txComplete = CanBenchDiag.txComplete - before[2];
    window->txErrors = CanBenchDiag.txErrors - before[3];
    window->rxFrames = CanBenchDiag.rxFrames - before[4];
    window->rxBitsMin = CanBenchDiag.rxBitsMin - before[5];
    window->rxBitsMax = CanBenchDiag.rxBitsMax - before[6];
    irq_unlock(key);
    window->overruns = can_stats_get_rx_overruns(BenchCan) - overruns;
    enum can_state state;
    struct can_bus_err_cnt errors;
    if (can_get_state(BenchCan, &state, &errors) == 0) {
        window->busState = state;
        window->rxErrors = errors.rx_err_cnt;
        window->txBusErrors = errors.tx_err_cnt;
    }
    CanBenchDiag.windowCount++;
}

static void BenchRun(uint32_t command)
{
    CanBenchDiag.result = 0u;
    CanBenchDiag.trialCount = 0u;
    CanBenchDiag.windowCount = 0u;
    if (command == 1u) {
        CanBenchDiag.windowCount = 0u;
        BenchMeasureWindow(0u, 5000u);
        CanBenchDiag.result = CanBenchStop == 0u ? 1u : 2u;
        return;
    }
    if (!BenchDisableAll()) {
        CanBenchDiag.result = 3u;
        return;
    }
    if (command == 4u) {
        CanBenchDiag.result = 1u;
        return;
    }
    if (command == 2u) {
        CanBenchDiag.trialCount = 0u;
        memset((void *)CanBenchDiag.trials, 0, sizeof(CanBenchDiag.trials));
        for (uint32_t id = 1u; id <= 6u && CanBenchStop == 0u; id++) {
            for (uint32_t round = 1u; round <= BENCH_ROUNDS && CanBenchStop == 0u; round++) {
                if (BenchSend(id, 0u, NULL) != 0) { CanBenchStop = 1u; break; }
                k_msleep(5);
                for (uint32_t phase = 0u; phase < 2u; phase++) {
                    volatile BenchTrial *trial = &CanBenchDiag.trials[CanBenchDiag.trialCount];
                    trial->id = id;
                    trial->round = round;
                    trial->command = phase == 0u ? 0xfcu : 0xfdu;
                    uint32_t ok = BenchTransition(id, trial->command, trial, phase != 0u);
                    CanBenchDiag.trialCount++;
                    if (!ok) CanBenchStop = 1u;
                }
                k_msleep(20);
            }
        }
    } else if (command == 5u || command == 6u) {
        BenchJogWheel(command == 5u ? 6u : 5u);
    } else if (command == 3u) {
        static const uint32_t rates[] = {100u, 500u, 1000u, 2000u, 3000u};
        CanBenchDiag.windowCount = 0u;
        for (uint32_t i = 0u; i < ARRAY_SIZE(rates) && CanBenchStop == 0u; i++) {
            BenchMeasureWindow(rates[i], 2000u);
            k_msleep(50);
        }
    }
    /* 测试完成或超时返回都补发失能；结果 3 表示仍缺少失能确认，不冒充成功。 */
    uint32_t stopped = BenchDisableAll();
    CanBenchDiag.result = !stopped ? 3u : CanBenchStop != 0u ? 2u : 1u;
}

int main(void)
{
    BenchSerialInit();
    int ret = device_is_ready(BenchCan) ? 0 : -ENODEV;
    if (CanBenchIdentity[0] != 'A') ret = -EINVAL;
    if (ret == 0) ret = can_set_mode(BenchCan, CAN_MODE_NORMAL);
    if (ret == 0) ret = can_set_bitrate(BenchCan, 1000000u);
    const struct can_filter filter = {.id = 0u, .mask = 0u};
    if (ret == 0) {
        ret = can_add_rx_filter(BenchCan, BenchReceive, NULL, &filter);
        if (ret >= 0) ret = can_start(BenchCan);
    }
    CanBenchDiag.initError = ret;
    CanBenchDiag.ready = ret == 0;
    uint32_t lastStatus = 0u;
    BenchSerialStatus();
    for (;;) {
        CanBenchDiag.tickMs = k_uptime_get_32();
        BenchPollSerial();
        uint32_t command = CanBenchCommand;
        if (ret == 0 && command >= 1u && command <= 6u) {
            CanBenchDiag.busy = command;
            CanBenchCommand = 0u;
            BenchRun(command);
            __sync_synchronize();
            CanBenchDiag.busy = 0u;
            BenchSerialReport(command);
        }
        if (BenchReportRequested != 0u) {
            BenchReportRequested = 0u;
            BenchSerialReport(0u);
        }
        if ((uint32_t)(CanBenchDiag.tickMs - lastStatus) >= 1000u) {
            lastStatus = CanBenchDiag.tickMs;
            BenchSerialStatus();
        }
        k_msleep(10);
    }
}
