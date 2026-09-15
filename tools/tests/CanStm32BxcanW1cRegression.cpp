/* bxCAN 发送完成标志与邮箱复用的主机回归。 */

typedef unsigned int uint32_t;
typedef unsigned char uint8_t;
#define NULL nullptr
extern "C" int puts(const char *text);

#define CAN_TSR_RQCP0 (1u << 0)
#define CAN_TSR_TXOK0 (1u << 1)
#define CAN_TSR_ALST0 (1u << 2)
#define CAN_TSR_TERR0 (1u << 3)
#define CAN_TSR_RQCP1 (1u << 8)
#define CAN_TSR_TXOK1 (1u << 9)
#define CAN_TSR_ALST1 (1u << 10)
#define CAN_TSR_TERR1 (1u << 11)
#define CAN_TSR_RQCP2 (1u << 16)
#define CAN_TSR_TXOK2 (1u << 17)
#define CAN_TSR_ALST2 (1u << 18)
#define CAN_TSR_TERR2 (1u << 19)
#define CAN_TSR_TME   (7u << 26)
#define CAN_TSR_TME0  (1u << 26)
#define CAN_TSR_TME1  (1u << 27)
#define CAN_TSR_TME2  (1u << 28)
#define CAN_ESR_BOFF  (1u << 2)
#define CAN_TI0R_TXRQ 1u
#define CAN_TI0R_IDE  (1u << 2)
#define CAN_TI0R_EXID_Pos 3u
#define CAN_TI0R_STID_Pos 21u
#define CAN_TI1R_RTR (1u << 1)
#define CAN_TDT1R_DLC_Pos 0u
#define CAN_FRAME_IDE 1u
#define CAN_FRAME_RTR 2u
#define CAN_MAX_DLC 8u
#define EIO           5
#define EBUSY         16
#define ENETUNREACH   101
#define ENETDOWN      100
#define EINVAL        22
#define ENOTSUP       95
#define EAGAIN        11
#define K_FOREVER     0
#define LOG_DBG(...)  do { } while (0)
#define LOG_ERR(...)  do { } while (0)

class W1cTsr
{
public:
    W1cTsr() : pending(0u), injectBeforeClear(0u) {}

    operator uint32_t() const
    {
        return pending;
    }

    W1cTsr &operator=(uint32_t value)
    {
        InjectIfNeeded();
        ClearCompletedMailboxes(value);
        return *this;
    }

    W1cTsr &operator|=(uint32_t value)
    {
        InjectIfNeeded();
        ClearCompletedMailboxes(pending | value);
        return *this;
    }

    void Reset(uint32_t value)
    {
        pending = value;
        injectBeforeClear = 0u;
    }

    void InjectBeforeNextClear(uint32_t value)
    {
        injectBeforeClear = value;
    }

    uint32_t Value() const
    {
        return pending;
    }

private:
    void InjectIfNeeded()
    {
        pending |= injectBeforeClear;
        injectBeforeClear = 0u;
    }

    void ClearCompletedMailboxes(uint32_t value)
    {
        if ((value & CAN_TSR_RQCP0) != 0u) {
            pending &= ~(CAN_TSR_RQCP0 | CAN_TSR_TXOK0 |
                         CAN_TSR_ALST0 | CAN_TSR_TERR0);
        }
        if ((value & CAN_TSR_RQCP1) != 0u) {
            pending &= ~(CAN_TSR_RQCP1 | CAN_TSR_TXOK1 |
                         CAN_TSR_ALST1 | CAN_TSR_TERR1);
        }
        if ((value & CAN_TSR_RQCP2) != 0u) {
            pending &= ~(CAN_TSR_RQCP2 | CAN_TSR_TXOK2 |
                         CAN_TSR_ALST2 | CAN_TSR_TERR2);
        }
    }

    uint32_t pending;
    uint32_t injectBeforeClear;
};

struct CAN_TxMailBox_TypeDef
{
    uint32_t TIR;
    uint32_t TDTR;
    uint32_t TDLR;
    uint32_t TDHR;
};

struct CAN_TypeDef
{
    W1cTsr TSR;
    uint32_t ESR;
    CAN_TxMailBox_TypeDef sTxMailBox[3];
};

struct k_sem
{
    int given;
};

struct k_mutex
{
    int held;
};

typedef int k_timeout_t;

static unsigned int IrqDepth;
static void (*DeferredIrq)(void);
static int MutexDepth;

static unsigned int irq_lock()
{
    const unsigned int key = IrqDepth;
    IrqDepth++;
    return key;
}

static void irq_unlock(unsigned int key)
{
    IrqDepth = key;
    if (IrqDepth == 0u && DeferredIrq != nullptr) {
        void (*handler)(void) = DeferredIrq;
        DeferredIrq = nullptr;
        handler();
    }
}

struct device;
typedef void (*can_tx_callback_t)(const struct device *dev, int status,
                                  void *user_data);
static void (*CallbackWriteHook)(void);

class CallbackCell
{
public:
    CallbackCell() : value(nullptr) {}

    operator can_tx_callback_t() const
    {
        return value;
    }

    CallbackCell &operator=(can_tx_callback_t callback)
    {
        value = callback;
        if (CallbackWriteHook != nullptr) {
            void (*hook)(void) = CallbackWriteHook;
            CallbackWriteHook = nullptr;
            if (IrqDepth != 0u) {
                DeferredIrq = hook;
            } else {
                hook();
            }
        }
        return *this;
    }

private:
    can_tx_callback_t value;
};

struct can_stm32_mailbox
{
    CallbackCell tx_callback;
    void *callback_arg;
};

struct can_driver_data
{
    bool started;
};

struct can_stm32_data
{
    can_driver_data common;
    k_mutex inst_mutex;
    k_sem tx_int_sem;
    can_stm32_mailbox mb0;
    can_stm32_mailbox mb1;
    can_stm32_mailbox mb2;
};

struct can_stm32_config
{
    CAN_TypeDef *can;
};

struct device
{
    can_stm32_data *data;
    const can_stm32_config *config;
    const char *name;
};

struct can_frame
{
    uint32_t id;
    uint8_t flags;
    uint8_t dlc;
    uint32_t data_32[2];
};

static void k_mutex_lock(struct k_mutex *mutex, int timeout)
{
    (void)timeout;
    mutex->held++;
    MutexDepth++;
}

static void k_mutex_unlock(struct k_mutex *mutex)
{
    mutex->held--;
    MutexDepth--;
}

static void k_sem_give(struct k_sem *sem)
{
    sem->given++;
}

static int k_sem_take(struct k_sem *sem, int timeout)
{
    (void)sem;
    (void)timeout;
    if (IrqDepth != 0u || MutexDepth != 0) {
        __builtin_trap();
    }
    return -EAGAIN;
}

/* TestCanStm32BxcanW1c.ps1 从实际驱动提取的函数。 */
#include "CanStm32BxcanW1cActual.inc"

static void Require(bool condition, const char *message)
{
    if (!condition) {
        (void)puts(message);
        __builtin_trap();
    }
}

struct CallbackRecord
{
    int count;
    int status;
};

static void RecordCallback(const struct device *dev, int status, void *userData)
{
    (void)dev;
    CallbackRecord *record = static_cast<CallbackRecord *>(userData);
    record->count++;
    record->status = status;
}

struct Fixture
{
    CAN_TypeDef can = {};
    can_stm32_data data = {};
    can_stm32_config config = {&can};
    device dev = {&data, &config, "test"};
    CallbackRecord callbacks[3] = {};

    Fixture()
    {
        data.common.started = true;
    }

    void Reset(uint32_t tsr, bool busOff)
    {
        can.TSR.Reset(tsr);
        can.ESR = busOff ? CAN_ESR_BOFF : 0u;
        data.tx_int_sem.given = 0;
        for (int index = 0; index < 3; index++) {
            callbacks[index] = {};
        }
        data.mb0.tx_callback = RecordCallback;
        data.mb0.callback_arg = &callbacks[0];
        data.mb1.tx_callback = RecordCallback;
        data.mb1.callback_arg = &callbacks[1];
        data.mb2.tx_callback = RecordCallback;
        data.mb2.callback_arg = &callbacks[2];
    }
};

static void TestSingleMailbox()
{
    Fixture fixture;

    fixture.Reset(CAN_TSR_RQCP2 | CAN_TSR_TXOK2, false);
    CanStm32TxIsrHandlerFixed(&fixture.dev);
    Require(fixture.callbacks[0].count == 0 &&
                fixture.callbacks[1].count == 0 &&
                fixture.callbacks[2].count == 1 &&
                fixture.can.TSR.Value() == 0u,
            "单邮箱完成必须只回调一次并清除对应状态位");
}

static void TestMultipleMailboxes()
{
    Fixture original;
    Fixture fixed;
    const uint32_t completed = CAN_TSR_RQCP0 | CAN_TSR_TXOK0 |
                               CAN_TSR_RQCP1 | CAN_TSR_TXOK1 |
                               CAN_TSR_RQCP2 | CAN_TSR_TXOK2;

    original.Reset(completed, false);
    fixed.Reset(completed, false);
    CanStm32TxIsrHandlerOriginal(&original.dev);
    CanStm32TxIsrHandlerFixed(&fixed.dev);
    Require(original.callbacks[0].count == 1 &&
                original.callbacks[1].count == 0 &&
                original.callbacks[2].count == 0,
            "原 RMW 写法会清除其他完成位并漏掉其回调");
    Require(fixed.callbacks[0].count == 1 &&
                fixed.callbacks[1].count == 1 &&
                fixed.callbacks[2].count == 1 &&
                fixed.can.TSR.Value() == 0u,
            "精确 W1C 写入必须使三个邮箱各回调一次");
}

static void TestCompletionArrivingDuringIsr()
{
    Fixture original;
    Fixture fixed;

    original.Reset(CAN_TSR_RQCP0 | CAN_TSR_TXOK0, false);
    fixed.Reset(CAN_TSR_RQCP0 | CAN_TSR_TXOK0, false);
    original.can.TSR.InjectBeforeNextClear(CAN_TSR_RQCP1 | CAN_TSR_TXOK1);
    fixed.can.TSR.InjectBeforeNextClear(CAN_TSR_RQCP1 | CAN_TSR_TXOK1);
    CanStm32TxIsrHandlerOriginal(&original.dev);
    CanStm32TxIsrHandlerFixed(&fixed.dev);
    Require(original.callbacks[0].count == 1 &&
                original.callbacks[1].count == 0,
            "原 RMW 写法会清除 ISR 中途到达的完成位");
    Require(fixed.callbacks[0].count == 1 &&
                fixed.callbacks[1].count == 1,
            "精确 W1C 写入必须保留 ISR 中途到达的完成位");
}

static void TestBusOff()
{
    Fixture fixture;

    fixture.Reset(0u, true);
    CanStm32TxIsrHandlerFixed(&fixture.dev);
    Require(fixture.callbacks[0].count == 1 &&
                fixture.callbacks[1].count == 1 &&
                fixture.callbacks[2].count == 1 &&
                fixture.callbacks[0].status == -ENETUNREACH &&
                fixture.callbacks[1].status == -ENETUNREACH &&
                fixture.callbacks[2].status == -ENETUNREACH,
            "总线关闭时三个邮箱必须各交付一次失败回调");
}

static can_frame TestFrame()
{
    can_frame frame = {};
    frame.id = 0x200u;
    frame.dlc = 8u;
    return frame;
}

static void TestOldCompletionAfterSendSetup()
{
    Fixture original;
    Fixture fixed;
    CallbackRecord originalNew = {};
    CallbackRecord fixedNew = {};
    const uint32_t oldCompletion = CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0;
    const can_frame frame = TestFrame();

    original.Reset(oldCompletion, false);
    fixed.Reset(oldCompletion, false);
    Require(CanStm32SendOriginal(&original.dev, &frame, K_FOREVER,
                                 RecordCallback, &originalNew) == 0,
            "原发送函数必须接受硬件已空的邮箱");
    Require(CanStm32SendFixed(&fixed.dev, &frame, K_FOREVER,
                              RecordCallback, &fixedNew) == -EAGAIN,
            "修正发送函数必须保留旧完成尚未交付的邮箱资格");
    CanStm32TxIsrHandlerOriginal(&original.dev);
    Require(originalNew.count == 1 && original.callbacks[0].count == 0,
            "原发送函数会把旧完成错交付给新请求");
}

static Fixture *InjectedFixture;

static void InjectOldCompletionIrq()
{
    CanStm32TxIsrHandlerOriginal(&InjectedFixture->dev);
}

static void TestIrqBetweenCallbackAndArgument()
{
    Fixture fixture;
    CallbackRecord newRecord = {};
    const can_frame frame = TestFrame();

    fixture.Reset(CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0, false);
    InjectedFixture = &fixture;
    CallbackWriteHook = InjectOldCompletionIrq;
    Require(CanStm32SendOriginal(&fixture.dev, &frame, K_FOREVER,
                                 RecordCallback, &newRecord) == 0,
            "原发送函数必须进入邮箱回调赋值窗口");
    Require(fixture.callbacks[0].count == 1 && newRecord.count == 0,
            "夹在两次赋值之间的 IRQ 会用旧参数调用新回调");
    fixture.can.TSR.Reset(CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0);
    CanStm32TxIsrHandlerOriginal(&fixture.dev);
    Require(newRecord.count == 0,
            "被旧 IRQ 清空的回调不会收到新帧完成通知");
    CallbackWriteHook = nullptr;
}

static int DeferredIrqDelivered;
static CallbackRecord *ExpectedNewRecord;

static void InjectFixedIrq()
{
    Require(IrqDepth == 0u &&
                InjectedFixture->data.mb0.callback_arg == ExpectedNewRecord &&
                InjectedFixture->data.mb0.tx_callback != nullptr &&
                (InjectedFixture->can.sTxMailBox[0].TIR & CAN_TI0R_TXRQ) != 0u,
            "延后中断只能在新回调、参数与发送请求全部发布后执行");
    InjectedFixture->can.TSR.Reset(CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0);
    CanStm32TxIsrHandlerFixed(&InjectedFixture->dev);
    DeferredIrqDelivered++;
}

static void TestFixedCompletionThenReuse()
{
    Fixture fixture;
    CallbackRecord newRecord = {};
    const can_frame frame = TestFrame();

    fixture.Reset(CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0, false);
    CanStm32TxIsrHandlerFixed(&fixture.dev);
    Require(fixture.callbacks[0].count == 1,
            "修正 ISR 必须先交付旧请求完成");
    fixture.can.TSR.Reset(CAN_TSR_TME0);
    DeferredIrqDelivered = 0;
    InjectedFixture = &fixture;
    ExpectedNewRecord = &newRecord;
    CallbackWriteHook = InjectFixedIrq;
    Require(CanStm32SendFixed(&fixture.dev, &frame, K_FOREVER,
                              RecordCallback, &newRecord) == 0,
            "旧完成交付后修正发送函数必须允许复用邮箱");
    Require(DeferredIrqDelivered == 1 && IrqDepth == 0u,
            "回调安装期间的 IRQ 必须在发布后才交付");
    fixture.can.TSR.Reset(CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0);
    CanStm32TxIsrHandlerFixed(&fixture.dev);
    Require(fixture.callbacks[0].count == 1 && newRecord.count == 1,
            "修正路径必须分别交付旧帧和复用后的新帧完成");
}

int main()
{
    TestSingleMailbox();
    TestMultipleMailboxes();
    TestCompletionArrivingDuringIsr();
    TestBusOff();
    TestOldCompletionAfterSendSetup();
    TestIrqBetweenCallbackAndArgument();
    TestFixedCompletionThenReuse();
    return 0;
}
