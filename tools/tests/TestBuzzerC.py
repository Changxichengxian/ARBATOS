"""编译完整 C 板 PCM 驱动，使用定时器/DMA 桩检查缓冲、结束和停止竞态。"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zig", default=shutil.which("zig"))
    args = parser.parse_args()
    if not args.zig:
        parser.error("请用 --zig 指定 Zig 编译器")
    fixture = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define CLAMP(x,a,b) MIN(MAX(x,a),b)
#define ARG_UNUSED(x) ((void)(x))
#define __aligned(n) __attribute__((aligned(n)))
#define BUILD_ASSERT(...) /* 板级资源由实际 Zephyr 构建检查。 */
#define DT_NODELABEL(x) 0
#define DT_PATH(x) 0
#define DT_HAS_CHOSEN(x) 0
#define DT_DMAS_CELL_BY_NAME(node,name,cell) 7
#define DT_DMAS_CTLR_BY_NAME(...) 0
#define DEVICE_DT_GET(...) (&device)
#define ARBATOS_PWM_DT_SPEC_GET_BY_IDX(...) { .dev=&device, .channel=3 }
#define K_FOREVER -1
#define K_NO_WAIT 0
#define DMA_ADDR_ADJ_INCREMENT 0
#define DMA_ADDR_ADJ_NO_CHANGE 2
#define MEMORY_TO_PERIPHERAL 1
#define DMA_STATUS_BLOCK 1
#define DMA_STATUS_COMPLETE 0
#define LL_TIM_CHANNEL_CH3 4
struct device { int ready; };
static struct device device={1};
static int device_is_ready(const struct device *dev) { return dev->ready; }
struct pwm_dt_spec { const struct device *dev; unsigned channel,flags; };
struct k_mutex { unsigned locked; };
struct k_spinlock { unsigned unused; };
typedef unsigned k_spinlock_key_t;
struct k_work { void (*handler)(struct k_work *); unsigned pending; };
struct k_work_sync { unsigned unused; };
#define K_MUTEX_DEFINE(name) struct k_mutex name
static int k_mutex_lock(struct k_mutex *m,int timeout)
{ if(m->locked) { assert(timeout==K_NO_WAIT); return -EBUSY; } m->locked=1; return 0; }
static void k_mutex_unlock(struct k_mutex *m) { assert(m->locked); m->locked=0; }
static unsigned k_spin_lock(struct k_spinlock *l) { (void)l; return 0; }
static void k_spin_unlock(struct k_spinlock *l,unsigned key) { (void)l; (void)key; }
static void k_work_init(struct k_work *w,void (*handler)(struct k_work *)) { w->handler=handler; }
static int k_work_submit(struct k_work *w) { w->pending=1; return 0; }
static unsigned cancelCount;
static int k_work_cancel_sync(struct k_work *w,struct k_work_sync *s)
{ (void)s; cancelCount++; if(w->pending) w->handler(w); w->pending=0; return 0; }
static struct { unsigned CCR3,ARR,enabled,ccEnabled,request; } timer;
static struct { unsigned NDTR; } stream;
#define TIM4 (&timer)
#define DMA1_Stream7 (&stream)
static void LL_TIM_DisableDMAReq_CC3(void *t) { (void)t; timer.request=0; }
static void LL_TIM_EnableDMAReq_CC3(void *t) { (void)t; timer.request=1; }
static void LL_TIM_CC_DisableChannel(void *t,unsigned ch) { (void)t; (void)ch; timer.ccEnabled=0; }
static void LL_TIM_CC_EnableChannel(void *t,unsigned ch) { (void)t; (void)ch; timer.ccEnabled=1; }
static void LL_TIM_DisableCounter(void *t) { (void)t; timer.enabled=0; }
static void LL_TIM_EnableCounter(void *t) { (void)t; timer.enabled=1; }
static void LL_TIM_OC_SetCompareCH3(void *t,unsigned value) { (void)t; timer.CCR3=value; }
static void LL_TIM_OC_EnablePreload(void *t,unsigned ch) { (void)t; (void)ch; }
static void LL_TIM_GenerateEvent_UPDATE(void *t) { (void)t; }
static void LL_TIM_SetCounter(void *t,unsigned count) { (void)t; (void)count; }
static void LL_TIM_ClearFlag_CC3(void *t) { (void)t; }
static int pwm_is_ready_dt(const struct pwm_dt_spec *p) { return device_is_ready(p->dev); }
static int pwm_get_cycles_per_sec(const struct device *d,unsigned ch,uint64_t *rate)
{ (void)d; (void)ch; *rate=84000000; return 0; }
static int pwm_set_cycles(const struct device *d,unsigned ch,unsigned cycles,unsigned pulse,unsigned flags)
{ (void)d; (void)ch; (void)flags; timer.ARR=cycles-1; timer.CCR3=pulse; timer.enabled=timer.ccEnabled=1; return 0; }
struct dma_block_config { uint32_t source_address,dest_address,block_size;
unsigned source_addr_adj,dest_addr_adj,source_reload_en,dest_reload_en; };
struct dma_config { unsigned dma_slot,channel_direction,source_data_size,dest_data_size,source_burst_length,
dest_burst_length,channel_priority,block_count; struct dma_block_config *head_block;
void (*dma_callback)(const struct device *,void *,uint32_t,int); void *user_data; };
static struct dma_config submitted;
static unsigned dmaStops,dmaActive,inCallback;
static int configResult,startResult;
static int dma_config(const struct device *d,unsigned ch,struct dma_config *cfg)
{ (void)d; assert(ch==7); assert(cfg->head_block->block_size==2048); submitted=*cfg; return configResult; }
static int dma_start(const struct device *d,unsigned ch)
{ (void)d; (void)ch; dmaActive=startResult==0; return startResult; }
static int dma_stop(const struct device *d,unsigned ch)
{ (void)d; (void)ch; assert(!inCallback); dmaStops++; dmaActive=0; return 0; }
void BuzzerPcmStop(void);
void BuzzerToneStop(void);
'''
    driver = (ROOT / "shared/zephyr/port/platform/BspBuzzerC.c").read_text(encoding="utf-8")
    driver = re.sub(r"(?m)^#include[^\n]*\n", "", driver)
    tests = r'''
static void irq(int status)
{
    stream.NDTR=status==DMA_STATUS_BLOCK?512:1024;
    inCallback=1; submitted.dma_callback(&device,submitted.user_data,7,status); inCallback=0;
}
static void runWork(void)
{
    if(BuzzerEndWork.pending) { BuzzerEndWork.pending=0; BuzzerEndWork.handler(&BuzzerEndWork); }
}
int main(void)
{
    uint8_t data[300]; memset(data,255,sizeof(data));
    assert(BspBuzzerPlatformInit()==0);
    assert(BuzzerPcmStartU8(data,sizeof(data),12000,0,255)==-EINVAL);
    BuzzerSetEnable(1);
    assert(BuzzerPcmStartU8(data,sizeof(data),12000,0,255)==0);
    assert(timer.request && timer.ccEnabled && timer.enabled && dmaActive);
    assert(BuzzerLut[0]<BuzzerLut[128] && BuzzerLut[128]<BuzzerLut[255]);
    assert(BuzzerPos==256 && BuzzerPhase==0); /* 每个12k样本保持4个48k载波周期。 */
    irq(DMA_STATUS_BLOCK); assert(BuzzerHalfFinal[0] && BuzzerPcmIsRunning());
    irq(DMA_STATUS_COMPLETE); assert(BuzzerPcmIsRunning());
    irq(DMA_STATUS_BLOCK); assert(!BuzzerPcmIsRunning());
    assert(!timer.request && !timer.ccEnabled && !timer.enabled && timer.CCR3==0);
    assert(dmaActive); /* 中断只关闭请求，带等待的DMA清理留给线程。 */
    runWork(); assert(!dmaActive);
    assert(BuzzerPcmStartStreamU8(12000,128)==0 && !BuzzerArmed);
    assert(BuzzerPcmStreamWriteU8(data,sizeof(data))==sizeof(data));
    assert(BuzzerArmed && BuzzerLut[0]<BuzzerLut[128]);
    BuzzerPcmStreamFinish();
    assert(BuzzerPcmStreamWriteU8(data,1)==0);
    irq(DMA_STATUS_BLOCK); irq(DMA_STATUS_COMPLETE); irq(DMA_STATUS_BLOCK);
    assert(!BuzzerPcmIsRunning());
    /* 新播放在旧结束工作尚未运行时启动，旧工作不得关闭新DMA。 */
    void *oldGeneration=submitted.user_data;
    assert(BuzzerPcmStartU8(data,sizeof(data),12000,1,255)==0);
    runWork(); assert(BuzzerArmed && dmaActive);
    inCallback=1; BuzzerDmaCallback(&device,oldGeneration,7,DMA_STATUS_BLOCK); inCallback=0;
    assert(BuzzerArmed); /* 旧播放的迟到回调被代次拒绝。 */
    unsigned pos=BuzzerPos;
    BuzzerPcmStop(); assert(!BuzzerArmed && !dmaActive && !timer.request && cancelCount);
    irq(DMA_STATUS_BLOCK); assert(BuzzerPos==pos && !BuzzerPcmIsRunning());
    assert(BuzzerPcmStartStreamU8(48000,255)==0);
    assert(BuzzerPcmStreamWriteU8(data,3)==3);
    for(unsigned i=0;i<1024;i++) assert(BuzzerBuffer[i/512][i%512]>=1);
    irq(DMA_STATUS_BLOCK); irq(DMA_STATUS_COMPLETE); /* 空流保持静音中点，仍可继续写入。 */
    assert(BuzzerArmed && BuzzerBuffer[1][100]==BuzzerLut[128]);
    BuzzerPcmStop();
    assert(BuzzerPcmStartU8(data,sizeof(data),12000,1,255)==0);
    stream.NDTR=900; inCallback=1; BuzzerDmaCallback(&device,submitted.user_data,7,DMA_STATUS_BLOCK); inCallback=0;
    assert(!BuzzerArmed && !timer.request); /* 半缓冲回调迟到，拒绝改写正在播放的半区。 */
    runWork();
    startResult=-EIO; assert(BuzzerPcmStartU8(data,sizeof(data),12000,0,255)==-EIO);
    assert(!BuzzerPcmIsRunning() && !timer.request && !dmaActive); startResult=0;
    assert(BuzzerToneStartLegacy(20,10500)==0 && BuzzerMode==3 && timer.request);
    assert(BuzzerToneStep>0 && BuzzerToneStep<(1u<<27)); /* 旧PSC低音也能通过固定载波播放。 */
    BuzzerSetEnable(0); assert(!BuzzerPcmIsRunning() && !timer.ccEnabled);
    puts("PASS: C board DMA PCM buffers, drain, stale callbacks, stop and restart");
    return 0;
}
'''
    out = ROOT / "local/build/host-tests/buzzer-c"
    out.mkdir(parents=True, exist_ok=True)
    source = out / "regression.c"
    source.write_text(fixture + driver + tests, encoding="utf-8")
    executable = out / "regression.exe"
    subprocess.run([args.zig, "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", str(source), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
