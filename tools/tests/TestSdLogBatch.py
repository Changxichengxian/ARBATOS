"""直接运行固件写卡调度代码，检查凑批、超时、环绕和停写尾部排空。"""
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[2]
source = (root/'shared/application/services/storage/SdLog.c').read_text(encoding='utf-8')
poll = source[source.index('void SdLogPoll(void)'):source.index('void SdLogGetStats(')]
quiesce = source[source.index('int SdLogQuiesce(void)'):source.index('uint8_t SdLogTryWrite(')]
folder = root/'local/build/host-tests/sdlog-batch'
folder.mkdir(parents=True, exist_ok=True)
unit, exe = folder/'batch.c', folder/'batch.exe'
unit.write_text(r'''
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#define CONFIG_ARBATOS_MINI_BENCH 1
#define SDLOG_BUF_SIZE (128u*1024u)
#define SDLOG_FLUSH_CHUNK_MAX (16u*1024u)
#define SDLOG_FLUSH_BLOCKS_PER_POLL 4u
#define SDLOG_SYNC_PERIOD_MS 5000u
#define SDLOG_RESTART_REASON_STOP 1
#define FR_OK 0
typedef int FRESULT;
static uint32_t now, sdlog_active, sdlog_head, sdlog_tail, sdlog_last_tick_ms;
static uint32_t sdlog_last_sync_ms, sdlog_last_flush_ms, writes, written, produced;
static int sdlog_fp, sdlog_last_error, critical, closed;
static uint8_t sdlog_buf[SDLOG_BUF_SIZE], sdlog_flush_in[SDLOG_FLUSH_CHUNK_MAX];
#define taskENTER_CRITICAL() (++critical)
#define taskEXIT_CRITICAL() (--critical)
static uint32_t BspTimeGetTickMs(void) { return now; }
static uint32_t sdlog_used_bytes(uint32_t h, uint32_t t) { return (h+SDLOG_BUF_SIZE-t)%SDLOG_BUF_SIZE; }
static int sdlog_sync_profiled(void) { return 0; }
static int f_close(int *fp) { (void)fp; closed++; return 0; }
static void SdcardUnmount(void) { assert(0); }
static void sdlog_close_on_error(void) { assert(0); }
static void sdlog_remember_error(int reason, int error) { (void)reason; (void)error; }
static int SdLogWrite_block(const uint8_t *p, uint32_t n)
{
    assert(critical == 0);
    for (uint32_t i=0; i<n; i++) assert(p[i] == (uint8_t)(written+i));
    written += n;
    writes++;
    sdlog_last_flush_ms = now;
    return 0;
}
static void Produce(uint32_t n)
{
    assert(sdlog_active && n < SDLOG_BUF_SIZE-sdlog_used_bytes(sdlog_head,sdlog_tail));
    for (uint32_t i=0; i<n; i++) {
        sdlog_buf[sdlog_head] = (uint8_t)produced++;
        sdlog_head = (sdlog_head+1)%SDLOG_BUF_SIZE;
    }
}
''' + quiesce + poll + r'''
int main(void)
{
    sdlog_active=1;
    Produce(1000);
    now=49;
    SdLogPoll();
    assert(writes==0 && critical==0);
    now=50;
    SdLogPoll();
    assert(writes==1 && written==1000 && sdlog_head==sdlog_tail);
    Produce(SDLOG_FLUSH_CHUNK_MAX);
    SdLogPoll();
    assert(writes==2 && written==1000+SDLOG_FLUSH_CHUNK_MAX);
    /* 刚完成批量写入后的零头，也必须由停写接口立即排空。 */
    Produce(17);
    assert(SdLogQuiesce()==0 && !sdlog_active && written==produced && closed==1);
    /* 从环形缓冲末端跨界复制，字节顺序不能变。 */
    sdlog_active=1;
    sdlog_head=sdlog_tail=SDLOG_BUF_SIZE-7;
    Produce(SDLOG_FLUSH_CHUNK_MAX+3);
    SdLogPoll();
    assert(sdlog_used_bytes(sdlog_head,sdlog_tail)==3);
    assert(SdLogQuiesce()==0 && written==produced && critical==0 && closed==2);
    /* 毫秒计数翻转后，50 ms 超时仍有效。 */
    sdlog_active=1;
    sdlog_last_flush_ms=0xfffffff0u;
    Produce(5);
    now=0x21;
    uint32_t before=writes;
    SdLogPoll();
    assert(writes==before);
    now=0x22;
    SdLogPoll();
    assert(writes==before+1 && written==produced && critical==0);
    puts("batch, deadline, wrap, quiesce and tick-overflow passed");
}
''', encoding='utf-8')
subprocess.run([r'C:/Users/28111/AppData/Local/Microsoft/WinGet/Links/zig.exe',
                'cc', '-O2', '-Wall', '-Wextra', '-Werror', str(unit), '-o', str(exe)], check=True)
subprocess.run([str(exe)], check=True)
