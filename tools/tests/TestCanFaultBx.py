"""完整编译 F4 故障 CAN 后端，模拟 W1C 邮箱、无ACK与取消失败。"""
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
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define CONFIG_SOC_SERIES_STM32F4X 1
#define CONFIG_CAN_STM32_BXCAN 1
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define DT_ALIAS(name) name
#define can_primary 1
#define can_secondary 2
#define DT_REG_ADDR(node) ((node)*0x1000u)
#define CAN1_BASE 0x1000u
#define CAN2_BASE 0x2000u
#define RCC_APB1ENR_CAN1EN (1u<<25)
#define RCC_APB1ENR_CAN2EN (1u<<26)
#define CAN_MCR_INRQ 1u
#define CAN_MCR_SLEEP 2u
#define CAN_MSR_INAK 1u
#define CAN_MSR_SLAK 2u
#define CAN_ESR_BOFF 4u
#define CAN_BTR_LBKM (1u<<30)
#define CAN_BTR_SILM (1u<<31)
#define CAN_TSR_RQCP0 1u
#define CAN_TSR_RQCP1 (1u<<8)
#define CAN_TSR_RQCP2 (1u<<16)
#define CAN_TSR_TXOK0 2u
#define CAN_TSR_ALST0 4u
#define CAN_TSR_TERR0 8u
#define CAN_TSR_ABRQ0 (1u<<7)
#define CAN_TSR_ABRQ1 (1u<<15)
#define CAN_TSR_ABRQ2 (1u<<23)
#define CAN_TSR_TME0 (1u<<26)
#define CAN_TSR_TME1 (1u<<27)
#define CAN_TSR_TME2 (1u<<28)
#define CAN_TI0R_TXRQ 1u
typedef struct { uint32_t TIR,TDTR,TDLR,TDHR; } CAN_TxMailBox_TypeDef;
typedef struct { uint32_t MCR,MSR,TSR,RF0R,RF1R,IER,ESR,BTR;
uint32_t reserved[88]; CAN_TxMailBox_TypeDef sTxMailBox[3]; } CAN_TypeDef;
static struct { uint32_t APB1ENR; } rcc;
#define RCC (&rcc)
static unsigned primask=1,dsb,reads;
static uint32_t __get_PRIMASK(void) { return primask; }
static void __DSB(void) { dsb++; }
static uint32_t regs[2][0x200/4];
static unsigned abortFail[2],abortWrites[2],sends[2],clearStuck[2];
enum { Complete,Pending,Error,StaleComplete };
static unsigned txMode[2];
static unsigned busFor(uintptr_t addr)
{ unsigned bus=(unsigned)(addr/0x1000u)-1u; assert(bus<2); return bus; }
static uint32_t sys_read32(uintptr_t addr)
{ reads++; return regs[busFor(addr)][(addr%0x1000u)/4]; }
static void sys_write32(uint32_t value,uintptr_t addr)
{
    unsigned bus=busFor(addr),offset=(unsigned)(addr%0x1000u);
    uint32_t *tsr=&regs[bus][offsetof(CAN_TypeDef,TSR)/4];
    if(offset==offsetof(CAN_TypeDef,TSR)) {
        for(unsigned slot=0;slot<3;slot++) {
            unsigned shift=slot*8;
            uint32_t *tir=&regs[bus][(offsetof(CAN_TypeDef,sTxMailBox)+slot*16)/4];
            if((value&(1u<<shift)) && !clearStuck[bus]) *tsr&=~(15u<<shift);
            if(value&(1u<<(shift+7))) {
                abortWrites[bus]|=1u<<slot;
                if(!(abortFail[bus]&(1u<<slot))) {
                    *tir&=~1u;
                    *tsr=(*tsr&~(15u<<shift)) | (1u<<shift) | (1u<<(26+slot));
                }
            }
        }
        return;
    }
    regs[bus][offset/4]=value;
    if(offset==offsetof(CAN_TypeDef,sTxMailBox[0].TIR) && (value&CAN_TI0R_TXRQ)) {
        sends[bus]++;
        *tsr&=~CAN_TSR_TME0;
        if(txMode[bus]==Complete || txMode[bus]==Error) {
            regs[bus][offset/4]&=~1u;
            *tsr|=CAN_TSR_TME0 | CAN_TSR_RQCP0 | (txMode[bus]==Complete?CAN_TSR_TXOK0:CAN_TSR_TERR0);
        } else if(txMode[bus]==StaleComplete) {
            /* 旧状态看起来已完成，但新请求仍挂在邮箱，不能报告成功。 */
            *tsr|=CAN_TSR_TME0 | CAN_TSR_RQCP0 | CAN_TSR_TXOK0;
        }
    }
}
'''
    driver = (ROOT / "shared/zephyr/port/can/BspCanFaultPort.c").read_text(encoding="utf-8")
    driver = re.sub(r"(?m)^#include[^\n]*\n", "", driver)
    tests = r'''
#define REG(bus,member) regs[bus][offsetof(CAN_TypeDef,member)/4]
static void reset(void)
{
    memset(regs,0,sizeof(regs)); memset(abortFail,0,sizeof(abortFail));
    memset(abortWrites,0,sizeof(abortWrites)); memset(sends,0,sizeof(sends));
    memset(clearStuck,0,sizeof(clearStuck)); memset(txMode,0,sizeof(txMode));
    memset(FaultBxOwned,0,sizeof(FaultBxOwned));
    rcc.APB1ENR=RCC_APB1ENR_CAN1EN|RCC_APB1ENR_CAN2EN;
    primask=1; dsb=reads=0;
    for(unsigned b=0;b<2;b++) { REG(b,TSR)=CAN_FAULT_BX_EMPTY; REG(b,IER)=0xffffffffu; }
}
int main(void)
{
    uint8_t data[8]={0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
    reset(); assert(BspCanFaultPortSupported()==1);
    assert(BspCanFaultPortSend(1,0x321,data,8)==1); /* 未取消普通帧不得接管邮箱。 */
    BspCanFaultPortAbort();
    assert(FaultBxOwned[0] && FaultBxOwned[1] && REG(0,IER)==0 && REG(1,IER)==0);
    assert(BspCanFaultPortSend(2,0x321,data,8)==0);
    assert(!sends[0] && sends[1]==1);
    assert(REG(1,sTxMailBox[0].TIR)==(0x321u<<21));
    assert(REG(1,sTxMailBox[0].TDTR)==8 && REG(1,sTxMailBox[0].TDLR)==0x44332211u);
    assert(REG(1,sTxMailBox[0].TDHR)==0x88776655u);
    assert(BspCanFaultPortSend(2,0x123,data,3)==0 && REG(1,sTxMailBox[0].TDLR)==0x00332211u);
    assert(REG(1,sTxMailBox[0].TDHR)==0);
    primask=0; assert(BspCanFaultPortSend(1,1,data,1)==1); primask=1;
    assert(BspCanFaultPortSend(0,1,data,1)==1 && BspCanFaultPortSend(3,1,data,1)==1);
    assert(BspCanFaultPortSend(1,0x800,data,1)==1 && BspCanFaultPortSend(1,1,NULL,1)==1);
    assert(BspCanFaultPortSend(1,1,data,9)==1);
    REG(0,MCR)=CAN_MCR_INRQ; assert(BspCanFaultPortSend(1,1,data,1)==1); REG(0,MCR)=0;
    REG(0,MSR)=CAN_MSR_SLAK; assert(BspCanFaultPortSend(1,1,data,1)==1); REG(0,MSR)=0;
    REG(0,ESR)=CAN_ESR_BOFF; assert(BspCanFaultPortSend(1,1,data,1)==1); REG(0,ESR)=0;
    REG(0,BTR)=CAN_BTR_SILM; assert(BspCanFaultPortSend(1,1,data,1)==1); REG(0,BTR)=0;
    rcc.APB1ENR=RCC_APB1ENR_CAN2EN; assert(BspCanFaultPortSend(2,1,data,1)==1);
    reset(); REG(0,TSR)&=~CAN_TSR_TME1; REG(0,sTxMailBox[1].TIR)=1;
    abortFail[0]=2; BspCanFaultPortAbort();
    assert(!FaultBxOwned[0] && FaultBxOwned[1] && reads<1000000);
    assert(BspCanFaultPortSend(1,1,data,1)==1 && !sends[0]);
    assert(BspCanFaultPortSend(2,1,data,1)==0);
    reset(); BspCanFaultPortAbort(); txMode[0]=Pending;
    assert(BspCanFaultPortSend(1,1,data,1)==3 && !FaultBxOwned[0]);
    assert((REG(0,sTxMailBox[0].TIR)&1u)==0 && abortWrites[0]==7);
    assert(BspCanFaultPortSend(1,1,data,1)==1 && sends[0]==1);
    reset(); BspCanFaultPortAbort(); txMode[0]=Pending; abortFail[0]=1;
    assert(BspCanFaultPortSend(1,1,data,1)==3 && !FaultBxOwned[0] && reads<1000000);
    assert((REG(0,sTxMailBox[0].TIR)&1u)!=0);
    reset(); BspCanFaultPortAbort(); REG(0,TSR)|=CAN_TSR_RQCP0|CAN_TSR_TXOK0; clearStuck[0]=1;
    assert(BspCanFaultPortSend(1,1,data,1)==1 && !sends[0]);
    reset(); BspCanFaultPortAbort(); txMode[0]=StaleComplete;
    assert(BspCanFaultPortSend(1,1,data,1)==3 && !FaultBxOwned[0]);
    reset(); BspCanFaultPortAbort(); txMode[0]=Error;
    assert(BspCanFaultPortSend(1,1,data,1)==1);
    txMode[0]=Complete; assert(BspCanFaultPortSend(1,1,data,1)==0);
    puts("PASS: F4 fatal CAN abort, W1C, no ACK, stale completion, mailbox and route isolation");
    return 0;
}
'''
    out = ROOT / "local/build/host-tests/can-fault-bx"
    out.mkdir(parents=True, exist_ok=True)
    source = out / "regression.c"
    source.write_text(fixture + driver + tests, encoding="utf-8")
    executable = out / "regression.exe"
    subprocess.run([args.zig, "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", str(source), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
