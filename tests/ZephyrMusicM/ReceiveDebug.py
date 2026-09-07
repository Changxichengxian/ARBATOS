"""本机H723恢复烧录：校验芯片后只初始化AP0和M7，避开异常跟踪ROM。"""
from pyocd.coresight.ap import APv1Address, AccessPort
from pyocd.coresight.cortex_m import CortexM
from pyocd.coresight.minimal_mem_ap import MinimalMemAP
import time


def will_init_target(target, init_sequence):
    def create_core():
        minimal = MinimalMemAP(target.dp)
        minimal.init()
        running = (minimal.read32(0xE000EDF0) & (1 << 17)) == 0
        minimal.write32(0xE000EDF0, 0xA05F0003)
        minimal.write32(0x5C001004, 0x700187)
        target.dp.flush()
        address = APv1Address(0)
        ap = AccessPort.create(target.dp, address)
        target.dp.aps[address] = ap
        # 此DAP批量传输会在大块读取时USB超时；单次限制为8个32位字。
        read_words = ap.read_memory_block32
        write_words = ap.write_memory_block32
        def read_small(address, count):
            result = []
            for offset in range(0, count, 8):
                result.extend(read_words(address + offset * 4, min(8, count - offset)))
            return result
        def write_small(address, data):
            for offset in range(0, len(data), 8):
                write_words(address + offset * 4, data[offset:offset + 8])
                target.dp.flush()
        ap.read_memory_block32 = read_small
        ap.write_memory_block32 = write_small
        if ap.read32(0x5C001000) & 0xFFF != 0x483:
            raise RuntimeError("Recovery script requires STM32H723")
        ap.write32(0x5C001004, 0x700187)
        core = CortexM(target.session, ap, target.memory_map, 0, address=0xE000E000)
        ap.core = core
        target.add_core(core)
        target._new_core_num = 1
        core.init()
        if running:
            core.resume()
    init_sequence.replace_task("discovery", create_core)


def did_reset(core, reset_type):
    # 烧录器的软件复位也会清掉调试保持设置，必须在复位后恢复。
    core.write32(0x5C001004, 0x700187)
    core.flush()


def did_init_target(target):
    for region in target.memory_map:
        if not region.is_flash or region.flash is None:
            continue
        flash = region.flash
        erase = flash.erase_sector
        wait = flash.wait_for_completion
        erasing = [False]
        def erase_quiet(address):
            erasing[0] = True
            try:
                return erase(address)
            finally:
                erasing[0] = False
        def wait_quiet(timeout=None):
            # 擦除期间立即轮询DHCSR会使此DAP卡住；先等Flash操作完成再读。
            time.sleep(4.0 if erasing[0] else 0.01)
            return wait(timeout=timeout)
        flash.erase_sector = erase_quiet
        flash.wait_for_completion = wait_quiet
