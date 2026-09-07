"""逐项读取 H723 调试端口，避免普通连接失败后直接归因于接线。"""
from pyocd.core.helpers import ConnectHelper
from pyocd.coresight.ap import AccessPort, APv1Address
import sys

s = ConnectHelper.session_with_chosen_probe(
    unique_id="486655686570", target_override="stm32h723xx", frequency=100000,
    options={"connect_mode": "attach", "resume_on_disconnect": False,
             "cmsis_dap.limit_packets": True, "cmsis_dap.deferred_transfers": False})
failed = False
try:
    s.open(init_board=False)
    dp = s.board.target.dp
    dp.create_connect_sequence().invoke()
    print("DP", hex(dp.read_dp(0)), "CTRL", hex(dp.read_dp(4)), flush=True)
    for index in range(3):
        try:
            print("AP", index, "IDR", hex(dp.read_ap((index << 24) | 0xfc)), flush=True)
        except Exception as exc:
            failed = True
            print("AP", index, type(exc).__name__, str(exc), flush=True)
            dp.clear_sticky_err()
    try:
        ap = AccessPort.create(dp, APv1Address(2))
        print("AP2 DBG ID", hex(ap.read32(0xe00e1000)), flush=True)
        print("AP2 DBG CR", hex(ap.read32(0xe00e1004)), flush=True)
        if "--recover" in sys.argv:
            # H723 的 D3 保持位在 7/8，不能照抄其他 H7 型号的 3/4/5。
            ap.write32(0xe00e1004, (ap.read32(0xe00e1004) & ~0x38) | 0x700187)
            dp.flush()
            print("AP2 DBG CR updated", hex(ap.read32(0xe00e1004)), flush=True)
            core_ap = AccessPort.create(dp, APv1Address(0))
            print("DHCSR before", hex(core_ap.read32(0xe000edf0)), flush=True)
            core_ap.write32(0xe000edf0, 0xa05f0003)
            core_ap.write32(0x40000440, 0)
            mode = core_ap.read32(0x58020400)
            core_ap.write32(0x58020418, 1 << 17)
            core_ap.write32(0x58020400, (mode & ~(3 << 2)) | (1 << 2))
            dp.flush()
            assert ((core_ap.read32(0x58020400) >> 2) & 3) == 1
            assert ((core_ap.read32(0x58020414) >> 1) & 1) == 0
            core_ap.write32(0xe000edf0, 0xa05f0003)
            core_ap.write32(0x40000440, 0)
            dp.flush()
            print("Heater PB1 forced LOW; DHCSR", hex(core_ap.read32(0xe000edf0)), flush=True)
    except Exception as exc:
        failed = True
        print("Memory access", type(exc).__name__, str(exc), flush=True)
finally:
    s.close()
raise SystemExit(1 if failed else 0)
