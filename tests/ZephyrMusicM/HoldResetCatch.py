"""在用户按住 RESET 时设置暂停，并保持 DAP 连接等待松手。"""
import time
from pyocd.core.helpers import ConnectHelper
from pyocd.coresight.minimal_mem_ap import MinimalMemAP

s = ConnectHelper.session_with_chosen_probe(
    unique_id="486655686570", target_override="stm32h723xx", frequency=1000000,
    options={"connect_mode": "attach", "resume_on_disconnect": False,
             "cmsis_dap.limit_packets": True})
try:
    s.open(init_board=False)
    dp = s.board.target.dp
    dp.create_connect_sequence().invoke()
    deadline = time.monotonic() + 90
    while True:
        try:
            ap = MinimalMemAP(dp)
            ap.init()
            ap.write32(0xe000edfc, ap.read32(0xe000edfc) | 1)
            ap.write32(0xe000edf0, 0xa05f0001)
            dp.flush()
            print("ARMED: release RESET now; DAP remains connected", flush=True)
            break
        except Exception:
            dp.clear_sticky_err()
            if time.monotonic() > deadline:
                raise
            time.sleep(0.2)
    while time.monotonic() < deadline:
        value = ap.read32(0xe000edf0)
        if value & (1 << 17):
            ap.write32(0x5c001004, 0x700187)
            print("HALTED", hex(value), "DBG", hex(ap.read32(0x5c001004)), flush=True)
            break
        time.sleep(0.05)
    else:
        raise TimeoutError("reset release did not produce a halted core")
finally:
    s.close()
