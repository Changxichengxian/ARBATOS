"""保留原计划与原始结果，通过严格外层锁核验接续唯一新24例对照。"""
import sys
from TraceMonitorLockCompatibility import base, queue, install

install()
sys.argv = [str(__file__), "--plan", str(base / "evaluation/plan.json"), "--resume"]
queue.main()
