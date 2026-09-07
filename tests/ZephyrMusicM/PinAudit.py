"""对照当前 STM32H723VGT6 引脚表，检查自定义设备树的复用值。"""
import argparse
import json
import re
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    reference = root / "local/cache/zephyrproject/modules/hal/stm32/dts/st/h7/stm32h723vgtx-pinctrl.dtsi"
    pattern = r"(\w+):\s*\w+\s*\{\s*pinmux\s*=\s*<([^>]+)>"
    expected = dict(re.findall(pattern, reference.read_text(encoding="utf-8")))
    aliases = {"spi2_miso_pc2": "spi2_miso_pc2_c"}
    rows = []
    for name in ["zephyr/boards/dm_mc02_h7/dm_mc02_h7-pinctrl.dtsi",
                 "zephyr/targets/hero-m-music.overlay"]:
        for label, actual in re.findall(pattern, (root / name).read_text(encoding="utf-8")):
            wanted = expected.get(aliases.get(label, label))
            rows.append({"file": name, "pin": label, "actual": actual, "expected": wanted,
                         "match": wanted is not None and re.sub(r"\s", "", actual) == re.sub(r"\s", "", wanted)})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"reference": str(reference), "pins": rows},
                                     ensure_ascii=False, indent=2), encoding="utf-8")
    failures = [row for row in rows if not row["match"]]
    print(f"{len(rows)} pin mappings checked, {len(failures)} mismatches")
    for failure in failures:
        print(failure)
    raise SystemExit(bool(failures))


if __name__ == "__main__":
    main()
