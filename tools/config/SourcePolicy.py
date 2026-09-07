"""生成器与静态检查共用的 Zephyr 源文件边界。"""

import re

FORBIDDEN_SOURCE = re.compile(
    r"(?:^|/)(?:projects/[^/]+/(?:Core|Drivers|Middlewares|MDK-ARM)/|shared/hal/|"
    r".*(?:boardmain|boardfreertos|instask)[^/]*\.c$|"
    r"boards/(?:DjiCF407|DjiAF427|DmMc02H7)/(?:bsp|devices|app)/|"
    r".*\.(?:s|lib)$)", re.IGNORECASE
)
