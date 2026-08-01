/*
 * 旧代码把 SD 日志工作区标成 .ccmram。Zephyr 不应让它成为孤立段：
 * 统一放入可访问外设 DMA 的普通 RAM，避免 H7 DTCM 的总线可见性问题。
 */
SECTION_DATA_PROLOGUE(.ccmram,,)
{
	. = ALIGN(4);
	*(.ccmram)
	. = ALIGN(4);
} GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)
