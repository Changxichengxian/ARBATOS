#ifndef IMU_CAL_STORE_H
#define IMU_CAL_STORE_H

/* 独立内部 Flash 双副本，校验板号和校验和。
 * 正式固件只在 IMU 手动校准已锁定输出、暂停存储时允许擦写。 */
#include <stdint.h>
int ImuCalStoreLoad(float offset[3]);
int ImuCalStoreIsWriting(void);
uint32_t ImuCalStoreSequence(void);
int ImuCalStoreLoaded(void);
int ImuCalStoreSave(const float offset[3], float temperature);
void ImuCalStoreQueue(const float offset[3], float temperature);

#endif
