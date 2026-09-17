#ifndef IMU_CAL_STORE_H
#define IMU_CAL_STORE_H

/* 独立内部 Flash 双副本，校验板号和安装矩阵版本。
 * M 板仅准备固件允许擦写；A 板正式固件只读，由停机校准工具烧入。 */
int ImuCalStoreLoad(float offset[3]);
int ImuCalStoreIsWriting(void);
int ImuCalStoreSave(const float offset[3], float temperature);
void ImuCalStoreQueue(const float offset[3], float temperature);

#endif
