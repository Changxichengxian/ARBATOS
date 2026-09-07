#ifndef IMU_CAL_STORE_H
#define IMU_CAL_STORE_H

/* 独立内部 Flash 双副本，校验板号和安装矩阵版本；只在静态准备固件中擦写。 */
int ImuCalStoreLoad(float offset[3]);
int ImuCalStoreIsWriting(void);
int ImuCalStoreSave(const float offset[3], float temperature);
void ImuCalStoreQueue(const float offset[3], float temperature);

#endif
