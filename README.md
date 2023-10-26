# FreeRTOS_MPU6050
## 硬件使用 stm32f103c8t6 + mpu6050 + OLED(6线spi)
## 软件使用 标准库 + FreeRTOS + 6轴卡尔曼滤波算法解算(pitch + roll)角度
## yaw直接通过积分获取
双任务( MPU6050解算 和 OLED显示输出 )
具体连线方法看代码


