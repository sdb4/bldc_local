IMUSRC = 	imu/mpu9150.c \
			imu/icm20948.c \
			imu/icm45686_driver/imu/inv_imu_transport.c \
			imu/icm45686_driver/imu/inv_imu_driver.c \
			imu/icm45686_driver/imu/inv_imu_driver_advanced.c \
			imu/icm45686_wrapper.c \
			imu/ahrs.c \
			imu/imu.c \
			imu/BMI160_driver/bmi160.c \
			imu/bmi160_wrapper.c \
			imu/lsm6ds3.c \
			imu/Fusion/FusionAhrs.c \
			imu/Fusion/FusionBias.c \
			imu/Fusion/FusionCompass.c

IMUINC = 	imu \
			imu/BMI160_driver \
			imu/icm45686_driver \
			imu/Fusion
