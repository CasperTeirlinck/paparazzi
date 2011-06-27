
IMU_UMARIM_CFLAGS  = -DUSE_IMU
IMU_UMARIM_CFLAGS += -DIMU_TYPE_H=\"subsystems/imu/imu_umarim.h\"

IMU_UMARIM_SRCS    = $(SRC_SUBSYSTEMS)/imu.c             \
                     $(SRC_SUBSYSTEMS)/imu/imu_umarim.c


IMU_UMARIM_CFLAGS += -DUSE_I2C
IMU_UMARIM_CFLAGS += -DUSE_I2C1
IMU_UMARIM_CFLAGS += -DIMU_UMARIM_I2C_DEVICE=i2c1

ap.CFLAGS += $(IMU_UMARIM_CFLAGS)
ap.srcs   += $(IMU_UMARIM_SRCS)

