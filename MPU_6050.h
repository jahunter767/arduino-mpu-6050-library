#ifndef MPU_6050_h
#define MPU_6050_h

/*
#ifdef __AVR__
#include <avr/pgmspace.h>
#endif
#if (ARDUINO >= 100)
#include "Arduino.h"
#endif
*/

#include "lib/i2c/I2C.h"

#define MPU_6050_DLPF_BANDWIDTH_260          0x00
#define MPU_6050_DLPF_BANDWIDTH_184          0x01
#define MPU_6050_DLPF_BANDWIDTH_94           0x02
#define MPU_6050_DLPF_BANDWIDTH_44           0x03
#define MPU_6050_DLPF_BANDWIDTH_21           0x04
#define MPU_6050_DLPF_BANDWIDTH_10           0x05
#define MPU_6050_DLPF_BANDWIDTH_5            0x06

#define MPU_6050_GYRO_FULL_SCALE_250         0x00
#define MPU_6050_GYRO_FULL_SCALE_500         0x08
#define MPU_6050_GYRO_FULL_SCALE_1000        0x10
#define MPU_6050_GYRO_FULL_SCALE_2000        0x18

#define MPU_6050_ACCEL_FULL_SCALE_2G         0x00
#define MPU_6050_ACCEL_FULL_SCALE_4G         0x08
#define MPU_6050_ACCEL_FULL_SCALE_8G         0x10
#define MPU_6050_ACCEL_FULL_SCALE_16G        0x18

#define MPU_6050_FIFO_ENABLE_GYRO_X          0x40
#define MPU_6050_FIFO_ENABLE_GYRO_Y          0x20
#define MPU_6050_FIFO_ENABLE_GYRO_Z          0x10

#define MPU_6050_FIFO_ENABLE_SLAVE_2         0x04
#define MPU_6050_FIFO_ENABLE_SLAVE_1         0x02
#define MPU_6050_FIFO_ENABLE_SLAVE_0         0x01


/*
#define MPU_6050_DEFAULT_ADDRESS             0x68
#define MPU_6050_FIFO_SIZE                  1024

#define MPU_6050_SAMPLE_RATE_REG             0x19

#define MPU_6050_CONFIG_REG                  0x1A
#define MPU_6050_CONFIG_EXT_SIGNAL_SET_MASK  0xC7
#define MPU_6050_CONFIG_DLPF_CFG_MASK        0xF8

#define MPU_6050_GYRO_CFG_REG                0x1B
#define MPU_6050_GYRO_CFG_SELF_TEST_MASK     0x1F
#define MPU_6050_GYRO_CFG_RANGE_SEL_MASK     0xE7

#define MPU_6050_ACCEL_CFG_REG               0x1C
#define MPU_6050_ACCEL_CFG_SELF_TEST_MASK    0x1F
#define MPU_6050_ACCEL_CFG_RANGE_SEL_MASK    0xE7

#define MPU_6050_FIFO_ENABLE_REG             0x23
#define MPU_6050_FIFO_ENABLE_TEMP_MASK       0x7F
#define MPU_6050_FIFO_ENABLE_GYRO_MASK       0x8F
#define MPU_6050_FIFO_ENABLE_ACCEL_MASK      0xF7
#define MPU_6050_FIFO_ENABLE_SLAVE_MASK      0xF8

#define MPU_6050_INT_PIN_CFG_REG             0x37
#define MPU_6050_INT_LEVEL_MASK              0x7F
#define MPU_6050_INT_OPEN_MASK               0xBF
#define MPU_6050_INT_LATCH_MASK              0xDF
#define MPU_6050_INT_READ_CLEAR_MASK         0xEF

#define MPU_6050_INT_ENABLE_REG              0x38
#define MPU_6050_INT_ENABLE_FIFO_OVF_MASK    0xEF
#define MPU_6050_INT_ENABLE_DATA_RDY_MASK    0xFE

#define MPU_6050_INT_STATUS_REG              0x3A
#define MPU_6050_INT_STATUS_FIFO_OVF_MASK    0xEF
#define MPU_6050_INT_STATUS_DATA_RDY_MASK    0xFE

#define MPU_6050_ACCEL_X_HIGH_REG            0x3B
#define MPU_6050_ACCEL_X_LOW_REG             0x3C
#define MPU_6050_ACCEL_Y_HIGH_REG            0x3D
#define MPU_6050_ACCEL_Y_LOW_REG             0x3E
#define MPU_6050_ACCEL_Z_HIGH_REG            0x3F
#define MPU_6050_ACCEL_Z_LOW_REG             0x40

#define MPU_6050_TEMP_HIGH_REG               0x41
#define MPU_6050_TEMP_LOW_REG                0x42

#define MPU_6050_GYRO_X_HIGH_REG             0x43
#define MPU_6050_GYRO_X_LOW_REG              0x44
#define MPU_6050_GYRO_Y_HIGH_REG             0x45
#define MPU_6050_GYRO_Y_LOW_REG              0x46
#define MPU_6050_GYRO_Z_HIGH_REG             0x47
#define MPU_6050_GYRO_Z_LOW_REG              0x48

#define MPU_6050_USER_CTRL_REG               0x6A
#define MPU_6050_USER_CTRL_FIFO_EN_MASK      0xBF
#define MPU_6050_USER_CTRL_FIFO_RST_MASK     0xFB

#define MPU_6050_PWR_MGMT_1_REG              0x6B
#define MPU_6050_PWR_MGMT_1_RESET_MASK       0x7F
#define MPU_6050_PWR_MGMT_1_SLEEP_MASK       0xBF
#define MPU_6050_PWR_MGMT_1_CYCLE_MASK       0xDF
#define MPU_6050_PWR_MGMT_1_TEMP_DIS_MASK    0xF7
#define MPU_6050_PWR_MGMT_1_CLKSEL_MASK      0xF8

#define MPU_6050_PWR_MGMT_2_REG              0x6C
#define MPU_6050_PWR_MGMT_2_LP_WAKE_MASK     0x3F
#define MPU_6050_PWR_MGMT_2_ACCEL_STBY_MASK  0xC7
#define MPU_6050_PWR_MGMT_2_GYRO_STBY_MASK   0xF8

#define MPU_6050_FIFO_COUNT_HIGH_REG         0x72
#define MPU_6050_FIFO_COUNT_LOW_REG          0x73

#define MPU_6050_FIFO_DATA_REG               0x74

#define MPU_6050_WHO_AM_I_REG                0x75
*/


struct mpuReadings{
    int16_t accel[3], accelOffset[3] = {0, 0, 0};
    int16_t gyro[3], gyroOffset[3] = {0, 0, 0};
    float temp;
};


class MPU_6050 {
    private:
        //-----------------------------Constants-----------------------------//

        static const uint8_t MPU_6050_DEFAULT_ADDRESS            = 0x68;
        static const uint16_t MPU_6050_FIFO_SIZE                 = 1024;

        static const uint8_t MPU_6050_SAMPLE_RATE_REG            = 0x19;

        static const uint8_t MPU_6050_CONFIG_REG                 = 0x1A;
        static const uint8_t MPU_6050_CONFIG_EXT_SIGNAL_SET_MASK = 0xC7;
        static const uint8_t MPU_6050_CONFIG_DLPF_CFG_MASK       = 0xF8;

        static const uint8_t MPU_6050_GYRO_CFG_REG               = 0x1B;
        static const uint8_t MPU_6050_GYRO_CFG_SELF_TEST_MASK    = 0x1F;
        static const uint8_t MPU_6050_GYRO_CFG_RANGE_SEL_MASK    = 0xE7;

        static const uint8_t MPU_6050_ACCEL_CFG_REG              = 0x1C;
        static const uint8_t MPU_6050_ACCEL_CFG_SELF_TEST_MASK   = 0x1F;
        static const uint8_t MPU_6050_ACCEL_CFG_RANGE_SEL_MASK   = 0xE7;

        static const uint8_t MPU_6050_FIFO_ENABLE_REG            = 0x23;
        static const uint8_t MPU_6050_FIFO_ENABLE_TEMP_MASK      = 0x7F;
        static const uint8_t MPU_6050_FIFO_ENABLE_GYRO_MASK      = 0x8F;
        static const uint8_t MPU_6050_FIFO_ENABLE_ACCEL_MASK     = 0xF7;
        static const uint8_t MPU_6050_FIFO_ENABLE_SLAVE_MASK     = 0xF8;

        static const uint8_t MPU_6050_INT_PIN_CFG_REG            = 0x37;
        static const uint8_t MPU_6050_INT_LEVEL_MASK             = 0x7F;
        static const uint8_t MPU_6050_INT_OPEN_MASK              = 0xBF;
        static const uint8_t MPU_6050_INT_LATCH_MASK             = 0xDF;
        static const uint8_t MPU_6050_INT_READ_CLEAR_MASK        = 0xEF;

        static const uint8_t MPU_6050_INT_ENABLE_REG             = 0x38;
        static const uint8_t MPU_6050_INT_ENABLE_FIFO_OVF_MASK   = 0xEF;
        static const uint8_t MPU_6050_INT_ENABLE_DATA_RDY_MASK   = 0xFE;

        static const uint8_t MPU_6050_INT_STATUS_REG             = 0x3A;
        static const uint8_t MPU_6050_INT_STATUS_FIFO_OVF_MASK   = 0xEF;
        static const uint8_t MPU_6050_INT_STATUS_DATA_RDY_MASK   = 0xFE;

        static const uint8_t MPU_6050_ACCEL_X_HIGH_REG           = 0x3B;
        static const uint8_t MPU_6050_ACCEL_X_LOW_REG            = 0x3C;
        static const uint8_t MPU_6050_ACCEL_Y_HIGH_REG           = 0x3D;
        static const uint8_t MPU_6050_ACCEL_Y_LOW_REG            = 0x3E;
        static const uint8_t MPU_6050_ACCEL_Z_HIGH_REG           = 0x3F;
        static const uint8_t MPU_6050_ACCEL_Z_LOW_REG            = 0x40;

        static const uint8_t MPU_6050_TEMP_HIGH_REG              = 0x41;
        static const uint8_t MPU_6050_TEMP_LOW_REG               = 0x42;

        static const uint8_t MPU_6050_GYRO_X_HIGH_REG            = 0x43;
        static const uint8_t MPU_6050_GYRO_X_LOW_REG             = 0x44;
        static const uint8_t MPU_6050_GYRO_Y_HIGH_REG            = 0x45;
        static const uint8_t MPU_6050_GYRO_Y_LOW_REG             = 0x46;
        static const uint8_t MPU_6050_GYRO_Z_HIGH_REG            = 0x47;
        static const uint8_t MPU_6050_GYRO_Z_LOW_REG             = 0x48;

        static const uint8_t MPU_6050_USER_CTRL_REG              = 0x6A;
        static const uint8_t MPU_6050_USER_CTRL_FIFO_EN_MASK     = 0xBF;
        static const uint8_t MPU_6050_USER_CTRL_FIFO_RST_MASK    = 0xFB;

        static const uint8_t MPU_6050_PWR_MGMT_1_REG             = 0x6B;
        static const uint8_t MPU_6050_PWR_MGMT_1_RESET_MASK      = 0x7F;
        static const uint8_t MPU_6050_PWR_MGMT_1_SLEEP_MASK      = 0xBF;
        static const uint8_t MPU_6050_PWR_MGMT_1_CYCLE_MASK      = 0xDF;
        static const uint8_t MPU_6050_PWR_MGMT_1_TEMP_DIS_MASK   = 0xF7;
        static const uint8_t MPU_6050_PWR_MGMT_1_CLKSEL_MASK     = 0xF8;

        static const uint8_t MPU_6050_PWR_MGMT_2_REG             = 0x6C;
        static const uint8_t MPU_6050_PWR_MGMT_2_LP_WAKE_MASK    = 0x3F;
        static const uint8_t MPU_6050_PWR_MGMT_2_ACCEL_STBY_MASK = 0xC7;
        static const uint8_t MPU_6050_PWR_MGMT_2_GYRO_STBY_MASK  = 0xF8;

        static const uint8_t MPU_6050_FIFO_COUNT_HIGH_REG        = 0x72;
        static const uint8_t MPU_6050_FIFO_COUNT_LOW_REG         = 0x73;

        static const uint8_t MPU_6050_FIFO_DATA_REG              = 0x74;

        static const uint8_t MPU_6050_WHO_AM_I_REG               = 0x75;

        //-----------------------------Atributes-----------------------------//

        I2C* bus;
        uint8_t _address;
        uint16_t queueHead;
        bool FIFOBuffEnabled, gyroFIFOEnabled, accelFIFOEnabled, tempFIFOEnabled;

        //-----------------------------Methods-----------------------------//

        uint16_t FIFOAvailable();
        float convertToCelcius(int16_t rawTemp);

    public:
        MPU_6050();

        // Pseudo Constructors
        void new_MPU_6050();
        void new_MPU_6050(uint8_t address);

        // Power Management
        void wakeUp();
        void reset();

        // Configuration
        void setSampleRate(uint8_t prescaler);
        void configureDLPF(uint8_t bandwidth);
        void configureGyro(uint8_t range);
        void configureAccel(uint8_t range);
        void enableFIFOBuffer();
        void enableTempFIFO();
        void enableGyroFIFO(uint8_t axes);
        void enableAccelFIFO();
        void calibrateModule(mpuReadings* coord);

        void readAccel(mpuReadings* coord);
        void readTemp(mpuReadings* data);
        void readGyro(mpuReadings* coord);

        void readAccelFIFO(mpuReadings* coord);
        void readTempFIFO(mpuReadings* data);
        void readGyroFIFO(mpuReadings* coord);
        void readAllFIFO(mpuReadings* data);
};
#endif /* MPU_6050_h */
