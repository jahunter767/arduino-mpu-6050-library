#include "MPU_6050.h"

//************************Constructors************************//
MPU_6050::MPU_6050(){}

// Pseudo Constructors
void MPU_6050::new_MPU_6050(){
    this->_address = MPU_6050_DEFAULT_ADDRESS;
    this->bus = new I2C();
    this->bus->new_I2C(this->_address);

    this->queueHead = 0;
    this->FIFOBuffEnabled = false;
    this->gyroFIFOEnabled = false;
    this->accelFIFOEnabled = false;
    this->tempFIFOEnabled = false;
} // End Constructor

void MPU_6050::new_MPU_6050(uint8_t address){
    this->_address = address;
    this->bus = new I2C();
    this->bus->new_I2C(this->_address);

    this->queueHead = 0;
    this->FIFOBuffEnabled = false;
    this->gyroFIFOEnabled = false;
    this->accelFIFOEnabled = false;
    this->tempFIFOEnabled = false;
} // End Constructor

//************************Private************************//

float MPU_6050::convertToCelcius(int16_t rawTemp){
    return ((rawTemp/340) + 36.53);
} // End-convertToCelcuis

// Returns how much unread bytes are stored in the FIFO buffer
uint16_t MPU_6050::FIFOAvailable(){
    uint16_t temp;
    temp = bus->readByte(MPU_6050_FIFO_COUNT_HIGH_REG);
    temp = temp << 8;
    temp |= bus->readByte(MPU_6050_FIFO_COUNT_LOW_REG);
    return temp;
} // End-available

//************************Public************************//

//-----------Power Management-----------//

void MPU_6050::wakeUp(){
    bus->maskedWriteByte(MPU_6050_PWR_MGMT_1_REG, MPU_6050_PWR_MGMT_1_SLEEP_MASK, 0);
} // End-wakeUp

void MPU_6050::reset(){
    bus->maskedWriteByte(MPU_6050_PWR_MGMT_1_REG, MPU_6050_PWR_MGMT_1_RESET_MASK, ~MPU_6050_PWR_MGMT_1_RESET_MASK);
    delay(250);
} // End-Reset

//-----------Configuration-----------//

void MPU_6050::setSampleRate(uint8_t prescaler){
    bus->writeByte(MPU_6050_SAMPLE_RATE_REG, prescaler);
} // End-setSampleRate

void MPU_6050::configureGyro(uint8_t range){
    bus->maskedWriteByte(MPU_6050_GYRO_CFG_REG, MPU_6050_GYRO_CFG_RANGE_SEL_MASK, range);
} // End-configureGyro

void MPU_6050::configureAccel(uint8_t range){
    bus->maskedWriteByte(MPU_6050_ACCEL_CFG_REG, MPU_6050_ACCEL_CFG_RANGE_SEL_MASK, range);
} // End-configureAccel

void MPU_6050::configureDLPF(uint8_t bandwidth){
    bus->maskedWriteByte(MPU_6050_CONFIG_REG, MPU_6050_CONFIG_DLPF_CFG_MASK, bandwidth);
} // End-configureDLPF

void MPU_6050::enableFIFOBuffer(){
    bus->maskedWriteByte(MPU_6050_USER_CTRL_REG, MPU_6050_USER_CTRL_FIFO_EN_MASK, ~MPU_6050_USER_CTRL_FIFO_EN_MASK);
    FIFOBuffEnabled = true;
} // End-enableFIFOBuffer

void MPU_6050::enableGyroFIFO(uint8_t axes){
    if(FIFOBuffEnabled){
        bus->maskedWriteByte(MPU_6050_FIFO_ENABLE_REG, MPU_6050_FIFO_ENABLE_GYRO_MASK, axes);
        gyroFIFOEnabled = true;
    } // End-If
} // End-enableGyroFIFO

void MPU_6050::enableAccelFIFO(){
    if(FIFOBuffEnabled){
        bus->maskedWriteByte(MPU_6050_FIFO_ENABLE_REG, MPU_6050_FIFO_ENABLE_ACCEL_MASK, ~MPU_6050_FIFO_ENABLE_ACCEL_MASK);
        accelFIFOEnabled = true;
    } // End-If
} // End-enableAccelFIFO

void MPU_6050::enableTempFIFO(){
    if(FIFOBuffEnabled){
        bus->maskedWriteByte(MPU_6050_FIFO_ENABLE_REG, MPU_6050_FIFO_ENABLE_TEMP_MASK, ~MPU_6050_FIFO_ENABLE_TEMP_MASK);
        tempFIFOEnabled = true;
    } // End-If
} // End-enableTempFIFO

void MPU_6050::calibrateModule(mpuReadings* coord){
    int32_t sumAccel[3] = {0, 0, 0}, sumGyro[3] = {0, 0, 0};
    uint8_t count;

    for (count = 0; count < 200; count ++){
        MPU_6050::readAccel(coord);
        sumAccel[0] += (int32_t) coord->accel[0];
        sumAccel[1] += (int32_t) coord->accel[1];
        sumAccel[2] += (int32_t) coord->accel[2];

        MPU_6050::readGyro(coord);
        sumGyro[0] += (int32_t) coord->gyro[0];
        sumGyro[1] += (int32_t) coord->gyro[1];
        sumGyro[2] += (int32_t) coord->gyro[2];
    } // End-For

    coord->accelOffset[0] = sumAccel[0]/200;
    coord->accelOffset[1] = sumAccel[1]/200;
    coord->accelOffset[2] = sumAccel[2]/200;

    coord->gyroOffset[0] = sumGyro[0]/200;
    coord->gyroOffset[1] = sumGyro[1]/200;
    coord->gyroOffset[2] = sumGyro[2]/200;
} // End-calibrateModule

//----------------------------------------------//

// Updates the structure with the raw accelerometer values read by the module
void MPU_6050::readAccel(mpuReadings* coord){
    int16_t temp;
    
    temp = bus->readByte(MPU_6050_ACCEL_X_HIGH_REG);
    temp = temp << 8;
    temp |= bus->readByte(MPU_6050_ACCEL_X_LOW_REG);
    coord->accel[0] = temp - coord->accelOffset[0];

    temp = bus->readByte(MPU_6050_ACCEL_Y_HIGH_REG);
    temp = temp << 8;
    temp |= bus->readByte(MPU_6050_ACCEL_Y_LOW_REG);
    coord->accel[1] = temp - coord->accelOffset[1];

    temp = bus->readByte(MPU_6050_ACCEL_Z_HIGH_REG);
    temp = temp << 8;
    temp |= bus->readByte(MPU_6050_ACCEL_Z_LOW_REG);
    coord->accel[2] = temp - coord->accelOffset[2];
} // End-readAccel

// Updates the structure with the temperature in celcius
void MPU_6050::readTemp(mpuReadings* data){
    int16_t temperature;
    temperature = bus->readByte(MPU_6050_TEMP_HIGH_REG);
    temperature = temperature << 8;
    temperature |= bus->readByte(MPU_6050_TEMP_HIGH_REG);
    data->temp = convertToCelcius(temperature);
} // End-getTemp

// Updates the structure with the raw gyroscope values read by the module
void MPU_6050::readGyro(mpuReadings* coord){
    int16_t temp;

    temp = bus->readByte(MPU_6050_GYRO_X_HIGH_REG);
    temp = temp << 8;
    temp |= bus->readByte(MPU_6050_GYRO_X_LOW_REG);
    coord->gyro[0] = temp - coord->gyroOffset[0];

    temp = bus->readByte(MPU_6050_GYRO_Y_HIGH_REG);
    temp = temp << 8;
    temp |= bus->readByte(MPU_6050_GYRO_Y_LOW_REG);
    coord->gyro[1] = temp - coord->gyroOffset[1];

    temp = bus->readByte(MPU_6050_GYRO_Z_HIGH_REG);
    temp = temp << 8;
    temp |= bus->readByte(MPU_6050_GYRO_Z_LOW_REG);
    coord->gyro[2] = temp - coord->gyroOffset[2];
} // End-readGyro

// Updates the structure with the average of the raw accelerometer readings from FIFO
void MPU_6050::readAccelFIFO(mpuReadings* coord){
    if (accelFIFOEnabled){
        int32_t x = 0, y = 0, z = 0;
        int16_t temp;
        uint16_t count, numBytes = MPU_6050::FIFOAvailable();
        uint8_t stepSize = 6, rawReadings[numBytes];
        
        if (numBytes > 0){
            if (tempFIFOEnabled){
                stepSize += 2;
            } // End-If
            if (gyroFIFOEnabled){
                stepSize += 6;
            } // End-If

            numBytes = numBytes - (numBytes%stepSize); // Ensures that no partial readings are read from FIFO buffer
            bus->readByteStream(MPU_6050_FIFO_DATA_REG, numBytes, rawReadings);

            // Sum the readings from FIFO
            for (count = 0; count < numBytes; count+=stepSize){
                temp = rawReadings[count];
                temp = temp << 8;
                temp |= rawReadings[count+1];
                x += (temp - coord->accelOffset[0]);
                
                temp = rawReadings[count+2];
                temp = temp << 8;
                temp |= rawReadings[count+3];
                y += (temp - coord->accelOffset[1]);
                
                temp = rawReadings[count+4];
                temp = temp << 8;
                temp |= rawReadings[count+5];
                z += (temp - coord->accelOffset[2]);
            } // End-For

            coord->accel[0] = (x/(numBytes/stepSize));
            coord->accel[1] = (y/(numBytes/stepSize));
            coord->accel[2] = (z/(numBytes/stepSize));
        } // End-If
    } // End-IF
} // End-readAccelFIFO

// Computes the average of the temperature values in the FIFO buffer
// then converts it to celcius and updates the structure
void MPU_6050::readTempFIFO(mpuReadings* data){
    if (tempFIFOEnabled){
        int32_t rawTemp;
        int16_t temp;
        uint16_t count, numBytes = MPU_6050::FIFOAvailable();
        uint8_t start = 0, stepSize = 2, rawReadings[numBytes];
        
        if (numBytes > 0){
            if (accelFIFOEnabled){
                stepSize += 6;
                start += 6;
            } // End-If
            if (gyroFIFOEnabled){
                stepSize += 6;
            } // End-If

            numBytes = numBytes - (numBytes%stepSize); // Ensures that no partial readings are read from FIFO buffer
            bus->readByteStream(MPU_6050_FIFO_DATA_REG, numBytes, rawReadings);

            // Sum the readings from FIFO
            for (count = start; count < numBytes; count+=stepSize){
                temp = rawReadings[count];
                temp = temp << 8;
                temp |= rawReadings[count+1];
                rawTemp += temp;
            } // End-For
            
            data->temp = convertToCelcius(rawTemp/(numBytes/stepSize));
        } // End-If
    } // End-IF
} // End-readTempFIFO

// Updates the structure with the average of the raw gyroscope readings from FIFO
void MPU_6050::readGyroFIFO(mpuReadings* coord){
    if (gyroFIFOEnabled){
        int32_t x = 0, y = 0, z = 0;
        int16_t temp;
        uint16_t count, numBytes = MPU_6050::FIFOAvailable();
        uint8_t start = 0, stepSize = 6, rawReadings[numBytes];
        
        if (numBytes > 0){
            if (tempFIFOEnabled){
                start += 2;
                stepSize += 2;
            } // End-If
            if (accelFIFOEnabled){
                start += 6;
                stepSize += 6;
            } // End-If

            numBytes = numBytes - (numBytes%stepSize); // Ensures that no partial readings are read from FIFO buffer
            bus->readByteStream(MPU_6050_FIFO_DATA_REG, numBytes, rawReadings);

            // Sum the readings from FIFO
            for (count = start; count < numBytes; count+=stepSize){
                temp = rawReadings[count];
                temp = temp << 8;
                temp |= rawReadings[count+1];
                x += (temp - coord->gyroOffset[0]);
                
                temp = rawReadings[count+2];
                temp = temp << 8;
                temp |= rawReadings[count+3];
                y += (temp - coord->gyroOffset[1]);
 
                temp = rawReadings[count+4];
                temp = temp << 8;
                temp |= rawReadings[count+5];
                z += (temp - coord->gyroOffset[2]);
            } // End-For

            coord->gyro[0] = (x/(numBytes/stepSize));
            coord->gyro[1] = (y/(numBytes/stepSize));
            coord->gyro[2] = (z/(numBytes/stepSize));
        } // End-If
    } // End-IF
} // End-readGyroFIFO

// Updates structure with average of the raw readings of the gyroscope and accelerometer
// and the average temperature in celcius using the data in the FIFO buffer
void MPU_6050::readAllFIFO(mpuReadings* data){
    if (FIFOBuffEnabled){
        uint16_t numBytes = MPU_6050::FIFOAvailable();
        if (numBytes > 0){
            int32_t ax = 0, ay = 0, az = 0, temperature = 0, gx = 0, gy = 0, gz = 0;
            int16_t temp;
            uint16_t count, i;
            uint8_t stepSize = 0, rawReadings[numBytes], tempOffset = 0, gyroOffset = 0;
            
            if (accelFIFOEnabled){
                stepSize += 6;
                tempOffset += 6;
                gyroOffset += 6;
            } // End-If
            if (tempFIFOEnabled){
                stepSize += 2; 
                gyroOffset += 2;
            } // End-If
            if (gyroFIFOEnabled){
                stepSize += 6;
            } // End-If

            numBytes = numBytes - (numBytes%stepSize); // Ensures that no partial readings are read from FIFO buffer
            bus->readByteStream(MPU_6050_FIFO_DATA_REG, numBytes, rawReadings);
            
            // Sum the readings from FIFO
            for (count = 0; count < numBytes; count+=stepSize){
                if (accelFIFOEnabled){
                    temp = rawReadings[count];
                    temp = temp << 8;
                    temp |= rawReadings[count+1];
                    ax += (temp - data->accelOffset[0]);
                    
                    temp = rawReadings[count+2];
                    temp = temp << 8;
                    temp |= rawReadings[count+3];
                    ay += (temp - data->accelOffset[1]);
                    
                    temp = rawReadings[count+4];
                    temp = temp << 8;
                    temp |= rawReadings[count+5];
                    az += (temp - data->accelOffset[2]);
                } // End-If

                if (tempFIFOEnabled){
                    i = count + tempOffset;
                    temp = rawReadings[i];
                    temp = temp << 8;
                    temp |= rawReadings[i+1];
                    temperature += temp;
                } // End-If

                if (gyroFIFOEnabled){
                    i = count + gyroOffset;
                    temp = rawReadings[i];
                    temp = temp << 8;
                    temp |= rawReadings[i+1];
                    gx += (temp - data->gyroOffset[0]);
                    
                    temp = rawReadings[i+2];
                    temp = temp << 8;
                    temp |= rawReadings[i+3];
                    gy += (temp - data->gyroOffset[1]);
                    
                    temp = rawReadings[i+4];
                    temp = temp << 8;
                    temp |= rawReadings[i+5];
                    gz += (temp - data->gyroOffset[2]);
                } // End-If
            } // End-For

            temp = numBytes/stepSize;

            data->accel[0] = (ax/temp);
            data->accel[1] = (ay/temp);
            data->accel[2] = (az/temp);

            data->temp = MPU_6050::convertToCelcius(temperature/temp);

            data->gyro[0] = (gx/temp);
            data->gyro[1] = (gy/temp);
            data->gyro[2] = (gz/temp);
        } // End-IF
    } // End-If
} // End-readAllFifo
