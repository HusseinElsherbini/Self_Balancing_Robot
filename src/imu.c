#include "imu.h"


mpu6050_data_t mpu6050_raw_data = {

    .raw_data = {0}
};

mpu6050_offset_data_t mpu6050_offsets = {

    .ax_offset = 0xEF6D,
    .ay_offset = 0x08BE,
    .az_offset = 0x0366,
    .xg_offset = 0x003F,
    .yg_offset = 0xFFDD,
    .zg_offset = 0x0027,
};

void get_raw_measurements(volatile I2C_t *i2cx ,bool blocking){

    i2cx->data_requested = true;
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_AX_H, &mpu6050_raw_data.raw_data, 14, blocking);

}
void process_raw_measurements(void){
    // reorder data 
    uint8_t temp = 0;
    for(int i = 0; i < (sizeof(mpu6050_raw_data.raw_data)/sizeof(mpu6050_raw_data.raw_data[0])); i+=2){
        temp = mpu6050_raw_data.raw_data[i + 1];
        mpu6050_raw_data.raw_data[i + 1] = mpu6050_raw_data.raw_data[i];
        mpu6050_raw_data.raw_data[i] = temp;
    }
}

void sensorMeanMeasurements(volatile I2C_t *i2cx, long *mean_ax, long *mean_ay, long *mean_az, long *mean_gx, long *mean_gy, long *mean_gz){
    
    int bufferSize = 1000;
    long i=0, buff_ax =0, buff_ay=0, buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
    while(i<(bufferSize+101)){

        get_raw_measurements(i2cx, true);
        process_raw_measurements();
        // discard first 100 measurements
        if(i > 100 && i <= (bufferSize+100)){
            buff_ax+=mpu6050_raw_data.ax;
            buff_ay+=mpu6050_raw_data.ay;
            buff_az+=mpu6050_raw_data.az;
            buff_gx+=mpu6050_raw_data.gx;
            buff_gy+=mpu6050_raw_data.gy;
            buff_gz+=mpu6050_raw_data.gz;
        }
        if (i==(bufferSize+100)){
            *mean_ax=buff_ax/bufferSize;
            *mean_ay=buff_ay/bufferSize;
            *mean_az=buff_az/bufferSize;
            *mean_gx=buff_gx/bufferSize;
            *mean_gy=buff_gy/bufferSize;
            *mean_gz=buff_gz/bufferSize;
        }
        i++;
        delay(2, true);
    }    
}
void calibrateMpu6050(volatile I2C_t *i2cx){

    int acc_deadzone = 8;
    int gyro_deadzone = 1;

    int ax_offset = 0, ay_offset = 0, az_offset = 0, gx_offset = 0, gy_offset = 0, gz_offset = 0;
    long mean_ax=0, mean_ay=0, mean_az=0, mean_gx=0, mean_gy=0, mean_gz=0;

    sensorMeanMeasurements(i2cx, &mean_ax, &mean_ay, &mean_az, &mean_gx, &mean_gy, &mean_gz);


    ax_offset=-mean_ax/8;
    ay_offset=-mean_ay/8;
    az_offset=(16384-mean_az)/8;

    gx_offset=-mean_gx/4;
    gy_offset=-mean_gy/4;
    gz_offset=-mean_gz/4;

    asm("nop");
    setGyroXoffset(i2cx, gx_offset, true);
    setGyroYoffset(i2cx, gy_offset, true);
    setGyroZoffset(i2cx, gz_offset, true);

    setAccXoffset(i2cx, ax_offset, true);
    setAccYoffset(i2cx, ay_offset, true);
    setAccZoffset(i2cx, az_offset, true);
    
    while(1){
        int ready = 0;

        // set offsets 
        setGyroXoffset(i2cx, gx_offset, true);
        setGyroYoffset(i2cx, gy_offset, true);
        setGyroZoffset(i2cx, gz_offset, true);

        setAccXoffset(i2cx, ax_offset, true);
        setAccYoffset(i2cx, ay_offset, true);
        setAccZoffset(i2cx, az_offset, true);

        sensorMeanMeasurements(i2cx, &mean_ax, &mean_ay, &mean_az, &mean_gx, &mean_gy, &mean_gz);

        if(abs(mean_ax) <= acc_deadzone)
        {
            ready++;
            readAccXoffset(i2cx, true);
        }
        else ax_offset = ax_offset-(mean_ax/acc_deadzone);

        if(abs(mean_ay) <= acc_deadzone){ 
            ready++;
            readAccYoffset(i2cx, true);
        }
        else ay_offset = ay_offset-(mean_ay/acc_deadzone);

        if(abs(16834 - mean_az) <= acc_deadzone){
            ready++;
            readAccZoffset(i2cx, true);
        }
        else az_offset = az_offset+((16834 - mean_az)/acc_deadzone);

        if(abs(mean_gx) <= gyro_deadzone){
            ready++;
            readGyroXoffset(i2cx, true);
        }
        else gx_offset = gx_offset-mean_gx/(gyro_deadzone + 1);

        if(abs(mean_gy) <= gyro_deadzone) {
            ready++;   
            readGyroYoffset(i2cx, true);
        }
        else gy_offset = gy_offset- mean_gy/(gyro_deadzone + 1);

        if(abs(mean_gz) <= gyro_deadzone){
            ready++;
            readGyroZoffset(i2cx, true);
        }
        else gz_offset = gz_offset- mean_gz/(gyro_deadzone + 1);

        if (ready == 6) {
            asm("nop");

            setGyroXoffset(i2cx, gx_offset, true);
            setGyroYoffset(i2cx, gy_offset, true);
            setGyroZoffset(i2cx, gz_offset, true);

            setAccXoffset(i2cx, ax_offset, true);
            setAccYoffset(i2cx, ay_offset, true);
            setAccZoffset(i2cx, az_offset, true);

            readGyroXoffset(i2cx, true);
            readGyroYoffset(i2cx, true);
            readGyroZoffset(i2cx, true);

            readAccXoffset(i2cx, true);
            readAccYoffset(i2cx, true);
            readAccZoffset(i2cx, true);
            break;
        }
    }

}

void readGyroConfig(volatile I2C_t *i2cx, bool blocking){

    uint8_t data = 0;
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_G_CFG, &data, 1, blocking);
}

void setAccConfig(volatile I2C_t *i2cx, uint8_t config, bool blocking){

    i2c_write_bits(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_A_CFG, bit_mask(3U, 4U), config, blocking);
}

void readAccConfig(volatile I2C_t *i2cx, bool blocking){

    uint8_t data = 0;
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_A_CFG, &data, 1, blocking);
}

void set_mem_bank(volatile I2C_t *i2cx, uint8_t bank, bool prefetchEnabled, bool userBank, bool blocking){

    uint8_t data = 0;
    bank &= 0x1F;
    if(userBank){
        bank |= 0x20;
    }
    if (prefetchEnabled){

        bank |= 0x40;
    }

    i2c_write(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_BANK_SEL, &bank, 1, blocking);
    //i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_BANK_SEL, &data, 1, true);
}

void set_mem_start_address(volatile I2C_t *i2cx, uint8_t address, bool blocking){
    uint8_t data = 0;

    i2c_write(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_MEM_START_ADDR, &address, 1, blocking);

}

void read_mem_byte(volatile I2C_t *i2cx, uint8_t *data, bool blocking){

    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_MEM_R_W, data, 1, blocking);

}

void checkOTPbankvalidity(volatile I2C_t *i2cx, uint8_t *data, bool blocking){

    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_XG_OFFS_TC, data, 1, blocking);

}

void setOTPbankValid(volatile I2C_t *i2cx, bool enable, bool blocking){

    uint8_t data = 0;
    i2c_write_bit(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_XG_OFFS_TC, 0U, &data, 1, enable, blocking);
}

void setSlaveAddress(volatile I2C_t *i2cx, uint8_t num, uint8_t address, bool blocking){

    uint8_t data = 0;
    if(num > 3){
        return;
    }
    i2c_write(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_I2C_SLV0_ADDR + num*3, &address, 1, blocking);

}

void setI2CMasterMode(volatile I2C_t *i2cx, bool enable, bool blocking){

    uint8_t data = 0;
    i2c_write_bit(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_USER_CTRL, 5U, &data, 1, enable, blocking);
}

void resetI2Cmaster(volatile I2C_t *i2cx, bool blocking){

    uint8_t data = 0;
    i2c_write_bit(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_USER_CTRL, 1U, &data, 1, false, blocking);

}

void resetIMU(volatile I2C_t *i2cx, bool blocking){

    uint8_t data = 0;
    i2c_write_bit(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_PWR_MGMT_1, 7U, &data, 1, true, blocking);

}

void setIMUtoSleepEnable(volatile I2C_t *i2cx, bool enable, bool blocking){

    uint8_t data = 0;
    i2c_write_bit(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_PWR_MGMT_1, 6U, &data, 1, enable, blocking);

}

void setIMUclockSrc(volatile I2C_t *i2cx, uint8_t clkSrc, bool blocking){

    uint8_t data = 0;
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_PWR_MGMT_1, &data, 1, true);
    data |= clkSrc;
    i2c_write(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_PWR_MGMT_1, &data, 1, blocking);

}

void enableDmpInt(volatile I2C_t *i2cx, uint8_t enable, bool blocking){

    uint8_t data = 0;
    i2c_write_bit(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_INTR_EN, 1U, &data, 1U, enable, blocking);
}

void setDMPConfig1(volatile I2C_t *i2cx, uint8_t *data, uint8_t numOfBytes, bool blocking){

    i2c_write(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_DMP_CFG_1, data, numOfBytes, blocking);
}

void setDMPConfig2(volatile I2C_t *i2cx, uint8_t *data, bool blocking){

    i2c_write(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_DMP_CFG_2, data, 1, blocking);
}

void setIMUsmplrt(volatile I2C_t *i2cx, uint8_t rate, bool blocking){

    i2c_write(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_SAMPLE_RATE_DIV, &rate, 1, blocking);
}

void setDLPF(volatile I2C_t *i2cx, uint8_t dlpfRate , bool blocking){

    i2c_write(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_DLFP_CFG, &dlpfRate, 1, blocking);
}

uint16_t readFifoByteCount(volatile I2C_t *i2cx, bool blocking){

    uint8_t data[2] = {0};
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_FIFO_COUNT_H, &data, 2, blocking);

    return (uint16_t)(data[0] << 8 | data[1]);
}

void resetFifo(volatile I2C_t *i2cx, uint8_t enable , bool blocking){

    uint8_t data = 0;
    i2c_write_bit(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_USER_CTRL, 2U, &data, 1, enable, blocking);
}

void enableFifo(volatile I2C_t *i2cx, uint8_t enable , bool blocking){

    uint8_t data = 0;
    i2c_write_bit(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_FIFO_EN, USER_CTRL_FIFO_EN, &data, 1, enable, blocking);
}

void configFifoInterrupt(volatile I2C_t *i2cx, uint8_t *data, bool blocking){

    i2c_write(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_INT_PIN_BYPASS, data, 1, blocking);
}

void configUserControlReg(volatile I2C_t *i2cx, uint8_t *data, bool blocking){

    i2c_write(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_USER_CTRL, data, 1, blocking);
}

void enableMeasurementsToFifo(volatile I2C_t *i2cx, uint8_t *data , bool blocking){

    i2c_write(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_FIFO_EN, data, 1, blocking);
}

void resetDMP(volatile I2C_t *i2cx, uint8_t enable , bool blocking){

    uint8_t data = 0;
    i2c_write_bit(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_USER_CTRL, 3U, &data, 1, enable, blocking);
}

void enableDMP(volatile I2C_t *i2cx, uint8_t enable , bool blocking){

    uint8_t data = 0;
    i2c_write_bit(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_USER_CTRL, 7U, &data, 1, enable, blocking);
}

bool getIntStatus(volatile I2C_t *i2cx, uint8_t enable , bool blocking){

    uint8_t data = 0;
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_INT_STATUS, &data, 1, blocking);
    return (bool)(data & ~(1 << 0));
}

void fullIMUReset(volatile I2C_t *i2cx, bool blocking){
    
    i2c_write_bits(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_USER_CTRL, bit_mask(0U, 2U), 0x00, blocking);
}

void setGyroXoffset(volatile I2C_t *i2cx, int16_t offset, bool blocking){

    uint8_t data[2] = {(uint8_t)(offset >> 8), (uint8_t)(offset)};
    i2c_write(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_XG_OFFS_H, &data, 2, blocking);
}

void setGyroYoffset(volatile I2C_t *i2cx, int16_t offset, bool blocking){

    uint8_t data[2] = {(uint8_t)(offset >> 8), (uint8_t)(offset)};
    i2c_write(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_YG_OFFS_H, &data, 2, blocking);
}

void setGyroZoffset(volatile I2C_t *i2cx, int16_t offset, bool blocking){

    uint8_t data[2] = {(uint8_t)(offset >> 8), (uint8_t)(offset)};
    i2c_write(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_ZG_OFFS_H, &data, 2, blocking);
}

void readGyroXoffset(volatile I2C_t *i2cx, bool blocking){

    uint16_t data = 0;
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_XG_OFFS_H, &data, 2, blocking);
}

void readGyroYoffset(volatile I2C_t *i2cx, bool blocking){

    uint16_t data = 0;
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_YG_OFFS_H, &data, 2, blocking);
}

void readGyroZoffset(volatile I2C_t *i2cx, bool blocking){

    uint16_t data = 0;
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_ZG_OFFS_H, &data, 2, blocking);
}

void setAccXoffset(volatile I2C_t *i2cx, int16_t offset, bool blocking){

    // read reg first, leave 0 bit preserved
    uint8_t data[2] = {0};
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_XA_OFFS_H, &data, 2, blocking); 
    // read 0th bit
    data[1] &= 0x01;
    uint8_t offsetData[2] = {(uint8_t)(offset >> 8), (uint8_t)(offset | data[1])};
    i2c_write(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_XA_OFFS_H, &offsetData, 2, blocking);
}

void setAccYoffset(volatile I2C_t *i2cx, int16_t offset, bool blocking){

    // read reg first, leave 0 bit preserved
    uint8_t data[2] = {0};
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_YA_OFFS_H, &data, 2, blocking); 
    // read 0th bit
    data[1] &= 0x01;
    uint8_t offsetData[2] = {(uint8_t)(offset >> 8), (uint8_t)(offset | data[1])};
    i2c_write(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_YA_OFFS_H, &offsetData, 2, blocking);

}

void setAccZoffset(volatile I2C_t *i2cx, int16_t offset, bool blocking){

    // read reg first, leave 0 bit preserved
    uint8_t data[2] = {0};
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_ZA_OFFS_H, &data, 2, blocking); 
    // read 0th bit
    data[1] &= 0x01;
    uint8_t offsetData[2] = {(uint8_t)(offset >> 8), (uint8_t)(offset | data[1])};
    i2c_write(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_ZA_OFFS_H, &offsetData, 2, blocking);
}

void readAccXoffset(volatile I2C_t *i2cx, bool blocking){

    uint16_t data = 0;
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_XA_OFFS_H, &data, 2, blocking);
}

void readAccYoffset(volatile I2C_t *i2cx, bool blocking){

    uint16_t data = 0;
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_YA_OFFS_H, &data, 2, blocking);
}

void readAccZoffset(volatile I2C_t *i2cx, bool blocking){

    uint16_t data = 0;
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_ZA_OFFS_H, &data, 2, blocking);
}

bool writeMemBlock(volatile I2C_t *i2cx, uint8_t *dmpMem , uint32_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem, bool blocking){
    
   // set the appropriate bank and starting address
    set_mem_bank(i2cx, bank, false, false, true);
    set_mem_start_address(i2cx, address, true);

    uint8_t dataChunkSize;
    uint8_t *verifyBuff = 0;
    uint8_t *progBuff   = 0;
    uint16_t i;
    uint8_t j;

    if(verify){
        verifyBuff = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    }
    if(useProgMem){
        progBuff = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    }


    // make sure not to send more data than what is left
    for(i = 0; i < dataSize;){
        dataChunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        if(i + dataChunkSize > dataSize){
            dataChunkSize = dataSize - i;
        }
    
        // make sure bank boundary isn't exceeded (256 bytes)
        if(dataChunkSize > 256 - address){
            dataChunkSize = 256 - address;
        }

        if(useProgMem){
            // write 16 bytes of memory 
            for(j=0; j < dataChunkSize; j++){

                progBuff[j] = *(dmpMem + i + j);
            }
        }
        else{
            progBuff = (uint8_t *)dmpMem + i;
        }

        i2c_write(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_MEM_R_W, progBuff, dataChunkSize, blocking);

        // verify data sent
        if(verify){

            set_mem_bank(i2cx, bank, false, false, true);
            set_mem_start_address(i2cx, address, true);
        
            i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_MEM_R_W, verifyBuff, dataChunkSize, blocking);

            // verify written data is same as read data, if not, return false
            if (memcmp(progBuff, verifyBuff, dataChunkSize) != 0){

                free(verifyBuff);
                if(useProgMem){
                    free(progBuff);
                }
                return false;
            }

        }
        // increment chunksize by 16 bytes
        i += MPU6050_DMP_MEMORY_CHUNK_SIZE;
        
        // update address by 16 bytes
        address += MPU6050_DMP_MEMORY_CHUNK_SIZE;

        if( i < dataSize){
            if(address == 0){
                bank++;
            }

            set_mem_bank(i2cx, bank, false, false, true);
            set_mem_start_address(i2cx, address, true);                
            
        }
    }
    if(verify){
        free(verifyBuff);
    }
    if(useProgMem){
        free(progBuff);
    }
    return true;   
}

bool writeDMPfw(volatile I2C_t *i2cx){

    return writeMemBlock(i2cx, &dmpMemory, MPU6050_DMP_CODE_SIZE, 0, 0, true, true, true);
}

void imu_init(volatile I2C_t *i2cx){

    uint8_t data = 0;

    // reset mpu
    resetIMU(&i2c1, true);
    
    // delay for 100ms
    delay(100U, true);

    // full signal path reset
    fullIMUReset(&i2c1, true);

    // delay for 100ms
    delay(100U, true);

    // disable sleep 
    setIMUtoSleepEnable(&i2c1, false, true);

     // set clock source PLL_X_GYRO
    setIMUclockSrc(i2cx, PLL_CLKSRC_GYRO_X, true);    
           
    data = 0x00;
    // disable DMP and FIFO overflow interrupt 
    enableMeasurementsToFifo(i2cx, &data, true);    

    data = INTR_CFG_ANY_CLR;
    // set active level for fifo interrpts to HIGH, configure mpu to clear active interrupt flag on fifo reads
    configFifoInterrupt(i2cx, &data, true);
    
     // set sampling rate to 400Hz
    setIMUsmplrt(i2cx, 4U , true);

    // set digital low pass filter to 44 Hz 
    setDLPF(i2cx, DLFP_CFG_FILTER_3 , true);    
    
    // load dmp firmware into memory 
    if(!writeDMPfw(i2cx)){
        while(1);
    }

    // set DMP config
    uint8_t dmpConfigdata[2] = {0x04, 0x00};
    setDMPConfig1(i2cx, &dmpConfigdata, 2, true);

    data = 0xC0;
    // enable and reset FIFO
    configUserControlReg(i2cx, &data, true);

    // reset DMP
    resetDMP(i2cx, true, true);

    // enable dmp int
    enableDmpInt(i2cx, true, true);
    
    // disable dmp initially 
    enableDMP(i2cx, false, true);

    setAccConfig(i2cx, A_CFG_2G, true);
    readAccConfig(i2cx, true);

    // read acc/gyro config
    readGyroConfig(i2cx,true);

    // run calibration routine
    //calibrateMpu6050(i2cx);
    asm("nop");

    // set offsets to pre-determined values 
    setGyroXoffset(i2cx, mpu6050_offsets.xg_offset, true);
    setGyroYoffset(i2cx, mpu6050_offsets.yg_offset, true);
    setGyroZoffset(i2cx, mpu6050_offsets.zg_offset, true);

    setAccXoffset(i2cx, mpu6050_offsets.ax_offset, true);
    setAccYoffset(i2cx, mpu6050_offsets.ay_offset, true);
    setAccZoffset(i2cx, mpu6050_offsets.az_offset, true);

    while(1){
        get_raw_measurements(i2cx ,true);
    }
    asm("nop");

    return 0; // success

}