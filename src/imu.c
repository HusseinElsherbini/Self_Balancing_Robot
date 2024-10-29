#include "imu.h"
#include <math.h>
#include "kalman_filter.h"


mpu6050_data_t mpu6050_data = {

    .rawData.raw_data = {0},
    .processedData.processed_data = {0.0f},
    .filteredData.filtered_data = (0.0f)
};

mpu6050_offset_data_t mpu6050_offsets = {

    .ax_offset = 0xEF79,
    .ay_offset = 0x08C9,
    .az_offset = 0x0325,
    .xg_offset = 0x003A,
    .yg_offset = 0xFFDE,
    .zg_offset = 0x0022,
};

mpu6050_config_t mpu6050_config = {

    .power_management_1.CLKSEL       = PLL_CLKSRC_GYRO_X,  // set clock source to the GYRO x PLL
    .configurationRegs[INT_ENABLE]   = 0x0,                // all interrupts off 
    .configurationRegs[FIFO_CFG]     = 0x0,                // all fifo sources to zero, using DMP fifo
    .acc_cfg.afs_select              = A_CFG_2G,           // accelerometer sensitivity to +-2g
    .interrupt_pin_cfg.int_level     = 0x1,                // logic level for interrupt pin to active low
    .interrupt_pin_cfg.int_rd_clear  = 0x1,                // interrupt status bit is cleared on any read
    .dlpf_cfg.dlpf_config            = DLFP_CFG_FILTER_1,  // digital low pass filter is set to 184hz for acc and 188hz for gyro
    .user_control.fifo_enable        = 0x1,                // enable fifo 
    .user_control.dmp_maybe          = 0x1,                 
    .dmp_config_1.dmp_prg_start_addr = 0x04,               // dmp program start address
    .sample_rate_div.sample_rate_div = 0x0,                // sample rate = Gyro output / (1 + SMPLRATE_DIV)

};
void get_raw_measurements(volatile I2C_t *i2cx ,bool blocking){

    i2cx->data_requested = true;
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_AX_H, &mpu6050_data.rawData.raw_data, 14, blocking);

}
void process_raw_measurements(mpu6050_data_t *mpu6050_data){
    // reorder data 
    uint8_t temp = 0;
    for(int i = 0; i < (sizeof(mpu6050_data->rawData.raw_data)/sizeof(mpu6050_data->rawData.raw_data[0])); i+=2){
        temp = mpu6050_data->rawData.raw_data[i + 1];
        mpu6050_data->rawData.raw_data[i + 1] = mpu6050_data->rawData.raw_data[i];
        mpu6050_data->rawData.raw_data[i] = temp;
    }

}

void convert_raw_data_to_angle(mpu6050_data_t *mpu6050_data, float timeStep){

    // turn raw measurements into angles and angular_velocity 
    mpu6050_data->processedData.processedData.angular_velocity = (float)(mpu6050_data->rawData.rawData.gx/131.0f);
    mpu6050_data->processedData.processedData.acc_angle_x = (180/3.141592) * atan(mpu6050_data->rawData.rawData.ax / sqrt(pow(mpu6050_data->rawData.rawData.ay, 2) + pow(mpu6050_data->rawData.rawData.az, 2))); 
    mpu6050_data->processedData.processedData.gyro_angle_x = mpu6050_data->processedData.processedData.gyro_angle_x + (float)((timeStep/1000.00f)*mpu6050_data->processedData.processedData.angular_velocity);
    mpu6050_data->processedData.processedData.angle = (float)(0.04*mpu6050_data->processedData.processedData.gyro_angle_x + 0.96*mpu6050_data->processedData.processedData.acc_angle_x);

}

void process_dmp_packet(mpu6050_dmp_data_t *mpu6050_dmp_data){
    // reorder data 
    uint8_t temp = 0;

    for(int i = 0; i < 16; i+=4){
        temp = mpu6050_dmp_data->raw_data[i + 3];
        mpu6050_dmp_data->raw_data[i + 3] = mpu6050_dmp_data->raw_data[i];
        mpu6050_dmp_data->raw_data[i] = temp;
        temp = mpu6050_dmp_data->raw_data[i + 1];
        mpu6050_dmp_data->raw_data[i + 1] = mpu6050_dmp_data->raw_data[i];
        mpu6050_dmp_data->raw_data[i] = temp;
    }
    for(int i = 16; i < DMP_PACKET_SIZE; i+=2){
        temp = mpu6050_dmp_data->raw_data[i + 1];
        mpu6050_dmp_data->raw_data[i + 1] = mpu6050_dmp_data->raw_data[i];
        mpu6050_dmp_data->raw_data[i] = temp;
    }
}

void sensorMeanMeasurements(volatile I2C_t *i2cx, long *mean_ax, long *mean_ay, long *mean_az, long *mean_gx, long *mean_gy, long *mean_gz){
    
    int bufferSize = 1000;
    long i=0, buff_ax =0, buff_ay=0, buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
    while(i<(bufferSize+101)){

        get_raw_measurements(i2cx, true);
        process_raw_measurements(&mpu6050_data);
        // discard first 100 measurements
        if(i > 100 && i <= (bufferSize+100)){
            buff_ax+=mpu6050_data.rawData.rawData.ax;
            buff_ay+=mpu6050_data.rawData.rawData.ay;
            buff_az+=mpu6050_data.rawData.rawData.az;
            buff_gx+=mpu6050_data.rawData.rawData.gx;
            buff_gy+=mpu6050_data.rawData.rawData.gy;
            buff_gz+=mpu6050_data.rawData.rawData.gz;
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

    int acc_deadzone = 6;
    int gyro_deadzone = 1;

    int ax_offset = 0, ay_offset = 0, az_offset = 0, gx_offset = 0, gy_offset = 0, gz_offset = 0;
    long mean_ax=0, mean_ay=0, mean_az=0, mean_gx=0, mean_gy=0, mean_gz=0;

    // set offsets to zero values 
    setGyroXoffset(i2cx, (int16_t)0x0000, true);
    setGyroYoffset(i2cx, (int16_t)0x0000, true);
    setGyroZoffset(i2cx, (int16_t)0x0000, true);

    setAccXoffset(i2cx, (int16_t)0x0000, true);
    setAccYoffset(i2cx, (int16_t)0x0000, true);
    setAccZoffset(i2cx, (int16_t)0x0000, true);

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

        if(abs(16384 - mean_az) <= acc_deadzone){
            ready++;
            readAccZoffset(i2cx, true);
        }
        else az_offset = az_offset+((16384 - mean_az)/acc_deadzone);

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

// Custom round function
static inline int16_t custom_round(float x) {
    return (int16_t)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

// Helper function to mimic the 'map' function in Arduino
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



void read_mpu6050_data(volatile I2C_t *i2cx) {
    static int32_t accel_avg[3] = {0}, gyro_avg[3] = {0};
    static int sample_count = 0;
    
    uint8_t buffer[14];
    i2c_read(i2cx, IMU_ADDRESS, REG_AX_H, buffer, 14, true);
    
    int16_t ax = (buffer[0] << 8) | buffer[1];
    int16_t ay = (buffer[2] << 8) | buffer[3];
    int16_t az = (buffer[4] << 8) | buffer[5];
    int16_t gx = (buffer[8] << 8) | buffer[9];
    int16_t gy = (buffer[10] << 8) | buffer[11];
    int16_t gz = (buffer[12] << 8) | buffer[13];
    
    // Apply moving average filter
    accel_avg[0] = (accel_avg[0] * (MOVING_AVERAGE_SAMPLES - 1) + ax) / MOVING_AVERAGE_SAMPLES;
    accel_avg[1] = (accel_avg[1] * (MOVING_AVERAGE_SAMPLES - 1) + ay) / MOVING_AVERAGE_SAMPLES;
    accel_avg[2] = (accel_avg[2] * (MOVING_AVERAGE_SAMPLES - 1) + az) / MOVING_AVERAGE_SAMPLES;
    gyro_avg[0] = (gyro_avg[0] * (MOVING_AVERAGE_SAMPLES - 1) + gx) / MOVING_AVERAGE_SAMPLES;
    gyro_avg[1] = (gyro_avg[1] * (MOVING_AVERAGE_SAMPLES - 1) + gy) / MOVING_AVERAGE_SAMPLES;
    gyro_avg[2] = (gyro_avg[2] * (MOVING_AVERAGE_SAMPLES - 1) + gz) / MOVING_AVERAGE_SAMPLES;
    
    if (++sample_count >= MOVING_AVERAGE_SAMPLES) {
        float ax_g = (float)accel_avg[0] / ACCEL_SENSITIVITY;
        float ay_g = (float)accel_avg[1] / ACCEL_SENSITIVITY;
        float az_g = (float)accel_avg[2] / ACCEL_SENSITIVITY;
        float gx_dps = (float)gyro_avg[0] / GYRO_SENSITIVITY;
        float gy_dps = (float)gyro_avg[1] / GYRO_SENSITIVITY;
        float gz_dps = (float)gyro_avg[2] / GYRO_SENSITIVITY;
                
        sample_count = 0;
    }
}

void readGyroConfig(volatile I2C_t *i2cx, bool blocking){

    uint8_t data = 0;
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_G_CFG, &data, 1, blocking);
}

void setAccConfig(volatile I2C_t *i2cx, uint8_t config, bool blocking){

    i2c_write_bits(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_A_CFG, bit_mask(3U, 4U), config, blocking);
}

void setGyroConfig(volatile I2C_t *i2cx, uint8_t config, bool blocking){

    i2c_write_bits(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_G_CFG, bit_mask(3U, 4U), config, blocking);
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

    if((uint16_t)(data[0] << 8 | data[1]) > 1){
        asm("nop");
    }
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
void readFifoBytes(volatile I2C_t *i2cx, uint8_t *data, uint16_t byteCount, bool blocking){

    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_FIFO, data, byteCount, blocking);
}

bool getDMPpacket(volatile I2C_t *i2cx, uint8_t *data, uint8_t packetSize, bool blocking){

    int16_t fifoCount;
    bool packetReceived = false;

    fifoCount = readFifoByteCount(i2cx, true);

    // check dmp fifo count 
    if(fifoCount > packetSize){

        // if the buffer contains more than 200 bytes, it is faster to just reset the buffer and wait for next packet
        if(fifoCount > 200){
            resetFifo(i2cx, true, true);
            fifoCount = 0;
            return packetReceived;
        }
        // if less than 200 but more than 1, remove all bytes except latest packet
        else{
            uint8_t trash[DMP_PACKET_SIZE];
            uint16_t throwawayBytes;
            while((fifoCount = readFifoByteCount(i2cx, true)) > packetSize){

                fifoCount = fifoCount - packetSize;
                while(fifoCount){

                    if(fifoCount < packetSize){
                        throwawayBytes = packetSize;
                    }
                    else{
                        throwawayBytes = fifoCount;
                    }
                    readFifoBytes(i2cx, &trash, throwawayBytes, true);
                    fifoCount -= throwawayBytes;
                }
            } 
        }
    }
    // if there is no packet received, return false
    packetReceived = fifoCount == packetSize;
    if(!packetReceived){
        return false;
    }
    i2cx->data_requested = true;
    readFifoBytes(i2cx, data, packetSize, blocking);
    return packetReceived;
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

void read_mpu_config(volatile I2C_t *i2cx){

    // read all configuration registers
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_PWR_MGMT_1, &mpu6050_config.configurationRegs[PWR_MGMT_1], 1, true);
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_USER_CTRL, &mpu6050_config.configurationRegs[USER_CTRL], 1, true);
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_INTR_EN, &mpu6050_config.configurationRegs[INT_ENABLE], 1, true);
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_FIFO_CFG, &mpu6050_config.configurationRegs[FIFO_CFG], 1, true);
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_A_CFG, &mpu6050_config.configurationRegs[ACC_CFG], 1, true);
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_INT_PIN_BYPASS, &mpu6050_config.configurationRegs[RA_INT_PIN_BYPASS], 1, true);
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_SAMPLE_RATE_DIV, &mpu6050_config.configurationRegs[SAMPLE_RATE_DIV], 1, true);
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_DLFP_CFG, &mpu6050_config.configurationRegs[DLPF_CFG], 1, true);
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)MPU6050_RA_DMP_CFG_1, &mpu6050_config.configurationRegs[DMP_CFG_1], 1, true);
    i2c_read(i2cx, (uint16_t)IMU_ADDRESS, (uint16_t)REG_G_CFG, &mpu6050_config.configurationRegs[GYRO_CFG], 1, true);

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
    
     // set sampling rate to 200Hz
    setIMUsmplrt(i2cx, 1U , true);

    // set digital low pass filter to 44 Hz 
    setDLPF(i2cx, DLFP_CFG_FILTER_1 , true);    
    
    // load dmp firmware into memory 
    if(!writeDMPfw(i2cx)){
        while(1);
    }
    uint8_t dmpConfigdata[2];
    // set DMP config
    uint16_t startAddress = 1024U;
    dmpConfigdata[0] = startAddress >> 8;
    dmpConfigdata[1] = startAddress & 0xFF;
    
    setDMPConfig1(i2cx, &dmpConfigdata, 2, true);

    data = 0xC0;
    // enable and reset FIFO
    configUserControlReg(i2cx, &data, true);

    // reset DMP
    resetDMP(i2cx, true, true);

    // enable dmp int
    enableDmpInt(i2cx, true, true);
    
    setAccConfig(i2cx, A_CFG_2G, true);
    setGyroConfig(i2cx, 0x3, true);

    asm("nop");

    read_mpu_config(i2cx);

    asm("nop");

    resetFifo(i2cx, 1U, true);
    // disable dmp initially 
    enableDMP(i2cx, 1U, true);
    asm("nop");

    asm("nop");

    resetFifo(i2cx, 1U, true);
    
    //calibrateMpu6050(i2cx);

    // set offsets to pre-determined values 
    setGyroXoffset(i2cx, mpu6050_offsets.xg_offset, true);
    setGyroYoffset(i2cx, mpu6050_offsets.yg_offset, true);
    setGyroZoffset(i2cx, mpu6050_offsets.zg_offset, true);

    setAccXoffset(i2cx, mpu6050_offsets.ax_offset, true);
    setAccYoffset(i2cx, mpu6050_offsets.ay_offset, true);
    setAccZoffset(i2cx, mpu6050_offsets.az_offset, true);

    init_kalman_filter(&kalmanFilter, 0.0f, 5.0f);
    uint32_t time_elapsed = 0;
    uint32_t old_time = 0;
    uint32_t time = 0;
    /*
    while(1){
        time = gTickCount;
        time_elapsed = time - old_time;
        old_time = time;
        get_raw_measurements(i2cx, true);
        process_raw_measurements(&mpu6050_data);
        convert_raw_data_to_angle(&mpu6050_data, (float)time_elapsed);
        delay(1U, true);
    }    
    */
    return 0; // success

}