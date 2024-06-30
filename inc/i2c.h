#ifndef I2C_H_
#define I2C_H_
#include "main.h"
#include "stdint.h"
#include "platform.h"
#include "stdbool.h"
#include "cortex.h"
#include "task_scheduler.h"


#define I2C_READ                  (0x1)
#define I2C_BURST_READ            (0x2)
#define I2C_WRITE                 (0x3)

// I2C Status Register 1 (I2C_SR1) bits
#define I2C_SR1_SB          (1U << 0)  // Start Bit
#define I2C_SR1_ADDR        (1U << 1)  // Address Sent/Matched
#define I2C_SR1_BTF         (1U << 2)  // Byte Transfer Finished
#define I2C_SR1_ADD10       (1U << 3)  // 10-bit Header Sent
#define I2C_SR1_STOPF       (1U << 4)  // Stop Detection
#define I2C_SR1_RXNE        (1U << 6)  // Data Register Not Empty
#define I2C_SR1_TXE         (1U << 7)  // Data Register Empty
#define I2C_SR1_BERR        (1U << 8)  // Bus Error
#define I2C_SR1_ARLO        (1U << 9)  // Arbitration Lost
#define I2C_SR1_AF          (1U << 10) // Acknowledge Failure
#define I2C_SR1_OVR         (1U << 11) // Overrun/Underrun
#define I2C_SR1_PECERR      (1U << 12) // PEC Error in Reception
#define I2C_SR1_TIMEOUT     (1U << 14) // Timeout or Tlow Error
#define I2C_SR1_SMBALERT    (1U << 15) // SMBus Alert
#define I2C_CR1_START       (1U << 8)  // Start Generation
#define I2C_CR1_STOP        (1U << 9)  // Stop Generation
#define I2C_CR1_ACK         (1U << 10) // Acknowledge Enable
#define I2C_CR1_POS         (1U << 11) // Acknowledge/PEC Position

// Enumeration of error types
typedef enum {
	MPU6050_ERR_OK,                  // No error
	MPU6050_ERR_PARAM_CFG_FAIL,      // i2c_param_config() error
	MPU6050_ERR_DRIVER_INSTALL_FAIL, // i2c_driver_install() error
	MPU6050_ERR_INVALID_ARGUMENT,    // invalid parameter to function
	MPU6050_ERR_NO_SLAVE_ACK,        // No acknowledgment from slave
	MPU6050_ERR_INVALID_STATE,       // Driver not installed / not i2c master
	MPU6050_ERR_OPERATION_TIMEOUT,   // Bus busy,
	MPU6050_ERR_UNKNOWN,             // Unknown error
	MPU6050_ERR_MAX
} mpu6050_err_t;

typedef enum {
    I2C_IDLE,
    I2C_START_SENT,   
    I2C_REPEATED_START_SENT,  
    ADDR_FLAG_CLEARED,         
    I2C_SLAVE_ADDRESS_SENT_R,
    I2C_SLAVE_ADDRESS_SENT_W,     
    I2C_REGISTER_ADDRESS_SENT,
    I2C_WRITE_IN_PROGRESS,
    I2C_RXNE,             
    I2C_TXE,                  
    I2C_BYTE_TRANSFER_FINISIHED,                

}I2C_STATE_t;

// I2C handle structure
typedef struct {
    I2C_REGS_t* instance;          // I2C instance
    uint8_t *buffer;                // Data buffer
    uint16_t bufferSize;            // Size of the data buffer
    uint16_t bufferIndex;           // Current index in the buffer
    uint16_t slaveAddress;          // Slave address
    uint8_t registerAddress;        // Register address to read from
    uint8_t i2c_op;                 // flag indicating type of transaction
    bool data_requested;
    I2C_STATE_t i2c_state;          // state of current transaction 
    volatile bool isRepeatedStart;  // Flag to indicate repeated start condition
}I2C_t;

extern I2C_t i2c1;


void i2c_send_start(uint32_t *i2c_cr1);
void i2c_read(volatile I2C_t *i2cx, uint16_t imu_address,  uint16_t target_reg, uint8_t *buff, uint8_t numOfBytes, bool blocking);
void i2c_write(volatile I2C_t *i2cx, uint16_t imu_address,  uint16_t target_reg, uint8_t* data, uint8_t numOfBytes, bool blocking);
void i2c_write_bit(volatile I2C_t *i2cx, uint16_t imu_address, uint16_t target_reg, uint8_t bit_no, uint8_t* data, uint8_t numOfBytes, bool enable, bool blocking);
void i2c_write_bits(volatile I2C_t *i2cx, uint16_t imu_address, uint16_t target_reg, uint8_t mask, uint8_t data, bool blocking);
void i2c_ev_IRQhandler(void);

#endif /*I2C_H_*/