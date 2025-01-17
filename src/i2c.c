#include "i2c.h"
#include "led.h"
#include "gpio.h"
#include "task_macros.h"
#include "SEGGER_RTT.h"
#include "logger.h"

I2C_CONFIG_t i2cConfig = {
    .i2c_regs_base_addr            = (I2C_REGS_t *)I2C1_BASE_ADDRESS,
    .scl_pin_config.port_base_addr = (GPIO_REGS_t *)GPIOB_PERIPH_BASE,
    .scl_pin_config.pin            = 6,
    .sda_pin_config.port_base_addr = (GPIO_REGS_t *)GPIOB_PERIPH_BASE,
    .sda_pin_config.pin            = 7
}; 

I2C_t i2c1 = {

    .instance                       = (I2C_REGS_t *)I2C1_BASE_ADDRESS,
    .i2c_config                     = &i2cConfig,
    .bufferIndex                    = 0,
    .i2c_state                      = I2C_IDLE,
    .new_data_available             = false,
    .isRepeatedStart                = false,
    
};
static TimerHandle_t xI2CRecoveryTimer;
static I2C_Recovery_State recoveryState;

void simulateI2CBusStuck(I2C_t* i2c_handle) {
    // Force SDA low by configuring as output and setting low
    MODIFY_REG(i2c_handle->i2c_config->sda_pin_config.port_base_addr->MODER, 
               (3U << 2U*i2c_handle->i2c_config->sda_pin_config.pin), 
               (GPIO_OUTPUT_MODE << 2U*i2c_handle->i2c_config->sda_pin_config.pin));
    i2c_handle->i2c_config->sda_pin_config.port_base_addr->ODR &= ~(1 << i2c_handle->i2c_config->sda_pin_config.pin);
    
    // Set BUSY bit in SR2
    SET_BIT(i2c_handle->instance->I2C_SR2, (1 << 1));
}

/* Timer callback for toggling SCL */
static void vI2CRecoveryCallback(TimerHandle_t xTimer) {
    if (recoveryState.pulseCount < 9) {
        if (recoveryState.state == 0) {
            // Set SCL high
            recoveryState.port->ODR |= (1 << recoveryState.pin);
            recoveryState.state = 1;
        } else {
            // Set SCL low
            recoveryState.port->ODR &= ~(1 << recoveryState.pin);
            recoveryState.state = 0;
            recoveryState.pulseCount++;
        }
        // Timer will auto-reload for next toggle
    } else {
        // Stop timer after 9 clock pulses
        xTimerStop(xI2CRecoveryTimer, 0);
        
        // Restore I2C alternate function mode
        MODIFY_REG(recoveryState.port->MODER, 
                  (3U << 2U*recoveryState.pin), 
                  (GPIO_AF_MODE << 2U*recoveryState.pin));
        recoveryState.port->AFRL |= GENERIC_SET_MSK(4U, (4U)*recoveryState.pin);
    }
}

/* Function to initialize recovery timer */
void initI2CRecoveryTimer(void) {
    xI2CRecoveryTimer = xTimerCreate(
        "I2CRecovery",
        pdMS_TO_TICKS(1),    // 1ms period
        pdTRUE,              // Auto-reload
        NULL,
        vI2CRecoveryCallback
    );
}

/* Function to start I2C recovery sequence */
void startI2CRecovery(GPIO_REGS_t* port, uint8_t pin) {
    if (xI2CRecoveryTimer == NULL) {
        initI2CRecoveryTimer();
    }
    
    // Set up recovery state
    recoveryState.port = port;
    recoveryState.pin = pin;
    recoveryState.pulseCount = 0;
    recoveryState.state = 0;
    
    // Configure SCL as output
    MODIFY_REG(port->MODER, 
              (3U << 2U*pin), 
              (GPIO_OUTPUT_MODE << 2U*pin));
    port->AFRL &= ~GENERIC_SET_MSK(4U, (4U)*pin);
    
    // Start the recovery timer
    xTimerStart(xI2CRecoveryTimer, 0);
}

/* Modified I2C check and recovery function */
void checkAndRecoverI2C(I2C_t* i2c_handle) {
    if (READ_BIT(i2c_handle->instance->I2C_SR2, (1 << 1))) {
        // Start recovery sequence
        startI2CRecovery(
            i2c_handle->i2c_config->scl_pin_config.port_base_addr,
            i2c_handle->i2c_config->scl_pin_config.pin
        );
        
        // Wait for recovery to complete (9 clock cycles)
        // You might want to add a timeout here
        while (recoveryState.pulseCount < 9) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

void checkAndRecoverI2cBlocking(I2C_t* i2c_handle) {
    // if SDA stuck, toggle scl line to get it unstuck
    if(READ_BIT(i2c_handle->instance->I2C_SR2, (1 << 1))){

        //i2c_config.i2c_regs_base_addr->I2C_CR1 &= ~(1U << 0);
        MODIFY_REG(i2c_handle->i2c_config->scl_pin_config.port_base_addr->MODER, (3U << 2U*i2c_handle->i2c_config->scl_pin_config.pin), (GPIO_OUTPUT_MODE << 2U*i2c_handle->i2c_config->scl_pin_config.pin));
        i2c_handle->i2c_config->scl_pin_config.port_base_addr->AFRL  &= ~GENERIC_SET_MSK(4U, (4U)*i2c_handle->i2c_config->scl_pin_config.pin);

        for(int i = 0; i < 9; i++){

            i2c_handle->i2c_config->scl_pin_config.port_base_addr->ODR |= ( 1 << i2c_handle->i2c_config->scl_pin_config.pin);
            delay(1, true);
            i2c_handle->i2c_config->scl_pin_config.port_base_addr->ODR &= ~( 1 << i2c_handle->i2c_config->scl_pin_config.pin);
            delay(1, true);
            
        }

        // set SDA to Output mode, to issue stop condition
        MODIFY_REG(i2c_handle->i2c_config->sda_pin_config.port_base_addr->MODER, 
                  (3U << 2U*i2c_handle->i2c_config->sda_pin_config.pin), 
                  (GPIO_OUTPUT_MODE << 2U*i2c_handle->i2c_config->sda_pin_config.pin));

        MODIFY_REG(i2c_handle->i2c_config->scl_pin_config.port_base_addr->MODER, (3U << 2U*i2c_handle->i2c_config->scl_pin_config.pin), (GPIO_AF_MODE << 2U*i2c_handle->i2c_config->scl_pin_config.pin));
        i2c_handle->i2c_config->scl_pin_config.port_base_addr->AFRL  |= GENERIC_SET_MSK(4U, (4U)*i2c_handle->i2c_config->scl_pin_config.pin);

        i2c_handle->i2c_config->scl_pin_config.port_base_addr->ODR |= (1 << i2c_handle->i2c_config->scl_pin_config.pin);
        delay(1, true);
        i2c_handle->i2c_config->sda_pin_config.port_base_addr->ODR &= ~(1 << i2c_handle->i2c_config->sda_pin_config.pin);
        delay(1, true);
        i2c_handle->i2c_config->sda_pin_config.port_base_addr->ODR |= (1 << i2c_handle->i2c_config->sda_pin_config.pin);

       // 5. Restore I2C configuration
        // Return both pins to AF mode
        MODIFY_REG(i2c_handle->i2c_config->scl_pin_config.port_base_addr->MODER, 
                  (3U << 2U*i2c_handle->i2c_config->scl_pin_config.pin), 
                  (GPIO_AF_MODE << 2U*i2c_handle->i2c_config->scl_pin_config.pin));
        MODIFY_REG(i2c_handle->i2c_config->sda_pin_config.port_base_addr->MODER, 
                  (3U << 2U*i2c_handle->i2c_config->sda_pin_config.pin), 
                  (GPIO_AF_MODE << 2U*i2c_handle->i2c_config->sda_pin_config.pin));
                  
        i2c_handle->i2c_config->scl_pin_config.port_base_addr->AFRL |= GENERIC_SET_MSK(4U, (4U)*i2c_handle->i2c_config->scl_pin_config.pin);
        i2c_handle->i2c_config->sda_pin_config.port_base_addr->AFRL |= GENERIC_SET_MSK(4U, (4U)*i2c_handle->i2c_config->sda_pin_config.pin);
        
        // 6. Re-enable I2C peripheral
        i2c_handle->instance->I2C_CR1 |= (1U << 0);
        //i2c_config.i2c_regs_base_addr->I2C_CR1 |= GENERIC_SET_MSK(1U, 0U);
    }
}
void i2c_send_start(uint32_t *i2c_cr1){

    *(i2c_cr1) |= (1U << 8U);
}

void i2c_read(volatile I2C_t *i2cx, uint16_t imu_address,  uint16_t target_reg, uint8_t *buff, uint8_t numOfBytes, bool blocking){

    i2c1.slaveAddress       = imu_address;
    i2c1.registerAddress    = target_reg;
    i2c1.buffer             = buff;
    i2c1.bufferSize         = numOfBytes;
    i2c1.bufferIndex        = 0;
    i2c1.isRepeatedStart    = 0;
    //i2c1.R_W        = 0;
    //i2c1.eventCount = 0;
    i2c1.i2c_op             = I2C_READ;
    i2c1.i2c_state          = I2C_START_SENT;

    /* Disable Pos */
    CLEAR_BIT(i2cx->instance->I2C_CR1, I2C_CR1_POS);

    /* send start condition */
    i2c_send_start(&i2cx->instance->I2C_CR1);

    // enable interrupts 
    i2cx->instance->I2C_CR2 |= GENERIC_SET_MSK(0x7UL, 8U);

    if(blocking){
        while(i2cx->i2c_state != I2C_IDLE);
    }

}

void i2c_write(volatile I2C_t *i2cx, uint16_t imu_address, uint16_t target_reg, uint8_t* data, uint8_t numOfBytes, bool blocking){

    i2c1.slaveAddress     = imu_address;
    i2c1.registerAddress  = target_reg;
    i2c1.bufferSize       = numOfBytes;
    i2c1.bufferIndex      = 0;
    i2c1.isRepeatedStart  = 0;
    //i2c1.R_W        = 0;
    i2c1.buffer           = data;
    //i2c1.eventCount = 0;
    i2c1.i2c_op           = I2C_WRITE;
    i2c1.i2c_state        = I2C_START_SENT;

        /* Disable Pos */
    CLEAR_BIT(i2cx->instance->I2C_CR1, I2C_CR1_POS);
    // send start condition 
    i2c_send_start(&i2cx->instance->I2C_CR1);

    // enable interrupts 
    i2cx->instance->I2C_CR2 |= GENERIC_SET_MSK(0x7UL, 8U);

    if(blocking){
        while(i2cx->i2c_state != I2C_IDLE);
    }
}

void i2c_write_bit(volatile I2C_t *i2cx, uint16_t imu_address, uint16_t target_reg, uint8_t bit_no, uint8_t* data, uint8_t numOfBytes, bool enable, bool blocking){

    i2c_read(i2cx, (uint16_t)imu_address, (uint16_t)target_reg, data, 1, true);
    if(enable){
        *data |= (1U << bit_no);
    }
    else{
        *data &= ~(1 << bit_no);
    }
    i2c_write(i2cx, (uint16_t)imu_address, (uint16_t)target_reg, data, 1, blocking);
  
}

void i2c_write_bits(volatile I2C_t *i2cx, uint16_t imu_address, uint16_t target_reg, uint8_t mask, uint8_t data, bool blocking){

    uint8_t regData;
    i2c_read(i2cx, (uint16_t)imu_address, (uint16_t)target_reg, &regData, 1, true);
    
    // clear masked bits 
    regData &= ~mask;

    // set new value
    regData |= data;

    i2c_write(i2cx, (uint16_t)imu_address, (uint16_t)target_reg, &data, 1, blocking);
  
}

// I2C interrupt handler
void i2c_ev_IRQhandler(void) {

    volatile BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t sr1 = i2c1.instance->I2C_SR1;
    uint32_t sr2 = i2c1.instance->I2C_SR2;

    // Start condition generated
    if (sr1 & I2C_SR1_SB) {
        if (i2c1.i2c_state == I2C_REPEATED_START_SENT) {
            // Send slave address with read bit
            i2c1.instance->I2C_DR = (i2c1.slaveAddress << 1) | 1;
            i2c1.isRepeatedStart = 0;
            i2c1.i2c_state = I2C_SLAVE_ADDRESS_SENT_R;
        } else {
            // Send slave address with write bit
            i2c1.instance->I2C_DR = (i2c1.slaveAddress << 1) | 0;
            i2c1.i2c_state = I2C_SLAVE_ADDRESS_SENT_W;
        }
    }

    // Address sent
    else if (sr1 & I2C_SR1_ADDR) {
        if (i2c1.i2c_state == I2C_SLAVE_ADDRESS_SENT_W) {
            // Clear ADDR flag by reading SR1 and SR2
                (void)i2c1.instance->I2C_SR1;
                (void)i2c1.instance->I2C_SR2;
                i2c1.i2c_state = ADDR_FLAG_CLEARED;

        } 
        else if(i2c1.i2c_state == I2C_SLAVE_ADDRESS_SENT_R) {

            if(i2c1.bufferSize > 1){
                // Enable ACK for multiple byte read
                i2c1.instance->I2C_CR1 |= I2C_CR1_ACK;                
            }
            else{
                // disable Ack and send stop for single byte read 
                i2c1.instance->I2C_CR1 &= ~I2C_CR1_ACK;
                i2c1.instance->I2C_CR1 |= I2C_CR1_STOP;
            }
        }
    }

    // Transmit interrupt (TXE) for write operation
    else if(sr1 & I2C_SR1_TXE){
        // if event count is 2, send target register
        if(i2c1.i2c_state == ADDR_FLAG_CLEARED){
            i2c1.instance->I2C_DR = i2c1.registerAddress;  
            i2c1.i2c_state = I2C_REGISTER_ADDRESS_SENT;
        }
        // if target register sent, and this is a read op, generate restart.
        else if(i2c1.i2c_state == I2C_REGISTER_ADDRESS_SENT){

            if(i2c1.i2c_op == I2C_READ){
                i2c1.isRepeatedStart = 1;
                i2c1.instance->I2C_CR1 |= I2C_CR1_START;
                i2c1.i2c_state = I2C_REPEATED_START_SENT;               
            }
            else if(i2c1.i2c_op == I2C_WRITE){

                i2c1.i2c_state = I2C_WRITE_IN_PROGRESS;
            }
        }
        if(i2c1.i2c_state == I2C_WRITE_IN_PROGRESS){
            if(i2c1.bufferIndex == i2c1.bufferSize){

                i2c1.instance->I2C_CR2 &= ~(GENERIC_SET_MSK(0x7UL, 8U)); // disable interrupts
                i2c1.instance->I2C_CR1 |= I2C_CR1_STOP; // send stop
                i2c1.i2c_state = I2C_IDLE;    
                i2c1.i2c_op = 0;                        
            }
            else{
                i2c1.instance->I2C_DR = *(i2c1.buffer++); 
                i2c1.bufferIndex++;
            }   
        }
    }

    // Receive interrupt (RXNE) for read operation
    else if (sr1 & I2C_SR1_RXNE) {
        
        i2c1.buffer[i2c1.bufferIndex++] = i2c1.instance->I2C_DR;
        // if number of bytes to read left is one, disable ack, disable interrupts, 

        if(i2c1.bufferIndex == i2c1.bufferSize){

            if(i2c1.bufferSize > 1){
                // generate stop after recceiving last byte 
                i2c1.instance->I2C_CR1 |= I2C_CR1_STOP; // send stop
            }
            i2c1.i2c_state = I2C_IDLE;
            
             // Notify task if it's waiting for data
            if(xBalancingTaskHandle != NULL) {

                //i2c1.new_data_available = true;
                xTaskNotifyFromISR(xBalancingTaskHandle,
                                NOTIFY_IMU_DATA,
                                eSetBits,
                                &xHigherPriorityTaskWoken);
                //vTaskNotifyGiveFromISR(xBalancingTaskHandle, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
        // if byte before last, send NACK
        else if(i2c1.bufferIndex == i2c1.bufferSize - 1){
            i2c1.instance->I2C_CR1 &= ~I2C_CR1_ACK;  // disable ack 
        }
    }
}
void I2C1_EV_IRQHandler(void){
    DEBUG_ISR_ENTER();
    i2c_ev_IRQhandler();
    DEBUG_ISR_EXIT();
} 
void I2C1_ER_IRQHandler(void){
    DEBUG_ISR_ENTER();
    checkAndRecoverI2C(&i2c1);
    DEBUG_ISR_EXIT();
}

void i2cInit(I2C_t* i2c_handle){

    
    /* enable apb1 bus clock access to I2C1 and AHB1 to GPIOB */
    SET_BIT(RCC_REGS->RCC_APB1ENR, I2C1_APB1EN);
    SET_BIT(RCC_REGS->RCC_AHB1ENR, GPIOB_EN);

    /* set GPIO MODE reg to alternate function SCL:PB6 SDA:PB7 */
    i2c_handle->i2c_config->scl_pin_config.port_base_addr->MODER |= GPIO_MODE_MSK(i2c_handle->i2c_config->scl_pin_config.pin, GPIO_AF_MODE);
    i2c_handle->i2c_config->sda_pin_config.port_base_addr->MODER |= GPIO_MODE_MSK(i2c_handle->i2c_config->sda_pin_config.pin, GPIO_AF_MODE);

    /* set GPIO output pins to open drain */
    i2c_handle->i2c_config->scl_pin_config.port_base_addr->OTYPER |= GENERIC_SET_MSK(GPIO_OUTPUT_OPEN_DRAIN, i2c_handle->i2c_config->scl_pin_config.pin);
    i2c_handle->i2c_config->sda_pin_config.port_base_addr->OTYPER |= GENERIC_SET_MSK(GPIO_OUTPUT_OPEN_DRAIN, i2c_handle->i2c_config->sda_pin_config.pin);  

    /* Set alternate function type to I2C */
    i2c_handle->i2c_config->scl_pin_config.port_base_addr->AFRL |= GENERIC_SET_MSK(4U, (4U)*i2c_handle->i2c_config->scl_pin_config.pin);
    i2c_handle->i2c_config->sda_pin_config.port_base_addr->AFRL |= GENERIC_SET_MSK(4U, (4U)*i2c_handle->i2c_config->sda_pin_config.pin);  

    /* set PUPDR register to zero, no pull down or pull up needed for sda and scl */
    i2c_handle->i2c_config->scl_pin_config.port_base_addr->PUPDR &= ~(GENERIC_SET_MSK(1U, (2U)*i2c_handle->i2c_config->scl_pin_config.pin));
    i2c_handle->i2c_config->sda_pin_config.port_base_addr->PUPDR &= ~(GENERIC_SET_MSK(1U, (2U)*i2c_handle->i2c_config->sda_pin_config.pin)); 

    /* Reset I2C peripheral */
    SET_BIT(i2c_handle->i2c_config->i2c_regs_base_addr->I2C_CR1, (1 << 15U));

    /* Come out of reset mode */
    CLEAR_BIT(i2c_handle->i2c_config->i2c_regs_base_addr->I2C_CR1, (1 << 15U));

    /* set i2c mode to sm */
    i2c_handle->i2c_config->i2c_regs_base_addr->I2C_CCR &= ~(GENERIC_SET_MSK(0U, 15U));

    /* set peripheral clock frequency */
    i2c_handle->i2c_config->i2c_regs_base_addr->I2C_CR2 |=  GENERIC_SET_MSK(42U, 0U);

    /* set CCR freq value */
    i2c_handle->i2c_config->i2c_regs_base_addr->I2C_CCR |= GENERIC_SET_MSK(I2C_CCR_VAL, 0U);

    /* set TRISE register max val to CCR_FREQ + 1 */
    i2c_handle->i2c_config->i2c_regs_base_addr->I2C_TRISE |= GENERIC_SET_MSK(I2C_SM_TRISE_time, 0U);

    /* enable interrupts I2C_EV, I2C_ER, I2C_BUF*/
   // i2c_config.i2c_regs_base_addr->I2C_CR2 |=GENERIC_SET_MSK(0x7UL, 8U);

    /* enable i2c peripheral */
    i2c_handle->i2c_config->i2c_regs_base_addr->I2C_CR1 |= GENERIC_SET_MSK(1U, 0U);

    //simulateI2CBusStuck(i2c_handle); // used to simulate stuck bus for testing

    checkAndRecoverI2cBlocking(i2c_handle);

    __NVIC_SetPriority(I2C1_EV_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    __NVIC_SetPriority(I2C1_ER_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    __NVIC_EnableIRQ((uint32_t)I2C1_EV_IRQn);
    
    __NVIC_EnableIRQ((uint32_t)I2C1_ER_IRQn);

}