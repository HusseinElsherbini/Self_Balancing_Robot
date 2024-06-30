#include "i2c.h"
#include "led.h"
#include "gpio.h"

I2C_t i2c1 = {

    .instance                       = (I2C_REGS_t *)I2C1_BASE_ADDRESS,
    .bufferIndex                    = 0,
    .i2c_state                      = I2C_IDLE,
    .data_requested                 = false,
    .isRepeatedStart                = false,
    
};

I2C_CONFIG_t i2c_config = {
    .i2c_regs_base_addr            = (I2C_REGS_t *)I2C1_BASE_ADDRESS,
    .scl_pin_config.port_base_addr = (GPIO_REGS_t *)GPIOB_PERIPH_BASE,
    .scl_pin_config.pin            = 6,
    .sda_pin_config.port_base_addr = (GPIO_REGS_t *)GPIOB_PERIPH_BASE,
    .sda_pin_config.pin            = 7
}; 


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
            if(i2c1.data_requested){
                INTERRUPT_DISABLE()
                i2c1.data_requested = false;
                user_tasks[IMU_RETRIEVE_RAW_DATA].lock = UNLOCKED; 
                INTERRUPT_ENABLE()
            }
        }
        // if byte before last, send NACK
        else if(i2c1.bufferIndex == i2c1.bufferSize - 1){
            i2c1.instance->I2C_CR1 &= ~I2C_CR1_ACK;  // disable ack 
        }
    }
}
void I2C1_EV_IRQHandler(void){
    i2c_ev_IRQhandler();
} 
void I2C1_ER_IRQHandler(void){
    __asm("nop");
    MODIFY_REG(i2c_config.scl_pin_config.port_base_addr->MODER, (3U << 2U*i2c_config.scl_pin_config.pin), (GPIO_OUTPUT_MODE << 2U*i2c_config.scl_pin_config.pin));
    i2c_config.scl_pin_config.port_base_addr->AFRL  &= ~GENERIC_SET_MSK(4U, (4U)*i2c_config.scl_pin_config.pin);

    for(int i = 0; i < 9; i++){

        i2c_config.scl_pin_config.port_base_addr->ODR |= ( 1 << i2c_config.scl_pin_config.pin);
        delay(1, true);
        i2c_config.scl_pin_config.port_base_addr->ODR &= ~( 1 << i2c_config.scl_pin_config.pin);
        delay(1, true);
        
    }
    MODIFY_REG(i2c_config.scl_pin_config.port_base_addr->MODER, (3U << 2U*i2c_config.scl_pin_config.pin), (GPIO_AF_MODE << 2U*i2c_config.scl_pin_config.pin));
    i2c_config.scl_pin_config.port_base_addr->AFRL  |= GENERIC_SET_MSK(4U, (4U)*i2c_config.scl_pin_config.pin);
}

void i2cInit(void){

    
    /* enable apb1 bus clock access ot I2C1 and AHB1 to GPIOB */
    SET_BIT(RCC_REGS->RCC_APB1ENR, I2C1_APB1EN);
    SET_BIT(RCC_REGS->RCC_AHB1ENR, GPIOB_EN);

    /* set GPIO MODE reg to alternate function SCL:PB6 SDA:PB7 */
    i2c_config.scl_pin_config.port_base_addr->MODER |= GPIO_MODE_MSK(i2c_config.scl_pin_config.pin, GPIO_AF_MODE);
    i2c_config.sda_pin_config.port_base_addr->MODER |= GPIO_MODE_MSK(i2c_config.sda_pin_config.pin, GPIO_AF_MODE);

    /* set GPIO output pins to open drain */
    i2c_config.scl_pin_config.port_base_addr->OTYPER |= GENERIC_SET_MSK(GPIO_OUTPUT_OPEN_DRAIN, i2c_config.scl_pin_config.pin);
    i2c_config.sda_pin_config.port_base_addr->OTYPER |= GENERIC_SET_MSK(GPIO_OUTPUT_OPEN_DRAIN, i2c_config.sda_pin_config.pin);  

    /* Set alternate function type to I2C */
    i2c_config.scl_pin_config.port_base_addr->AFRL |= GENERIC_SET_MSK(4U, (4U)*i2c_config.scl_pin_config.pin);
    i2c_config.sda_pin_config.port_base_addr->AFRL |= GENERIC_SET_MSK(4U, (4U)*i2c_config.sda_pin_config.pin);  

    /* set PUPDR register to zero, no pull down or pull up needed for sda and scl */
    i2c_config.scl_pin_config.port_base_addr->PUPDR &= ~(GENERIC_SET_MSK(1U, (2U)*i2c_config.scl_pin_config.pin));
    i2c_config.sda_pin_config.port_base_addr->PUPDR &= ~(GENERIC_SET_MSK(1U, (2U)*i2c_config.sda_pin_config.pin)); 

    /* Reset I2C peripheral */
    SET_BIT(i2c_config.i2c_regs_base_addr->I2C_CR1, (1 << 15U));

    /* Come out of reset mode */
    CLEAR_BIT(i2c_config.i2c_regs_base_addr->I2C_CR1, (1 << 15U));

    /* set i2c mode to sm */
    i2c_config.i2c_regs_base_addr->I2C_CCR &= ~(GENERIC_SET_MSK(0U, 15U));

    /* set peripheral clock frequency */
    i2c_config.i2c_regs_base_addr->I2C_CR2 |=  GENERIC_SET_MSK(42U, 0U);

    /* set CCR freq value */
    i2c_config.i2c_regs_base_addr->I2C_CCR |= GENERIC_SET_MSK(I2C_CCR_VAL, 0U);

    /* set TRISE register max val to CCR_FREQ + 1 */
    i2c_config.i2c_regs_base_addr->I2C_TRISE |= GENERIC_SET_MSK(I2C_SM_TRISE_time, 0U);

    /* enable interrupts I2C_EV, I2C_ER, I2C_BUF*/
   // i2c_config.i2c_regs_base_addr->I2C_CR2 |=GENERIC_SET_MSK(0x7UL, 8U);

    /* enable i2c peripheral */
    i2c_config.i2c_regs_base_addr->I2C_CR1 |= GENERIC_SET_MSK(1U, 0U);

    // if SDA stuck, toggle scl line to get it unstuck
    if(READ_BIT(i2c1.instance->I2C_SR2, (1 << 1))){

        //i2c_config.i2c_regs_base_addr->I2C_CR1 &= ~(1U << 0);
        MODIFY_REG(i2c_config.scl_pin_config.port_base_addr->MODER, (3U << 2U*i2c_config.scl_pin_config.pin), (GPIO_OUTPUT_MODE << 2U*i2c_config.scl_pin_config.pin));
        i2c_config.scl_pin_config.port_base_addr->AFRL  &= ~GENERIC_SET_MSK(4U, (4U)*i2c_config.scl_pin_config.pin);

        for(int i = 0; i < 9; i++){

            i2c_config.scl_pin_config.port_base_addr->ODR |= ( 1 << i2c_config.scl_pin_config.pin);
            delay(1, true);
            i2c_config.scl_pin_config.port_base_addr->ODR &= ~( 1 << i2c_config.scl_pin_config.pin);
            delay(1, true);
            
        }
        MODIFY_REG(i2c_config.scl_pin_config.port_base_addr->MODER, (3U << 2U*i2c_config.scl_pin_config.pin), (GPIO_AF_MODE << 2U*i2c_config.scl_pin_config.pin));
        i2c_config.scl_pin_config.port_base_addr->AFRL  |= GENERIC_SET_MSK(4U, (4U)*i2c_config.scl_pin_config.pin);
        //i2c_config.i2c_regs_base_addr->I2C_CR1 |= GENERIC_SET_MSK(1U, 0U);
    }
/*
    uint8_t buffer[1];
    int length = 1;
    uint8_t address = 0x68; 
    uint16_t reg; 
    i2c1.i2c_regs->I2C_CR1 &= ~(1 << 8);
    i2c1.i2c_regs -> I2C_CR1 |= (1 << 8);
    while(!( i2c1.i2c_regs -> I2C_SR1 & I2C_START)){}
    i2c1.i2c_regs -> I2C_DR = (address << 1) | 0x00;
    while(!(i2c1.i2c_regs -> I2C_SR1 & I2C_ADDR_MATCH)){}
    reg = 0x00; 
    reg = i2c1.i2c_regs -> I2C_SR1;
    i2c_tr[i2c_step] = reg;
    i2c_step+=1;
    reg = i2c1.i2c_regs -> I2C_SR2;
    while(!(i2c1.i2c_regs -> I2C_SR1 & I2C_TxE)){}
    for(int i = 0 ; i < length; i++)
    { 
    i2c1.i2c_regs -> I2C_DR = buffer[i]; 
    while(!(i2c1.i2c_regs -> I2C_SR1 & I2C_TxE)){} 
    while(!(i2c1.i2c_regs -> I2C_SR1 & I2C_BTF)){}  
    reg = 0x00;  
    reg = i2c1.i2c_regs -> I2C_SR1;  
    i2c_tr[i2c_step] = reg;
    i2c_step+=1;
    reg = i2c1.i2c_regs -> I2C_SR2; 
    } 
    i2c1.i2c_regs->I2C_CR1 |= (1 << 9); 
    reg = i2c1.i2c_regs->I2C_SR1; 
    i2c_tr[i2c_step] = reg;
    i2c_step+=1;
    reg = i2c1.i2c_regs->I2C_SR2; */
    /* enable EV and ER irq nvic lines */
    NVIC_ENABLE_IRQ((uint32_t)I2C1_EV_IRQn);
    
    NVIC_ENABLE_IRQ((uint32_t)I2C1_ER_IRQn);

}