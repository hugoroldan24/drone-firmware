#ifndef I2C_H_
#define I2C_H_

void I2C1_Init(void);
void i2c1_perform_action(uint8_t action, uint8_t addr, uint8_t maddr, int n, uint8_t* data);

#endif /* I2C_H_ */

