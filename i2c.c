#include "i2c.h"

void initI2C(void) {
	
	TWBR = (1 << 3); // TWBR = 8 ..... F_CPU / (16+2*(8)*1) = 500kHz SCL period                           
	TWSR = (1 <<TWPS0);
	TWCR |= (1 << TWEN); //enable
}

void i2cWaitForComplete(void) {
	while (!(TWCR &	(1<<TWINT)));
}

void i2cStart(uint8_t address) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	i2cWaitForComplete();
	i2cSend(address << 1);
	i2cWaitForComplete();
}

void i2cStartRead(uint8_t address) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	i2cWaitForComplete();
	i2cSend((address << 1) | 0x01);
	i2cWaitForComplete();
}
void i2cStop(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

uint8_t i2cReadAck(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	i2cWaitForComplete();
	return (TWDR);
}

uint8_t i2cReadNoAck(void) {
	TWCR = (1 << TWINT) | (1 << TWEN);
	i2cWaitForComplete();
	return (TWDR);
}

void i2cSend(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	i2cWaitForComplete();
}