#include "Arduino.h"
#include "SPI.h"
#include "Esmacatshield.h"

Esmacatshield::Esmacatshield(int pin)
{
  pinMode(pin, OUTPUT);
  ecat_cs = pin;
  ecat_spi_settings = SPISettings(3000000, MSBFIRST, SPI_MODE1);
}

void Esmacatshield::start_spi(void)
{
  SPI.begin();
}

void Esmacatshield::write_reg_value(int write_addr, int value, bool led_on)
{
  uint8_t v1, v2;
  SPI.beginTransaction(ecat_spi_settings);
  digitalWrite(ecat_cs, LOW);
  delayMicroseconds(500);
  write_addr = write_addr << 3;
  if (led_on)
  {
    SPI.transfer(((WRITE_REG | write_addr) | LED_ON) & SINGLE_SHOT);
  }
  else
  {
    SPI.transfer((WRITE_REG | write_addr) & LED_OFF & SINGLE_SHOT);
  }
  v1 = (value & 0xFF00) >> 8;
  v2 = (value & 0x00FF);
  SPI.transfer(v1);
  SPI.transfer(v2);
  digitalWrite(ecat_cs, HIGH);
  SPI.endTransaction();

  delayMicroseconds(500);
}

int *Esmacatshield::get_ecat_registers(int regs[8])
{
  regs[0] = read_reg_value(1);
  regs[1] = read_reg_value(2);
  regs[2] = read_reg_value(3);
  regs[3] = read_reg_value(4);
  regs[4] = read_reg_value(5);
  regs[5] = read_reg_value(6);
  regs[6] = read_reg_value(7);
  regs[7] = read_reg_value(0);
  return (regs);
}

int Esmacatshield::read_reg_value(int read_addr)
{
  uint16_t v2, v3;
  SPI.beginTransaction(ecat_spi_settings);
  digitalWrite(ecat_cs, LOW);
  delayMicroseconds(500);
  read_addr = read_addr << 3;
  SPI.transfer(READ_REG | read_addr);
  v2 = SPI.transfer(0x00);
  v3 = SPI.transfer(0x00);
  digitalWrite(ecat_cs, HIGH);
  SPI.endTransaction();

  delayMicroseconds(500);
  return (v2 << 8) + v3;
}
