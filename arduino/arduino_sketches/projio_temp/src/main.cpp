#include <Esmacatshield.h>      //Include Esmacat Arduino Library

Esmacatshield slave(10);        //Define Chip Selector Pin
int counter;

int v[8];                       //EASE 8 registers

void setup() {
  slave.start_spi();            //Start SPI for EASE
  Serial.begin(9600);
}

void loop() {
  
  slave.get_ecat_registers(v);  //read all registers
  // slave.write_reg_value(0,counter++, true);   //Write register data (register,value, led_on)
  // delay(1000);
  // slave.write_reg_value(0,counter++, false);   //Write register data (register,value, led_on)
  delay(1000);
  Serial.println("Next Iteration");
  for (int i=0;i<8;i++)     //read all registers
  {
    Serial.print(i);
    Serial.print("\t");
    Serial.print (v[i]);
    Serial.println();
  }
}
