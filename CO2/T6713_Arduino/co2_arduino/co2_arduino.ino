#include <Wire.h>

// CO2 sensor address
#define slave_addr  0x15

// Function codes 
#define read_reg    0x04
#define write_coil  0x05
#define write_reg   0x06

uint8_t get_status_bytes[5] = {read_reg, 0x13, 0x8A, 0x00, 0x01};
uint8_t get_co2_bytes[5] = {read_reg, 0x13, 0x8B, 0x00, 0x01};


uint16_t get_status() {
  byte rcv_buf[4];
  uint8_t i = 0;

  // Ask sensor for status
  Wire.beginTransmission(slave_addr);
  Wire.write(get_status_bytes, 5);
  Wire.endTransmission();

  delay(10);

  Wire.requestFrom(slave_addr, 5);
  while(Wire.available()) {
    rcv_buf[i] = Wire.read();
  }

  return (rcv_buf[2]<<8)|rcv_buf[3];
}//get_status


uint16_t get_co2_ppm() {
  byte rcv_buf[4];
  uint8_t i = 0;

  // Ask sensor for ppm
  Wire.beginTransmission(slave_addr);
  Wire.write(get_co2_bytes, 5);
  uint8_t val = Wire.endTransmission();

  delay(10);

  Wire.requestFrom(slave_addr, 4);
  while(Wire.available()) {
    rcv_buf[i] = Wire.read();
    i++;
  }

  return (rcv_buf[2]<<8)|rcv_buf[3];
}//get_co2_ppm


void setup() {
  // put your setup code here, to run once:
  Wire.setClock(100000);
  Wire.begin();
  Serial.begin(9600);

//  enable_abc();
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.print("The status is: ");
  Serial.println(get_status());
  Serial.print("CO2 ppm: ");
  Serial.println(get_co2_ppm());
  Serial.println();
  delay(2000);

}
