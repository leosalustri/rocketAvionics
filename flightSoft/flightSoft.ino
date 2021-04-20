#include <SPI.h>

#define ISM330_CS 10
#define ISM330_SCK 13
#define ISM330_MISO 12
#define ISM330_MOSI 11
#define ISM330_INT_GYRO 24
#define ISM330_INT_ACC 25
#define ISM330_SPI_SPEED 1000000

#define ISM330_REG_INT1_CTRL 0x0D
#define ISM330_REG_INT2_CTRL 0x0E
#define ISM330_REG_WHO_AM_I 0x0F
#define ISM330_REG_CTRL1_XL 0x10
#define ISM330_REG_CTRL2_G 0x11
#define ISM330_REG_CTRL3_C 0x12
#define ISM330_REG_CTRL4_C 0x13
#define ISM330_REG_CTRL5_C 0x14
#define ISM330_REG_CTRL6_C 0x15
#define ISM330_REG_CTRL7_C 0x16
#define ISM330_REG_INTERNAL_FREQ_FINE 0x63



void setup() {

  Serial.begin(512000);
  while(!Serial)
    delay(5);

  pinMode(ISM330_CS, OUTPUT);

  SPI.begin();

  Serial.println("tentativo");
  SPI.beginTransaction(SPISettings(ISM330_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(ISM330_CS, LOW);
  uint8_t buffero[2] = {ISM330_REG_WHO_AM_I | 0b10000000};
  SPI.transfer(buffero, 2);
  digitalWrite(ISM330_CS, HIGH);
  SPI.endTransaction();
  Serial.print("whoAmI: ");
  Serial.println(buffero[1], BIN);

  SPI.beginTransaction(SPISettings(ISM330_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(ISM330_CS, LOW);
  uint8_t lallero[3] = {ISM330_REG_INT1_CTRL | 0b10000000, 0x02, 0x01};
  SPI.transfer(lallero, 3);
  digitalWrite(ISM330_CS, HIGH);
  SPI.endTransaction();
  Serial.print("valori interrupt: ");
  Serial.print(lallero[1], BIN);
  Serial.print("   ");
  Serial.println(lallero[2], BIN);

  SPI.beginTransaction(SPISettings(ISM330_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(ISM330_CS, LOW);
  lallero[0] = ISM330_REG_INT1_CTRL;
  lallero[1] = 0x02;
  lallero[2] = 0x01;
  SPI.transfer(lallero, 3);
  digitalWrite(ISM330_CS, HIGH);
  SPI.endTransaction();

  SPI.beginTransaction(SPISettings(ISM330_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(ISM330_CS, LOW);
  lallero[0] = ISM330_REG_INT1_CTRL | 0b10000000;
  SPI.transfer(lallero, 3);
  digitalWrite(ISM330_CS, HIGH);
  SPI.endTransaction();
  Serial.print("valori interrupt nuovi: ");
  Serial.print(lallero[1], BIN);
  Serial.print("   ");
  Serial.println(lallero[2], BIN);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(5);
}
