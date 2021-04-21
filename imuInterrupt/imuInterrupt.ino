#include <SPI.h>

#define DSO32_CS 10
#define DSO32_SCK 13
#define DSO32_MISO 12
#define DSO32_MOSI 11
#define DSO32_INT_GYRO 31
#define DSO32_INT_ACC 32
#define DSO32_SPI_SPEED 1000000

#define DSO32_REG_INT1_CTRL 0x0D
#define DSO32_REG_INT2_CTRL 0x0E
#define DSO32_REG_WHO_AM_I 0x0F
#define DSO32_REG_CTRL1_XL 0x10
#define DSO32_REG_CTRL2_G 0x11
#define DSO32_REG_CTRL3_C 0x12
#define DSO32_REG_CTRL4_C 0x13
#define DSO32_REG_CTRL5_C 0x14
#define DSO32_REG_CTRL6_C 0x15
#define DSO32_REG_CTRL7_C 0x16
#define DSO32_REG_INTERNAL_FREQ_FINE 0x63

#define DSO32_REG_STATUS 0x1E
#define DSO32_REG_OUT_TEMP_L 0x20 //primo byte temperatura, 2 in totale
#define DSO32_REG_OUTX_L_G 0x22 //primo byte giroscopio, 6 in totale 3x2
#define DSO32_REG_OUTX_L_A 0x28 //primo byte accelerometro, 6 in totale 3x2

//booleani di appoggio
volatile bool readyGyro = 0;
volatile bool readyAcc = 0;
bool gyroRead = 0;
bool accRead = 0;

//variabili per convertire i valori letti dal sensore in SI, il primo valore delle espressioni dipende dalla scala vedi il datasheet
const float dso32BitToRadSec =  70 * (PI/180) / 1000; //valido per 2000dps
const float dso32BitToMss = 0.976 * 9.80665 /1000; //valido per 32g, pare che g a roma sia 9.80353 o 9.80322
const float dso32BitToC = 1/256; //da decidere ci serve la temperatura?

float accX; //in m/ss
float accY; //in m/ss
float accZ; //in m/ss
float gyroX; //in rad/s
float gyroY; //in rad/s
float gyroZ; //in rad/s
float temp; //unità misura da decidere
int16_t rawAccX, rawAccY, rawAccZ, rawGyroX, rawGyroY, rawGyroZ, rawTemp;

//variabili per il timing

unsigned long prevMillis = 0;

void setup() {

  Serial.begin(512000);
  while(!Serial)
    delay(5);

  pinMode(DSO32_CS, OUTPUT);
  pinMode(DSO32_INT_GYRO, INPUT);
  pinMode(DSO32_INT_ACC, INPUT);

  //attachInterrupt
  attachInterrupt(digitalPinToInterrupt(32),IntA,RISING);
  attachInterrupt(digitalPinToInterrupt(31),IntG,RISING);
  //disattivo gli interrupt per evitare problemi durante l'inizializzazione
  noInterrupts();
 
  //inizializzo SPI e faccio sapere che verrà usata all'interno degli interrupt
  SPI.begin();
  //SPI.usingInterrupt(digitalPinToInterrupt(32));
  //SPI.usingInterrupt(digitalPinToInterrupt(31));

  SPI.beginTransaction(SPISettings(DSO32_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(DSO32_CS, LOW);
  uint8_t lallero[3] = {DSO32_REG_INT1_CTRL | 0b10000000, 0x02, 0x01};
  SPI.transfer(lallero, 3);
  digitalWrite(DSO32_CS, HIGH);
  SPI.endTransaction();
  Serial.print("valori interrupt: ");
  Serial.print(lallero[1], BIN);
  Serial.print("   ");
  Serial.println(lallero[2], BIN);
  
  Serial.println("setto imu");
  
  ImuSetup();

  SPI.beginTransaction(SPISettings(DSO32_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(DSO32_CS, LOW);
  uint8_t laller[3] = {DSO32_REG_INT1_CTRL | 0b10000000, 0x02, 0x01};
  SPI.transfer(laller, 3);
  digitalWrite(DSO32_CS, HIGH);
  SPI.endTransaction();
  Serial.print("valori interrupt: ");
  Serial.print(laller[1], BIN);
  Serial.print("   ");
  Serial.println(laller[2], BIN);
  
  SPI.beginTransaction(SPISettings(DSO32_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(DSO32_CS, LOW);
  byte buffero[2] = {DSO32_REG_WHO_AM_I | 0b10000000};
  SPI.transfer(buffero, 2);
  digitalWrite(DSO32_CS, HIGH);
  SPI.endTransaction();
  Serial.print("whoAmI: ");
  Serial.println(buffero[1], BIN);

  //riattivo gli interrupt
  interrupts();

  prevMillis = millis();
}

void loop() {
  
  if(readyGyro){
    readyGyro = 0;
    ImuGetGyro();
    gyroRead = 1;
  }
  if(readyAcc){
    readyAcc = 0;
    ImuGetAcc();
    accRead = 1;
  }

  if(millis() - prevMillis >= 500){

    //logga roba
    //ImuGetAcc();
    Serial.println(String(accX, 6) + " " + String(accY, 6) + " " + String(accZ, 6));
    //Serial.print(gyroRead);
    //Serial.print("  ");
    //Serial.println(accRead);
  }
}


//imposta i registri in base ai valori desiderati
void ImuSetup(){
  
  SPI.beginTransaction(SPISettings(DSO32_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(DSO32_CS, LOW);
  //pin interrupt sensore
  byte intBuff[3] = {DSO32_REG_INT1_CTRL, 0x02, 0x01}; //indirizzo int1, valore int1 gyro, valore int2 acc
  SPI.transfer(intBuff, 3);
  digitalWrite(DSO32_CS, HIGH);

  digitalWrite(DSO32_CS, LOW);
  //Data Rates e Full Scale accelerometro e gyro
  byte odrScaleBuff[3] = {DSO32_REG_CTRL1_XL, 0b01110100, 0b01111100}; //vedi CTRL1_XL e CTRL2_G, 833Hz, 32g,acc lpf2 disattivo, 2000dps
  SPI.transfer(odrScaleBuff,3);
  digitalWrite(DSO32_CS, HIGH);

  digitalWrite(DSO32_CS, LOW);
  //dataReady mask
  byte dRBuff[2] = {DSO32_REG_CTRL4_C, 0b0000000};
  SPI.transfer(dRBuff, 2);
  digitalWrite(DSO32_CS, HIGH);
  
  SPI.endTransaction();
  
}

void ImuGetGyro(){

  byte buff[6];
  SPI.beginTransaction(SPISettings(DSO32_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(DSO32_CS, LOW);
  
  SPI.transfer(DSO32_REG_OUTX_L_G | 0b10000000);
  SPI.transfer(buff,6);

  digitalWrite(DSO32_CS, HIGH);
  SPI.endTransaction();

  rawGyroX = buff[1] << 8 | buff[0];
  rawGyroY = buff[3] << 8 | buff[2];
  rawGyroZ = buff[5] << 8 | buff[4];
  
  gyroX = rawGyroX * dso32BitToRadSec;
  gyroY = rawGyroY * dso32BitToRadSec;
  gyroZ = rawGyroZ * dso32BitToRadSec;
  
}

void ImuGetAcc(){

  byte buff[6];
  SPI.beginTransaction(SPISettings(DSO32_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(DSO32_CS, LOW);
  
  SPI.transfer(DSO32_REG_OUTX_L_A | 0b10000000);
  SPI.transfer(buff,6);

  digitalWrite(DSO32_CS, HIGH);
  SPI.endTransaction();

  rawAccX = buff[1] << 8 | buff[0];
  rawAccY = buff[3] << 8 | buff[2];
  rawAccZ = buff[5] << 8 | buff[4];

  accX = rawAccX * dso32BitToMss;
  accY = rawAccY * dso32BitToMss;
  accZ = rawAccZ * dso32BitToMss;
  
}

void ImuGetTemp(){

  byte buff[2];
  SPI.beginTransaction(SPISettings(DSO32_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(DSO32_CS, LOW);
  
  SPI.transfer(DSO32_REG_OUT_TEMP_L | 0b10000000);
  SPI.transfer(buff,2);

  digitalWrite(DSO32_CS, HIGH);
  SPI.endTransaction();
  rawTemp = buff[1] << 8 | buff[0];
  temp = rawTemp * dso32BitToC;
  
}

void IntG(){
  readyGyro = 1;
  //gyroRead = 1;
}

void IntA(){
  readyAcc = 1;
  //gyroRead = 1;
}
