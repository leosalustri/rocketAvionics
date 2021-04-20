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

//variabili per convertire i valori letti dal sensore in SI, il primo valore delle espressioni dipende dalla scala vedi il datasheet
const float dso32BitToRadSec =  140 / (1000 * (PI/180)); //valido per 4000dps
const float dso32BitToMss = 0.488 / (1000 * 9.80665); //valido per 16g, pare che g a roma sia 9.80353 o 9.80322
const float dso32BitToBOH = 0; //da decidere ci serve la temperatura?

float accX; //in m/ss
float accY; //in m/ss
float accZ; //in m/ss
float gyroX; //in rad/s
float gyroY; //in rad/s
float gyroZ; //in rad/s
float temp; //unità misura da decidere

void setup() {

  Serial.begin(512000);
  while(!Serial)
    delay(5);

  pinMode(DSO32_CS, OUTPUT);
  pinMode(DSO32_INT_GYRO, INPUT);
  pinMode(DSO32_INT_ACC, INPUT);

  //attachInterrupt
  attachInterrupt(digitalPinToInterrupt(32),ImuGetAcc,RISING);
  attachInterrupt(digitalPinToInterrupt(31),ImuGetGyro,RISING);
  //disattivo gli interrupt per evitare problemi durante l'inizializzazione
  noInterrupts();
 
  //inizializzo SPI e faccio sapere che verrà usata all'interno degli interrupt
  SPI.begin();
  SPI.usingInterrupt(digitalPinToInterrupt(32));
  SPI.usingInterrupt(digitalPinToInterrupt(31));
  
  ImuSetup();

  Serial.println("tentativo");
  SPI.beginTransaction(SPISettings(DSO32_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(DSO32_CS, LOW);
  uint8_t buffero[2] = {DSO32_REG_WHO_AM_I | 0b10000000};
  SPI.transfer(buffero, 2);
  digitalWrite(DSO32_CS, HIGH);
  SPI.endTransaction();
  Serial.print("whoAmI: ");
  Serial.println(buffero[1], BIN);

  //riattivo gli interrupt
  interrupts();
  
}

void loop() {
  
  if(readyGyro){
    readyGyro = 0;
    ImuGetGyro();
  }
  if(readyAcc){
    readyAcc = 0;
    ImuGetAcc();
  }

  
  delay(5);
}


//imposta i registri in base ai valori desiderati
void ImuSetup(){
  
  SPI.beginTransaction(SPISettings(DSO32_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(DSO32_CS, LOW);
  
  //pin interrupt sensore
  uint8_t intBuff[3] = {DSO32_REG_INT1_CTRL, 0x02, 0x01}; //indirizzo int1, valore int1 gyro, valore int2 acc
  SPI.transfer(intBuff, 3);
  //Data Rates e Full Scale accelerometro e gyro
  uint8_t odrScaleBuff[3] = {DSO32_REG_CTRL1_XL, 0b01110100, 0b01111101}; //vedi CTRL1_XL e CTRL2_G, 833Hz, 16g,acc lpf2 disattivo, 4000dps
  SPI.transfer(odrScaleBuff,3);
  //dataReady mask
  uint8_t dRBuff[2] = {DSO32_REG_CTRL4_C, 0b0001000};
  SPI.transfer(dRBuff, 2);
  
  digitalWrite(DSO32_CS, HIGH);
  SPI.endTransaction();
  
}

void ImuGetGyro(){

  uint8_t buff[6];
  SPI.beginTransaction(SPISettings(DSO32_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(DSO32_CS, LOW);
  
  SPI.transfer(DSO32_REG_OUTX_L_G | 0b10000000);
  SPI.transfer(buff,6);

  digitalWrite(DSO32_CS, HIGH);
  SPI.endTransaction();

  gyroX = (buff[1] << 8 | buff[0]) * dso32BitToRadSec;
  gyroY = (buff[3] << 8 | buff[2]) * dso32BitToRadSec;
  gyroZ = (buff[5] << 8 | buff[4]) * dso32BitToRadSec;
  
}

void ImuGetAcc(){

  uint8_t buff[6];
  SPI.beginTransaction(SPISettings(DSO32_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(DSO32_CS, LOW);
  
  SPI.transfer(DSO32_REG_OUTX_L_A | 0b10000000);
  SPI.transfer(buff,6);

  digitalWrite(DSO32_CS, HIGH);
  SPI.endTransaction();

  accX = (buff[1] << 8 | buff[0]) * dso32BitToMss;
  accY = (buff[3] << 8 | buff[2]) * dso32BitToMss;
  accZ = (buff[5] << 8 | buff[4]) * dso32BitToMss;
  
}

void ImuGetTemp(){

  uint8_t buff[2];
  SPI.beginTransaction(SPISettings(DSO32_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(DSO32_CS, LOW);
  
  SPI.transfer(DSO32_REG_OUT_TEMP_L | 0b10000000);
  SPI.transfer(buff,2);

  digitalWrite(DSO32_CS, HIGH);
  SPI.endTransaction();
  temp = (buff[1] << 8 | buff[0]) * dso32BitToBOH;
  
}

void intG(){
  readyGyro = 1;
}

void intA(){
  readyAcc = 1;
}
