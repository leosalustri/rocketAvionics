#include <SPI.h>

#define BMP388_CS 19
#define BMP388_SCK 13
#define BMP388_MISO 12
#define BMP388_MOSI 11
#define BMP388_INT 20
#define BMP388_SPI_SPEED 1000000

#define BMP388_REG_INT1_CTRL 0x0D

//booleani di appoggio
volatile bool readyBaro = 0;
bool baroRead = 0;

//variabili per convertire i valori letti dal sensore in SI, il primo valore delle espressioni dipende dalla scala vedi il datasheet
const float BMP388BitPascal =  ; 
const float BMP388BitToC = ; 
float pressure; //in Pa
float temp; //unità misura da decidere
int16_t rawPressure, rawTemp;

//variabili per il timing

unsigned long prevMillis = 0;

void setup() {

  Serial.begin(512000);
  while(!Serial)
    delay(5);

  pinMode(BMP388_CS, OUTPUT);
  pinMode(BMP388_INT, INPUT);

  //attachInterrupt
  attachInterrupt(digitalPinToInterrupt(BMP388_INT),IntB,RISING);
  //disattivo gli interrupt per evitare problemi durante l'inizializzazione
  noInterrupts();
 
  //inizializzo SPI e faccio sapere che verrà usata all'interno degli interrupt
  SPI.begin();

  SPI.beginTransaction(SPISettings(BMP388_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(BMP388_CS, LOW);
  uint8_t lallero[3] = {BMP388_REG_INT1_CTRL | 0b10000000, 0x02, 0x01};
  SPI.transfer(lallero, 3);
  digitalWrite(BMP388_CS, HIGH);
  SPI.endTransaction();
  Serial.print("valori interrupt: ");
  Serial.print(lallero[1], BIN);
  Serial.print("   ");
  Serial.println(lallero[2], BIN);
  
  Serial.println("setto imu");
  
  BaroSetup();

  SPI.beginTransaction(SPISettings(BMP388_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(BMP388_CS, LOW);
  uint8_t laller[3] = {BMP388_REG_INT1_CTRL | 0b10000000, 0x02, 0x01};
  SPI.transfer(laller, 3);
  digitalWrite(BMP388_CS, HIGH);
  SPI.endTransaction();
  Serial.print("valori interrupt: ");
  Serial.print(laller[1], BIN);
  Serial.print("   ");
  Serial.println(laller[2], BIN);
  
  SPI.beginTransaction(SPISettings(BMP388_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(BMP388_CS, LOW);
  byte buffero[2] = {BMP388_REG_WHO_AM_I | 0b10000000};
  SPI.transfer(buffero, 2);
  digitalWrite(BMP388_CS, HIGH);
  SPI.endTransaction();
  Serial.print("whoAmI: ");
  Serial.println(buffero[1], BIN);

  //riattivo gli interrupt
  interrupts();

  prevMillis = millis();
}

void loop() {
  
  if(readyBaro){
    readyBaro = 0;
    BaroGetPress();
    BaroGetTemp();
    gyroRead = 1;
  }

  if(millis() - prevMillis >= 500){

    //logga roba
    Serial.println(String(pressure, 4) + " " + String(temp, 4));
    //Serial.println(baroRead);
  }
}


//imposta i registri in base ai valori desiderati
void BaroSetup(){
  
  SPI.beginTransaction(SPISettings(BMP388_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(BMP388_CS, LOW);
  //pin interrupt sensore
  byte intBuff[3] = {BMP388_REG_INT1_CTRL, 0x02, 0x01}; //indirizzo int1, valore int1 gyro, valore int2 acc
  SPI.transfer(intBuff, 3);
  digitalWrite(BMP388_CS, HIGH);

  digitalWrite(BMP388_CS, LOW);
  //Data Rates e Full Scale accelerometro e gyro
  byte odrScaleBuff[3] = {BMP388_REG_CTRL1_XL, 0b01110100, 0b01111100}; //vedi CTRL1_XL e CTRL2_G, 833Hz, 32g,acc lpf2 disattivo, 2000dps
  SPI.transfer(odrScaleBuff,3);
  digitalWrite(BMP388_CS, HIGH);

  digitalWrite(BMP388_CS, LOW);
  //dataReady mask
  byte dRBuff[2] = {BMP388_REG_CTRL4_C, 0b0000000};
  SPI.transfer(dRBuff, 2);
  digitalWrite(BMP388_CS, HIGH);
  
  SPI.endTransaction();
  
}

void BaroGetPress(){

  byte buff[2];
  SPI.beginTransaction(SPISettings(BMP388_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(BMP388_CS, LOW);
  
  SPI.transfer(BMP388_REG_OUT_TEMP_L | 0b10000000);
  SPI.transfer(buff,2);

  digitalWrite(BMP388_CS, HIGH);
  SPI.endTransaction();
  rawTemp = buff[1] << 8 | buff[0];
  temp = rawTemp * BMP388BitToC;
  
}

void BaroGetTemp(){

  byte buff[2];
  SPI.beginTransaction(SPISettings(BMP388_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(BMP388_CS, LOW);
  
  SPI.transfer(BMP388_REG_OUT_TEMP_L | 0b10000000);
  SPI.transfer(buff,2);

  digitalWrite(BMP388_CS, HIGH);
  SPI.endTransaction();
  rawTemp = buff[1] << 8 | buff[0];
  temp = rawTemp * BMP388BitToC;
  
}

void IntB(){
  readyBaro = 1;
  //baroRead = 1;
}
