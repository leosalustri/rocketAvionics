#include <SPI.h>
#include "SdFat.h"
#include "RingBuf.h"

#define SD_CONFIG  SdioConfig(FIFO_SDIO)  // Use Teensy SDIO
#define LOG_FILE_SIZE 10*25000*600  // Size to log 10 byte lines at 25 kHz for more than ten minutes.
#define RING_BUF_CAPACITY 400*512 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
#define LOG_FILENAME "AccelerometerLog.csv"

#define DSO32_CS 10
#define BMP388_CS 19
#define SPI_SCK 13
#define SPI_MISO 12
#define SPI_MOSI 11
#define DSO32_INT_GYRO 40
#define DSO32_INT_ACC 41
#define BMP388_INT 20
#define DSO32_SPI_SPEED 10000000
#define BMP388_SPI_SPEED 10000000

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

#define BMP388_REG_WHO_AM_I 0x00
#define BMP388_REG_STATUS 0x03
#define BMP388_REG_OUTP 0x04 //primo byte pressione, sono 3
#define BMP388_REG_OUTT 0x07 //primo byte temperatura, sono 3
#define BMP388_REG_INT_CTRL 0x19  //01000010
#define BMP388_REG_PWR_CTRL 0x1B  //00110011
#define BMP388_REG_OSR 0x1C  //00001010  oversampling press 2x  temp 2x
#define BMP388_REG_ODR 0x1D  //00000001  100Hz
#define BMP388_REG_CONFIG 0x1F  //00000010 iir filter coeff 1

SdFs sd;
FsFile file;
RingBuf<FsFile, RING_BUF_CAPACITY> rb;  // RingBuf for File type FsFile.

//booleani di appoggio
volatile bool readyGyro = 0;
volatile bool readyAcc = 0;
volatile bool readyBaro = 0;

//variabili per convertire i valori letti dal sensore in SI, il primo valore delle espressioni dipende dalla scala vedi il datasheet
const float dso32BitToRadSec =  70 * (PI/180) / 1000; //valido per 2000dps
const float dso32BitToMss = 0.976 * 9.80665 /1000; //valido per 32g, pare che g a roma sia 9.80353 o 9.80322

float accX, accY, accZ; //in m/ss
float gyroX, gyroY, gyroZ; //in rad/s
int16_t rawAccX, rawAccY, rawAccZ, rawGyroX, rawGyroY, rawGyroZ;
float pressure; //in Pa
float temp; //in C
float alti; //in m
uint32_t rawPressure, rawTemp;
float seaLevelPressure = 101325; // in Pa

//variabili per il timing

unsigned long prevMillis = 0;
bool endLog = 0;
bool logging = 0;
bool gyroReadNotLogged = 0;
bool accReadNotLogged = 0;

struct BMP388_calib_data{
    double par_t1;
    double par_t2;
    double par_t3;
    double par_p1;
    double par_p2;
    double par_p3;
    double par_p4;
    double par_p5;
    double par_p6;
    double par_p7;
    double par_p8;
    double par_p9;
    double par_p10;
    double par_p11;
    double t_lin;
} calData;

void setup() {
  Serial.begin(512000);
  while(!Serial)
    delay(5);

  pinMode(DSO32_CS, OUTPUT);
  pinMode(DSO32_INT_GYRO, INPUT);
  pinMode(DSO32_INT_ACC, INPUT);
  pinMode(BMP388_CS, OUTPUT);
  pinMode(BMP388_INT, INPUT);
  digitalWrite(DSO32_CS, HIGH);
  digitalWrite(BMP388_CS, HIGH);

  
/*
  sd.begin(SD_CONFIG);
  file.open(LOG_FILENAME, O_RDWR | O_CREAT | O_TRUNC);
  file.preAllocate(LOG_FILE_SIZE);
  rb.begin(&file);
 */
 // Initialize the SD.
  if (!sd.begin(SD_CONFIG)) {
    Serial.println("beccato");
    sd.initErrorHalt(&Serial);
  }
  // Open or create file - truncate existing file.
  if (!file.open(LOG_FILENAME, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println("open failed\n");
    return;
  }
  // File must be pre-allocated to avoid huge
  // delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE)) {
     Serial.println("preAllocate failed\n");
     file.close();
     return;
  }
  // initialize the RingBuf.
  rb.begin(&file);
  
  SPI.begin();

  ImuSetup();
  BaroSetup();

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
  //riattivo gli interrupt
  //interrupts();

  //attachInterrupt
  attachInterrupt(digitalPinToInterrupt(DSO32_INT_ACC),IntA,RISING);
  attachInterrupt(digitalPinToInterrupt(DSO32_INT_GYRO),IntG,RISING);
  attachInterrupt(digitalPinToInterrupt(BMP388_INT),IntB,RISING);

  prevMillis = millis();
}

void loop() {
  if(readyGyro){
    readyGyro = 0;
    ImuGetGyro();
    gyroReadNotLogged = 1;
  }
  if(readyAcc){
    readyAcc = 0;
    ImuGetAcc();
    accReadNotLogged = 1;
  }
  if(readyBaro){
    readyBaro = 0;
    BaroGetPress();
    BaroGetTemp();
  }
  logga();

  if(millis() - prevMillis >= 500){
    prevMillis = millis();
    //fai cose
    if(logging){
      Serial.println("sto loggando");
      Serial.println(accZ);
    }
    else{
      Serial.println("ho finito");
    }
  }
  
  if(millis() > 30000){
    logging = 0;
    endLog = 1;
  }
  else if (millis() > 3000){
    logging = 1;
  }
}

/***********************************************************************************************
 * FUNZIONI PER CALCOLI
 */

 float getAltitude(){
  alti = ((float)powf(seaLevelPressure / pressure, 0.190223f) - 1.0f) * (temp + 273.15f) / 0.0065f;
  return alti;
 }

/**********************************************************************************************
 * FUNZIONI PER LA IMU
 */
void ImuSetup(){
  SPI.beginTransaction(SPISettings(DSO32_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(DSO32_CS, LOW);
  byte buffero[2] = {DSO32_REG_WHO_AM_I | 0b10000000};
  SPI.transfer(buffero, 2);
  digitalWrite(DSO32_CS, HIGH);
  SPI.endTransaction();
  delayMicroseconds(500);
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
  //Serial.println("imu settata");
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

void IntG(){
  readyGyro = 1;
}

void IntA(){
  readyAcc = 1;
}

/***********************************************************************************************
 * FUNZIONI PER IL BAROMETRO
 */
 void BaroSetup(){
  SPI.beginTransaction(SPISettings(BMP388_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(BMP388_CS, LOW);
  //impostazioni
  byte settingsBuff[10] = {BMP388_REG_INT_CTRL, 0b01000010, BMP388_REG_PWR_CTRL, 0b00110011,
                            BMP388_REG_OSR, 0b00000011, BMP388_REG_ODR, 0x02, BMP388_REG_CONFIG, 0x02,};
  SPI.transfer(settingsBuff, 10);
  digitalWrite(BMP388_CS, HIGH);
  SPI.endTransaction();

  BaroGetCalibParam();
}

void BaroGetCalibParam(){
  SPI.beginTransaction(SPISettings(BMP388_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(BMP388_CS, LOW);
  byte buff[21];
  SPI.transfer(0x31 | 0b10000000);
  SPI.transfer(0x00);
  SPI.transfer(buff, 21);
  digitalWrite(BMP388_CS, HIGH);
  SPI.endTransaction();

  uint16_t REG_par_t1 = buff[1] << 8 | buff[0];
  uint16_t REG_par_t2 = buff[3] << 8 | buff[2];
  int8_t REG_par_t3 = buff[4];
  int16_t REG_par_p1 = buff[6] << 8 | buff[5];
  int16_t REG_par_p2= buff[8] << 8 | buff[7];
  int8_t REG_par_p3 = buff[9];
  int8_t REG_par_p4 = buff[10];
  uint16_t REG_par_p5 = buff[12] << 8 | buff[11];
  uint16_t REG_par_p6 = buff[14] << 8 | buff[13];
  int8_t REG_par_p7 = buff[15];
  int8_t REG_par_p8 = buff[16];
  int16_t REG_par_p9 = buff[18] << 8 | buff[17];
  int8_t REG_par_p10 = buff[19];
  int8_t REG_par_p11 = buff[20];

  calData.par_t1 = (double)REG_par_t1 / pow(2, -8);
  calData.par_t2 = (double)REG_par_t2 / pow(2, 30);
  calData.par_t3 = (double)REG_par_t3 / pow(2, 48);
  calData.par_p1 = ((double)REG_par_p1 - pow(2, 14)) / pow(2, 20);
  calData.par_p2 = ((double)REG_par_p2 - pow(2, 14)) / pow(2, 29);
  calData.par_p3 = (double)REG_par_p3 / pow(2, 32);
  calData.par_p4 = (double)REG_par_p4 / pow(2, 37);
  calData.par_p5 = (double)REG_par_p5 / pow(2, -3);
  calData.par_p6 = (double)REG_par_p6 / pow(2, 6);
  calData.par_p7 = (double)REG_par_p7 / pow(2, 8);
  calData.par_p8 = (double)REG_par_p8 / pow(2, 15);
  calData.par_p9 = (double)REG_par_p9 / pow(2, 48);
  calData.par_p10 = (double)REG_par_p10 / pow(2, 48);
  calData.par_p11 = (double)REG_par_p11 / pow(2, 65);
}

void BaroGetPress(){
  byte buff[4];
  SPI.beginTransaction(SPISettings(BMP388_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(BMP388_CS, LOW);
  SPI.transfer(BMP388_REG_OUTP | 0b10000000);
  SPI.transfer(buff,4);
  digitalWrite(BMP388_CS, HIGH);
  SPI.endTransaction();
  rawPressure = buff[3] <<16 | (buff[2] << 8 | buff[1]);
  pressure = BMP388_compensate_pressure(rawPressure, &calData);
}

void BaroGetTemp(){
  byte buff[4];
  SPI.beginTransaction(SPISettings(BMP388_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(BMP388_CS, LOW);
  SPI.transfer(BMP388_REG_OUTT | 0b10000000);
  SPI.transfer(buff,4);
  digitalWrite(BMP388_CS, HIGH);
  SPI.endTransaction();
  rawTemp = buff[3] <<16 | (buff[2] << 8 | buff[1]);
  temp = BMP388_compensate_temperature(rawTemp, &calData);
}

static float BMP388_compensate_temperature(uint32_t uncomp_temp, struct BMP388_calib_data *calib_data){
  float partial_data1;
  float partial_data2;
  partial_data1 = (float)(uncomp_temp - calib_data->par_t1);
  partial_data2 = (float)(partial_data1 * calib_data->par_t2);
  calib_data->t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data->par_t3;
  return calib_data->t_lin;
}

static float BMP388_compensate_pressure(uint32_t uncomp_press, struct BMP388_calib_data *calib_data){
  float comp_press;
  float partial_data1;
  float partial_data2;
  float partial_data3;
  float partial_data4;
  float partial_out1;
  float partial_out2;
  partial_data1 = calib_data->par_p6 * calib_data->t_lin;
  partial_data2 = calib_data->par_p7 * (calib_data->t_lin * calib_data->t_lin);
  partial_data3 = calib_data->par_p8 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
  partial_out1 = calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;
  partial_data1 = calib_data->par_p2 * calib_data->t_lin;
  partial_data2 = calib_data->par_p3 * (calib_data->t_lin * calib_data->t_lin);
  partial_data3 = calib_data->par_p4 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
  partial_out2 = (float)uncomp_press * (calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);
  partial_data1 = (float)uncomp_press * (float)uncomp_press;
  partial_data2 = calib_data->par_p9 + calib_data->par_p10 * calib_data->t_lin;
  partial_data3 = partial_data1 *  partial_data2;
  partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calib_data->par_p11;
  comp_press = partial_out1 + partial_out2 + partial_data4;
  return comp_press;
}

void IntB(){
  readyBaro = 1;
}

/***********************************************************************************************
 * FUNZIONI PER IL LOGGING SU SD
 */

void logga(){
  size_t n = rb.bytesUsed();
  if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20) || endLog) { //sostituire a 20 valore sensato
    //Serial.println("File full - quiting.");
    rb.print("fineEEE");
    rb.sync();
    file.truncate();
    file.rewind();
    file.close();
   }
  if(gyroReadNotLogged && accReadNotLogged && logging){
    gyroReadNotLogged = 0;
    accReadNotLogged = 0;
    rb.print(micros());
    rb.print(" , ");
    rb.print(accX);
    rb.print(" , ");
    rb.print(accY);
    rb.print(" , ");
    rb.println(accZ);
  }
  
  if (n >= 512 && !file.isBusy()) {
    // Not busy only allows one sector before possible busy wait.
    // Write one sector from RingBuf to file.
    rb.writeOut(512);
  }
}
