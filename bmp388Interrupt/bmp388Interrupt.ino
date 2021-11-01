#include <SPI.h>

#define BMP388_CS 19
#define BMP388_SCK 13
#define BMP388_MISO 12
#define BMP388_MOSI 11
#define BMP388_INT 20
#define BMP388_SPI_SPEED 1000000

#define BMP388_REG_WHO_AM_I 0x00
#define BMP388_REG_STATUS 0x03
#define BMP388_REG_OUTP 0x04 //primo byte pressione, sono 3
#define BMP388_REG_OUTT 0x07 //primo byte temperatura, sono 3

#define BMP388_REG_INT_CTRL 0x19  //01000010
#define BMP388_REG_PWR_CTRL 0x1B  //00110011
#define BMP388_REG_OSR 0x1C  //00001010  oversampling press 2x  temp 2x
#define BMP388_REG_ODR 0x1D  //00000001  100Hz
#define BMP388_REG_CONFIG 0x1F  //00000010 iir filter coeff 1


//booleani di appoggio
volatile bool readyBaro = 0;
bool baroRead = 0;
volatile int baroCount = 0;

float pressure; //in Pa
float temp; //in C
uint32_t rawPressure, rawTemp;

//variabili per il timing

unsigned long prevMillis = 0;

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
  
};

struct BMP388_calib_data calData;

void setup() {

  Serial.begin(512000);
  while(!Serial)
    delay(5);

  pinMode(BMP388_CS, OUTPUT);
  pinMode(BMP388_INT, INPUT);
  digitalWrite(BMP388_CS, HIGH);

  //attachInterrupt
  attachInterrupt(digitalPinToInterrupt(BMP388_INT),IntB,RISING);
  //disattivo gli interrupt per evitare problemi durante l'inizializzazione
  noInterrupts();
 
  //inizializzo SPI e faccio sapere che verrà usata all'interno degli interrupt
  SPI.begin();

//  SPI.beginTransaction(SPISettings(BMP388_SPI_SPEED, MSBFIRST, SPI_MODE3));
//  digitalWrite(BMP388_CS, LOW);
//  byte buffe[3] = {BMP388_REG_WHO_AM_I | 0b10000000, 0b1000};
//  SPI.transfer(buffe, 3);
//  digitalWrite(BMP388_CS, HIGH);
//  SPI.endTransaction();
//  Serial.print("whoAmI: ");
//  Serial.println(buffe[2], HEX);

//  SPI.beginTransaction(SPISettings(BMP388_SPI_SPEED, MSBFIRST, SPI_MODE3));
//  digitalWrite(BMP388_CS, LOW);
//  byte buff[3] = {BMP388_REG_WHO_AM_I | 0b10000000, 0b1000};
//  SPI.transfer(buff, 3);
//  digitalWrite(BMP388_CS, HIGH);
//  SPI.endTransaction();
//  Serial.print("whoAmI: ");
//  Serial.println(buff[2], HEX);

  Serial.println("setto imu");
  
  BaroSetup();
  
//  SPI.beginTransaction(SPISettings(BMP388_SPI_SPEED, MSBFIRST, SPI_MODE3));
//  digitalWrite(BMP388_CS, LOW);
//  byte buffero[3] = {BMP388_REG_WHO_AM_I | 0b10000000, 0b1000};
//  SPI.transfer(buffero, 3);
//  digitalWrite(BMP388_CS, HIGH);
//  SPI.endTransaction();
//  Serial.print("whoAmI: ");
//  Serial.println(buffero[2], HEX);

  //riattivo gli interrupt
  interrupts();

  prevMillis = millis();
}

void loop() {
  
  if(readyBaro){
    readyBaro = 0;
    BaroGetPress();
    BaroGetTemp();
    baroRead = 1;
  }

  if(millis() - prevMillis >= 500){

    prevMillis = millis();
    //logga roba
    Serial.println(String(pressure, 4) + " " + String(temp, 4));
    //Serial.println(String(rawPressure) + " " + String(rawTemp));
    Serial.println(baroCount);
    baroCount = 0;
  }
}


//imposta i registri in base ai valori desiderati
void BaroSetup(){
  
  SPI.beginTransaction(SPISettings(BMP388_SPI_SPEED, MSBFIRST, SPI_MODE3));
  digitalWrite(BMP388_CS, LOW);
  
  byte settingsBuff[10] = {BMP388_REG_INT_CTRL, 0b01000010, BMP388_REG_PWR_CTRL, 0b00110011,
                            BMP388_REG_OSR, 0b00000011, BMP388_REG_ODR, 0x02, BMP388_REG_CONFIG, 0x02,};
  SPI.transfer(settingsBuff, 10);
  digitalWrite(BMP388_CS, HIGH);
  SPI.endTransaction();

  BaroGetCalibParam();
  
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

void IntB(){
  readyBaro = 1;
  //baroRead = 1;
  baroCount++;
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
