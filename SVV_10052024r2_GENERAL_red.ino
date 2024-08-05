#include <Wire.h>
#include <VL53L1X.h>
#include <SoftwareSerial.h>
#include "DFRobotDFPlayerMini.h" //https://texttospeech.ru/
#include "MPU6050.h"
#include "NewPing.h"
#include <GyverPower.h>

/////////////////////////////////

const uint8_t sensorCount = 3;  //количество сенсоров
const uint8_t xshutPins[sensorCount] = { 6, 15, 14 };//ПРИ КАЖДОМ ИЗМЕНЕНИИ проверять раз 5!!!!!!!!!!!!!


VL53L1X sensors[sensorCount];
NewPing sonar(9, 10, 400); // NewPing setup of pin and maximum distance.
int button_pin = 8; // pin кнопки
int sleep_time = 60000; // время ухода в сон в мс. (минуты * 60000)

//////////////////////////////
int hole_detec_1[10];
int maxDelta, R1, R2, R3, R4, last_Obj = 0, new_Obj = 0, proof = 0, last_note, wait = 2000, dt_range = 1500,step_1 = 0, coef_det,R_3;
/////////////////////////////////

MPU6050 accgyro;
int16_t ax, ay, az, gx, gy, gz;
float accz, gyrox, anglez, anglez1, angley, angley1, anglex, anglex1; //переменные гироскопа
float filtr_coef = 0.1, dist_coef = 0,image_r;

/////////////////////////////////

boolean lastReading = false;  // флаг предыдущего состояния кнопки
boolean buttonSingle = false; // флаг состояния "краткое нажатие"
boolean buttonMulti = false; // флаг состояния "двойное нажатие"
bool flag = false, q = true, sw = false;

int bounceTime = 10;          // задержка для подавления дребезга
int doubleTime = 1000;         // время, в течение которого нажатия можно считать двойным
int o = 0, mode = 1;

long onTime = 0;              // переменная обработки временного интервала
long lastSwitchTime = 0;      // переменная времени предыдущего переключения состояния

unsigned long timer[5] = {1, 1, 1, 1, 1}; //таймеры
unsigned long timer_PANIC = 1; //таймер для истиричной вибр раз в 5 сек
unsigned long timer_angl_error = 1;
int valid_cor[4] = { 4000, 4000, 4000, 4000 };
int true_range[3] = { 0, 0, 0};
int memR[4] = { 4000, 4000, 4000, 4000 };
int count;
int delta, last_timer = 0, range,timer_PANIC2;
#define NUM_READ 3  // порядок медианы

/////////////////////////////////

SoftwareSerial mySoftwareSerial(4, 5); // RX, TX для плеера DFPlayer Mini
DFRobotDFPlayerMini myDFPlayer;
boolean isPlaying = false;

////////////////////////////////

struct vibr { //Структура для расстояния и углов для одного среза
  int R[4] = {4000, 4000, 4000, 4000};
  short int AngleX = 0, AngleY = 0, AngleZ = 0; //собираем углы
  short int Ob_t = 0; //тип объекта
  // Функция для анализа ситуации и принятия решения об опасности
  // % 0 - чисто
  // % 1 - стена
  // % 2 - по курсу
  // % 3 - снизу говорим
  // % 4 - сверху говорим
  // % 5 - яма говорим
  // % 6 - близко не говорим
  // % 9 - неверный угол говорим
  int minR = 0; //мин расстояние
  int minR12 = 0; // мин расстояние с двух центральных датчиков т к иначе будет цеплять землю
  unsigned long t_start_imp = 0;
  int i = 0;
};
vibr v;


////////////////////////////////
void setup() {
  //pinMode(14, INPUT);
  int_interfaces();// Функция инициализации интерфесов и информационных линий.
  int_ToF();//Функция инициализации ToF датчиков.
  delay(100);
  int_DFP();//Функция инициализации DFPlayer.
  power.setSleepMode(POWERDOWN_SLEEP);
}

void loop() {
  button_state();
  //analogWrite(5, 255);//выключили вибрацию
  gyro_data(filtr_coef); //Гироскоп
  all_sensor_data_write_sruct_dev();//читаем все сенсоры
  new_Obj = object_type(v.R[0], v.R[1], v.R[2], v.R[3], 0); //определяем тип объекта
  if (new_Obj == last_Obj) { //верификация изменения объекта.
    proof = proof + 1;
  }
  else {
    proof = 0;
  }
  last_Obj = new_Obj;
  if (proof >= 2) { // порог на котором считается что обстановка изменилась.
    v.Ob_t = new_Obj;
  }
  if (digitalRead(button_pin)) { //если кнопка нажата
    device_control(); //управляем устройством
  }
  else {
    analogWrite(3, 0);//гасим вибромотор
  }
  print_range(v.R[0], v.R[1], v.R[2], v.R[3], v.Ob_t) ;
  //device_sleep();
}

void int_interfaces() { // Функция инициализации интерфесов и информационных линий.
  pinMode(3, OUTPUT); // ШИМ пин управления моторами
  pinMode(button_pin, INPUT_PULLUP);
  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);
  Serial.begin(9600);
  mySoftwareSerial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);  // use 400 kHz I2C
  delay(100);
  accgyro.initialize();
}

void int_ToF() { //Функция инициализации ToF датчиков.
  for (uint8_t i = 0; i < sensorCount; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }
  for (uint8_t i = 0; i < sensorCount; i++) {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    pinMode(xshutPins[i], INPUT);
    delay(10);

    if (!sensors[i].init()) {
      Serial.println("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }
    // Адрес каждого датчика должен быть изменен на уникальное значение, отличное от
    // значения по умолчанию 0x29 (кроме последнего, который можно оставить по
    // по умолчанию). Чтобы упростить задачу, будем считать вверх от 0x2A.
    sensors[i].setAddress(0x2A + i);
    sensors[i].setMeasurementTimingBudget(100000);
    sensors[i].setTimeout(5);
    sensors[i].startContinuous(50);
    sensors[i].setDistanceMode(VL53L1X::Long);
  }
}
void int_DFP() { //Функция инициализации DFPlayer.
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
  }
  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.volume(22);  //Set volume value. From 0 to 30
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  myDFPlayer.play(8);  //0008 ассистент готов к работе, для начала использования нажмите кнопку сзади
  //isPlaying = true;
  delay(4000);
  for (int i = 0; i < 3; i++) {
    analogWrite(3, 250);
    delay(100);
    analogWrite(3, 0);
    delay(100);
  }
  /*for(int i=0;i<10;i++){
    myDFPlayer.play(i);
    delay(2000);
    }/**/
}
void gyro_data(float coef) { // обновление данных с гироскопа.
  accgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //Serial.println(az);

  anglez = map(az, -18000, 18000, 90, -90);
  anglez1 = anglez * coef + anglez1 * (1 - coef);

  angley = map(ay, -18000, 18000, 90, -90);
  angley1 = angley * coef + angley1 * (1 - coef);

  anglex = map(ax, -18000, 18000, 90, -90);
  anglex1 = anglex * coef + anglex1 * (1 - coef);

  v.AngleX = anglex1;
  v.AngleY = angley1;
  v.AngleZ = anglez1; //собираем углы


}
void ToF_data() { //обновление данных с ToF датчиков.(Не используется в коде)
  R1 = sensors[1].read();
  R3 = sensors[0].read();
  R4 = sensors[0].read();
}
void ultasound_data() {
  ;//Обновление данных с RCWL1005.
}
void l2c_address() { //Функция проверки адресов по шине I2C.
  int nDevices;
  byte error, address;

  Serial.println("Scanning I2C bus...\n");


  nDevices = 0;

  Serial.print("   00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F");


  for (address = 0; address < 128; address++ )
  {
    if ((address % 0x10) == 0)
    {
      Serial.println();
      if (address < 16)
        Serial.print('0');
      Serial.print(address, 16);
      Serial.print(" ");
    }
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address); error = Wire.endTransmission();


    if (error == 0)
    {
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);

      nDevices++;
    }
    else
    {
      Serial.print("--");
    }

    Serial.print(" ");
    delay(1);
  }
  Serial.println();

  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
  {

    Serial.print("Found ");
    Serial.print(nDevices);
    Serial.println(" device(s) ");
  }

  delay(2500);           // wait 5 seconds for next scan

}
void play_note( int note ) { //Функция проигрывания сообщения.
  /*if(note == 13){
    isPlaying = false;}
    else if(millis() - timer[0]>1600){*/
  if ((millis() - timer[0] > 2000) && last_note == note) {
    myDFPlayer.play(note);
    //isPlaying = true;
    Serial.println("ЗВУК");
    timer[0] = millis();
  }
  else if (last_note != note) {
    myDFPlayer.play(note);
    last_note = note;
    timer[0] = millis();
  }

  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }

  /*isPlaying = true;
    timer[0] = millis();
    }*/
}



int object_type(int R1, int R2, int R3, int R4, int Angle_nakl) { //Функция определения типа объекта.
  // Функция для анализа ситуации и принятия решения об опасности
  // % 0 - чисто
  // % 1 - стена
  // % 2 - по курсу
  // % 3 - снизу
  // % 4 - сверху
  // % 5 - яма
  // % 6 - очень близко
  // % 7 - проход сбоку
  // % 8 - проход по центру
  // % 9 - неверный угол
  // % 10 - чисто прохода нету
  int type = 0; //по умолчанию считаем что нет препядсвия
  int mode = mode_switch();
  if (mode == 0) {
    if(bottom_detection_1())return 5; // % 5 - яма
    //int type = 0;  //по умолчанию считаем что нет препядсвия
    if (v.minR < 500 )
      return 6;   // % 6 - очень близко
    if ( (abs(v.AngleY) > 35 ) || (abs(v.AngleZ) > 35 ) )
      return 9;  // % 9 - неверный угол
    if ((max(max(abs(R1 - R2), abs(R2 - R3)), max(abs(R1 - R2), abs(R1 - R4))) < 500) && (R2 < 1500)) //int maxDelta = max(max(abs(R1 - R2), abs(R2 - R3)), abs(R3 - R4))
      return 1;  // % 1 - стена
    if (((R4 < 1200) || (R3 < 1200)) && ((R1 > R3) || (R1 > R4)))
      return 3;  // % 3 - снизу
    if (R1 < 1000)
      return 4;  // % 4 - сверху
    if (R2 < 1300)
      return  2;  // % 2 - по курсу
  }
  else {
      if (R3>1000)  return 7;
      if (R2>1000)  return 7;
      return 10;
    //dist_coef = 11 - 0.006 * max(R1, R4) ;
    //if ((R1 < dt_range) && (R4 < dt_range) && (R3 > dist_coef * max(R1, R4)))
    /*if ((((R1 < 1000) or ( R4 < 1000)) and ((R2 > 1500) or (R3 > 1500))) or ((R1 > 500) and (R2 > 1500) and (R3 > 1500) and (R4 > 500))) return 8;
    if (((R1 > 1500) or ( R4 > 1500)) and ((R2 < 1000) or ( R3 < 1000)))  return 7;
    int wall_check = 0;
    int pass_check = 0;
    if (R1 < 1000) wall_check++;
    if (R2 < 1000) wall_check++;
    if (R3 < 1000) wall_check++;
    if (R4 < 1000) wall_check++;
    if (wall_check >= 3) return 10;
    return 10;
    /*if (((R3 - R1) > 1000) && ((R3 - R4) > 1000) && ( max(R1, R4) < dt_range ))
      return 0;   // % 0(10) - сквозной проход (дверь окно и т.д.) сигналимзируем как чистое пространство
    if ((R1 < dt_range) && (R3 < dt_range) && (R4 < dt_range))
      return 6;  // % 6 (11) - прохода строго нет. индикация как крайне близкое препятствие
    if ((R3 < dt_range) && ((R3 > dt_range) || (R4 > dt_range)))
      return 12;  // % 12 - проход скраю зоны видимости */

  }

  return type;


}

void all_sensor_data_write_sruct_dev() {//Функция сбора показаний с датчиков и записи их в struct vibr.
  //Функция снятия данные с сенсоров
  //Опрашивает сначала УЗ потом ТОФ и возвращает в структуру 4 дальности R1,R2,R3,R4
  ///////////// Снятие замеров уз дальномера //////////////
  if (millis() - timer[1] > 60) {  // Проверям настало ли время для измерения
    timer[1] = millis();
    //valid_cor[3] = 10 * sonar.convert_cm(sonar.ping_median(3));
    valid_cor[3] = 10 * sonar.ping_cm();
    if (valid_cor[3] < 1 )
      valid_cor[3] = 4000;
  }
  ///////////// Снятие замеров TOF //////////////
  for (int i = 0; i < 3; i++) {  //читаем лазеры
    sensors[i].read();
    last_timer = millis();
    range =  sensors[i].ranging_data.range_mm;
    if (range == 0) {
      count++;
      if (count > 20) {
        true_range[i] = 0;
      }
    }
    else {
      true_range[i] = range;
      count = 0;
    }

    //Serial.print(delta);
    //Serial.print("   ");
    //Serial.println(true_range[i]);
    delta = millis() - last_timer;


    if (sensors[i].ranging_data.range_status != 0) {
      valid_cor[i] = 4000;  //sensors[i].ranging_data.range_mm+1000;//если низкий уровень сигнала возможно это темная поверхность добавляем 1000
    } else
      valid_cor[i] = true_range[i];
    memR[i] = valid_cor[i];
    //valid_cor[i] = findMedianN_optim(valid_cor[i], i);
  }
  v.R[0] = valid_cor[0];
  v.R[1] = valid_cor[3];
  v.R[2] = valid_cor[2];
  v.R[3] = valid_cor[1];
  for (int i = 0; i < 4; i++) {
    v.R[i] = findMedianN_optim(v.R[i], i);
    if (v.R[i] > 4000) v.R[i] = 4000;
  }

  v.minR = min(min(v.R[0], v.R[1]), min(v.R[2], v.R[3]));
  v.minR12 = min(v.R[1], v.R[2]);



}

void vibration(int Amp_10_235, int tau_ms,  int T ) {//Функция реализации вибрации.
  if ( (millis() - v.t_start_imp) <= tau_ms )
  {
    analogWrite(3, Amp_10_235);
  }
  else if ((millis() - v.t_start_imp) < T)
  {
    analogWrite(3, 0);
  }
  else
  {
    analogWrite(3, Amp_10_235);
    v.t_start_imp = millis();
  }
}
void vibration_panic()
{ //функция для оповещения об измении сит
  if (millis() - timer_PANIC > 5000) //дрыгаем раз в 5 сек
  {
    for (int i = 0; i < 3; i++) {
      analogWrite(3, 250);
      delay(100);
      analogWrite(3, 0);
      delay(100);
    }
    timer_PANIC = millis();
  }

}

void vibration_panic2()
{ //функция для оповещения об измении сит
  //if (millis() - timer_PANIC2 > 200) //дрыгаем раз в 5 сек
  //{
    //for (int i = 0; i < 1; i++) {
      analogWrite(3, 254);
      delay(60);
      analogWrite(3, 0);
      delay(40);
    //}
  //  timer_PANIC = millis();
  //}

}
void device_control() {//Функция логики работы для разных препятствий.
  //функция управления устройством
  // % 0 - чисто
  // % 1 - стена
  // % 2 - по курсу
  // % 3 - снизу
  // % 4 - сверху
  // % 5 - яма
  // % 6 - очень близко
  // % 9 - неверный угол

  if (v.Ob_t == 1) {
    Serial.println("стена по курсу");// не говорим аудио
    play_note(v.Ob_t );
  }
  else if (v.Ob_t == 3) {
    Serial.println("снизу");// Говорим аудио
    play_note(v.Ob_t );
    vibration_panic2();
    //vibration_mode(8);
  }
  else if (v.Ob_t == 4) {
    Serial.println("сверху");// Говорим аудио
    play_note(v.Ob_t );
    vibration_panic();
    // vibration_mode(8);
  }
  else if (v.Ob_t == 5) {
    Serial.println("яма");// Говорим аудио
    play_note(v.Ob_t );
    vibration_panic2();
    // vibration_mode(8);
  }
  else if (v.Ob_t == 6) {
    Serial.println("близко");// Говорим аудио
    //play_note(11);
    vibration(250, 150, 300);
    return;
  }
  else if (v.Ob_t == 7) {
    Serial.println("проход сбоку");// Говорим аудио
    //play_note(11);
    vibration_panic2();
    return;
  }
  else if (v.Ob_t == 8) {
    Serial.println("проход");// Говорим аудио
    //play_note(11);
    vibration(0, 150, 300);
    return;
  }
  else if (v.Ob_t == 9) {
    Serial.println("неверный угол");// Говорим аудио
    /*if (millis() - timer_angl_error > 10000) {
      play_note(v.Ob_t );
      timer_angl_error = millis();
    }*/
    vibration_panic();
  }
  else if (v.Ob_t == 10) {
    Serial.println("проход");// Говорим аудио
    //play_note(11);
    vibration(0, 150, 300);
    return;
  }
  else {
    Serial.println("чисто");
  }
  vibration(map(v.minR12, 0, 4000, 250, 95), map(v.minR12, 0, 4000, 1200, 300),  map(v.minR12, 0, 4000, 1200, 3000)) ;
  //vibration(map(v.minR12, 0, 4000, 90, 55), map(v.minR12, 0, 4000, 1200, 300),  1200) ;


}
void print_range(int R1, int R2, int R3, int R4, int object_type ) { //диагностический вывод данных в дисплей порт.
  Serial.print("R1:");
  Serial.print(R1);
  Serial.print(' ');
  Serial.print("R2:");
  Serial.print(R2);
  Serial.print(' ');
  Serial.print("R3:");
  Serial.print(R3);
  Serial.print(' ');
  Serial.print("R4:");
  Serial.print(R4);
  /*Serial.print(' ');
    Serial.print("maxDelta:");
    Serial.print(max( max(abs(R1-R2),abs(R2-R3)),abs(R3-R4) ) );*/
  Serial.print(' ');
  Serial.print("object_type:");
  Serial.print(object_type * 1000);
  Serial.print("  ");
  Serial.print(R_3);
  Serial.print(' ');/**/
  Serial.print("angle X:");
  Serial.print(v.AngleX);
  Serial.print(' ');
  Serial.print("angle Y:");
  Serial.print(v.AngleY);
  Serial.print(' ');
  Serial.print("angle Z:");
  Serial.print(v.AngleZ);
  //Serial.print(' ');
  /*Serial.print("Akk:");
    Serial.print(analogRead(A6)*0.0049);//A6*0.0049= напряжение на аккуме*/
  Serial.println();
}
void printDetail(uint8_t type, int value) {
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}
int findMedianN_optim(int newVal, int sensNum) {//Функция медианного фильтра.
  int out;
  if (sensNum == 0) {
    static int buffer1[NUM_READ];  // статический буфер
    static byte count = 0;
    buffer1[count] = newVal;
    if ((count < NUM_READ - 1) and (buffer1[count] > buffer1[count + 1])) {
      for (int i = count; i < NUM_READ - 1; i++) {
        if (buffer1[i] > buffer1[i + 1]) {
          int buff = buffer1[i];
          buffer1[i] = buffer1[i + 1];
          buffer1[i + 1] = buff;
        }
      }
    } else {
      if ((count > 0) and (buffer1[count - 1] > buffer1[count])) {
        for (int i = count; i > 0; i--) {
          if (buffer1[i] < buffer1[i - 1]) {
            int buff = buffer1[i];
            buffer1[i] = buffer1[i - 1];
            buffer1[i - 1] = buff;
          }
        }
      }
    }
    if (++count >= NUM_READ) count = 0;
    out = buffer1[(int)NUM_READ / 2];
  }

  if (sensNum == 1) {
    static int buffer2[NUM_READ];  // статический буфер
    static byte count = 0;
    buffer2[count] = newVal;
    if ((count < NUM_READ - 1) and (buffer2[count] > buffer2[count + 1])) {
      for (int i = count; i < NUM_READ - 1; i++) {
        if (buffer2[i] > buffer2[i + 1]) {
          int buff = buffer2[i];
          buffer2[i] = buffer2[i + 1];
          buffer2[i + 1] = buff;
        }
      }
    } else {
      if ((count > 0) and (buffer2[count - 1] > buffer2[count])) {
        for (int i = count; i > 0; i--) {
          if (buffer2[i] < buffer2[i - 1]) {
            int buff = buffer2[i];
            buffer2[i] = buffer2[i - 1];
            buffer2[i - 1] = buff;
          }
        }
      }
    }
    if (++count >= NUM_READ) count = 0;
    out = buffer2[(int)NUM_READ / 2];
  }

  if (sensNum == 2) {
    static int buffer3[NUM_READ];  // статический буфер
    static byte count = 0;
    buffer3[count] = newVal;
    if ((count < NUM_READ - 1) and (buffer3[count] > buffer3[count + 1])) {
      for (int i = count; i < NUM_READ - 1; i++) {
        if (buffer3[i] > buffer3[i + 1]) {
          int buff = buffer3[i];
          buffer3[i] = buffer3[i + 1];
          buffer3[i + 1] = buff;
        }
      }
    } else {
      if ((count > 0) and (buffer3[count - 1] > buffer3[count])) {
        for (int i = count; i > 0; i--) {
          if (buffer3[i] < buffer3[i - 1]) {
            int buff = buffer3[i];
            buffer3[i] = buffer3[i - 1];
            buffer3[i - 1] = buff;
          }
        }
      }
    }
    if (++count >= NUM_READ) count = 0;
    out = buffer3[(int)NUM_READ / 2];
  }
  if (sensNum == 3) {
    static int buffer4[NUM_READ];  // статический буфер
    static byte count = 0;
    buffer4[count] = newVal;
    if ((count < NUM_READ - 1) and (buffer4[count] > buffer4[count + 1])) {
      for (int i = count; i < NUM_READ - 1; i++) {
        if (buffer4[i] > buffer4[i + 1]) {
          int buff = buffer4[i];
          buffer4[i] = buffer4[i + 1];
          buffer4[i + 1] = buff;
        }
      }
    } else {
      if ((count > 0) and (buffer4[count - 1] > buffer4[count])) {
        for (int i = count; i > 0; i--) {
          if (buffer4[i] < buffer4[i - 1]) {
            int buff = buffer4[i];
            buffer4[i] = buffer4[i - 1];
            buffer4[i - 1] = buff;
          }
        }
      }
    }
    if (++count >= NUM_READ) count = 0;
    out = buffer4[(int)NUM_READ / 2];
  }


  return out;
}
void button_state() {
  ///////////// Вход в алгоритм нажатия кнопок и выбора режима //////////////
  boolean reading = digitalRead(button_pin);
  // проверка первичного нажатия
  if (reading && !lastReading) {
    onTime = millis();
  }

  if (!reading && lastReading) {
    if (((millis() - onTime) > bounceTime)) {
      if ((millis() - lastSwitchTime) >= doubleTime) {
        lastSwitchTime = millis();
        buttonSingle = true;
        o = 1;
      } else {
        o++;
        lastSwitchTime = millis();
        buttonSingle = false;
        buttonMulti = true;
      }
    }
  }

  lastReading = reading;

  if (buttonSingle && (millis() - lastSwitchTime) > doubleTime) {
    isButtonSingle();
  }
  if (buttonMulti && (millis() - lastSwitchTime) > doubleTime) {
    isButtonMulti(o);
  }
}
void isButtonSingle() {
  buttonMulti = false;
  buttonSingle = false;
  //Serial.println(1);
}
void isButtonMulti( int count ) {
  buttonSingle = false;
  buttonMulti = false;
  //Serial.println(count);
  if (count == 2) {
    mode++;
  }
  if (mode == 4) {
    mode = 1;
  }
  if (mode == 1) {
    myDFPlayer.volume(30);
  }
  else if (mode == 2) {
    myDFPlayer.volume(20);
  }
  else if (mode == 3) {
    myDFPlayer.volume(10);
  }
}
int mode_switch() {
  if ((v.AngleZ < (-80)) || (v.AngleZ > 50)) {
    if (sw == 0) {
      sw = 1;
      if (digitalRead(button_pin))//Типичный пример говно кода когда все не из одного места делается, а через одно место!!!
        play_note(11);
    }
    return 1;
  }
  if (sw == 1) {
    sw = 0;
    if (digitalRead(button_pin))//Типичный пример говно кода когда все не из одного места делается, а через одно место!!!
    play_note(10);
  }
  return 0;
}
void device_sleep() {

  if (digitalRead(button_pin) == 0) {
    timer[4] = millis();
  }
  else if ((millis() - timer[4]) > sleep_time) {
    attachInterrupt(1, isr, FALLING);
    power.sleep(SLEEP_FOREVER);
    detachInterrupt(1);
  }

}
void isr() {
  // пустая функция для пробуждения
}
int bottom_detection_1 () // детекция под ногами
{ 
  image_r = v.R[3] * cos(((41-v.AngleY)*3.14)/180); 
  if ((image_r - R_3) > 0.2*R_3) int coef_det = 0.1;
  if ((image_r - R_3) < 0.2*R_3) int coef_det = 0.9;
  R_3 =  R_3 * coef_det + image_r * (1 - coef_det);
  /*Serial.println();
  Serial.print(image_r);
  Serial.print("  ");
  Serial.print(R_3);
  Serial.print("  ");*/
  if (R_3 > 1300)return 1;
  return 0;
}
