#include <Wire.h>
#include <VL53L1X.h>
#include <SoftwareSerial.h>
#include "DFRobotDFPlayerMini.h"  //https://texttospeech.ru/
#include "MPU6050.h"
#include "NewPing.h"
//#include <GyverPower.h>

//////////////////////////////DEFINE\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

#define motor_pin 5    // пин с ШИМ для управления вибромотором
#define motor_max 240  // максимальное значение ШИМ для мотора
#define motor_min 5    // минимальное значение ШИМ для мотора 
#define motor_coef 50 //значение коэффициента от 0 до 100
//#define button_pin 8
#define button_pin 3
#define int_gyro_pin 17  // пин для int порта гироскопа. (может отсутствовать так как на практике не нужен(если не подключен к порту задать 99). На референсных схемах висит в воздухе)

#define ToF_pin_1 6   // пин shut ToF датчика 1
#define ToF_pin_2 15  // пин shut ToF датчика 2
//#define ToF_pin_3 7 // пин shut ToF датчика 3
#define ToF_pin_3 14  // пин shut ToF датчика 3

//#define usound_trigger_pin 10 // Триггер пин ультразвукового датчика может быть тем же что Эхо пин если контакты подключенны к 1 проводу
#define usound_trigger_pin 9  // Триггер пин ультразвукового датчика может быть тем же что Эхо пин если контакты подключенны к 1 проводу
#define usound_echo_pin 10    //  Эхо пин ультразвукового датчика может быть тем же что Триггер пин если контакты подключенны к 1 проводу

//#define serial_rx_pin 9 // пин RX для програмного сериал порта. Подключается в TX порт DFPlayer
#define serial_rx_pin 4  // пин RX для програмного сериал порта. Подключается в TX порт DFPlayer
//#define serial_tx_pin 5 // пин ЕX для програмного сериал порта. Подключается в RX порт DFPlayer
#define serial_tx_pin 2  // пин ЕX для програмного сериал порта. Подключается в RX порт DFPlayer

#define adc_pin A0  // пин АЦП ADC6 (pin 19)
#define ppwr_en 7 // ключ питания датчиков
#define nSTDBY_pin 16 // порт режима зарядки
#define nCHRG_pin 8  // порт заряда аккумулятора
////////////////////////////////////////////
#define NUM_READ 3  // можно не менять количество отсчетов

///////////////////////////// ПЕРЕМЕННЫЕ
const uint8_t sensorCount = 3;  //количество сенсоров
const uint8_t xshutPins[sensorCount] = { ToF_pin_1, ToF_pin_2, ToF_pin_3 };

VL53L1X sensors[sensorCount];
NewPing sonar(usound_trigger_pin, usound_echo_pin, 400);  // NewPing setup of pin and maximum distance.

//int maxDelta, R1, R2, R3, R4, last_Obj = 0, new_Obj = 0, proof = 0, last_note, wait = 2000, dt_range = 1500;
/////////////////////////////////

MPU6050 accgyro;
int16_t ax, ay, az, gx, gy, gz;
float accz, gyrox, anglez, anglez1, angley, angley1, anglex, anglex1;  //переменные гироскопа
float filtr_coef = 0.1, dist_coef = 0;

/////////////////////////////////

boolean lastReading = false;   // флаг предыдущего состояния кнопки
boolean buttonSingle = false;  // флаг состояния "краткое нажатие"
boolean buttonMulti = false;   // флаг состояния "двойное нажатие"
bool flag = false, q = true, sw = false;

int bounceTime = 10;    // задержка для подавления дребезга
int doubleTime = 1000;  // время, в течение которого нажатия можно считать двойным
int o = 0, mode = 1;

long onTime = 0;          // переменная обработки временного интервала
long lastSwitchTime = 0;  // переменная времени предыдущего переключения состояния

unsigned long timer[5] = { 1, 1, 1, 1, 1 };  //таймеры
unsigned long timer_PANIC = 1;               //таймер для истиричной вибр раз в 5 сек
unsigned long timer_angl_error = 1;
int valid_cor[4] = { 4000, 4000, 4000, 4000 };
int true_range[3] = { 0, 0, 0 };
int memR[4] = { 4000, 4000, 4000, 4000 };
int verif[5] = { 0, 0, 0, 0 ,0};
int verif_count = 0;
int count;
int delta, last_timer = 0, range, timer_PANIC2;
int maxDelta, last_Obj = 0, new_Obj = 0, proof = 0, last_note, wait = 2000;
/////////////////////////////////

SoftwareSerial mySoftwareSerial(serial_rx_pin, serial_tx_pin);  // RX, TX для плеера DFPlayer Mini
DFRobotDFPlayerMini myDFPlayer;
boolean isPlaying = false;

////////////////////////////////

struct vibr {  //Структура для расстояния и углов для одного среза
  int R[4] = { 4000, 4000, 4000, 4000 };
  int ErrR[4] = { 0, 0, 0, 0 };
  short int AngleX = 0, AngleY = 0, AngleZ = 0;  //собираем углы
  short int Ob_t = 0;                            //тип объекта
  float Volt = 0;                                // напряжение с батареи
  // Функция для анализа ситуации и принятия решения об опасности
  // % 1 - стена
  // % 2 - по курсу
  // % 3 - снизу
  // % 4 - сверху
  // % 5 - яма
  // % 6 - очень близко
  // % 7 - проход есть
  // % 8 - прохода нет
  // % 9 - неверный угол
  // % 10 - Готов к работе, нажмите кнопку сзади
  // % 11 - Режим прохода
  // % 12 - Базовый режим
  // % 13 - Низкий уровень заряда
  // % 14 - Батарея заряжена
  // % 15 - Подключено зарядное устройство
  // % 0 - чисто
  int minR = 0;    //мин расстояние
  int minR12 = 0;  // мин расстояние с двух центральных датчиков т к иначе будет цеплять землю
  unsigned long t_start_imp = 0;
  int i = 0;
};
vibr v;
////////////////////////////////
int R_PIT=0;//Значение для поиска ЯМЫ
int R_HEAD=0;//Значение для поиска препятствия на уровне ГОЛОВЫ
int pass_range = 1500;// порог сквозного прохода
int error[3]={0,0,0};
////////////////////////////////
void setup() {
  //pinMode(14, INPUT);
  //Serial.print("setup");
  //Serial.print("int_interfaces");
  int_interfaces();  // Функция инициализации интерфесов и информационных линий.
  //Serial.print("int_ToF");
  int_ToF();  //Функция инициализации ToF датчиков.
  delay(100);
  //Serial.print("int_DFP()");
  int_DFP();  //Функция инициализации DFPlayer.
  //power.setSleepMode(POWERDOWN_SLEEP);
}

void loop() {
  
  button_state();//Состояние кнопки

  gyro_data(filtr_coef);                                    //Гироскоп
  all_sensor_data_write_sruct_dev();                        //читаем все сенсоры
  new_Obj = object_type(v.R[0], v.R[1], v.R[2], v.R[3], 0); //определяем тип объекта
  Serial.println(new_Obj);
  verification_obj1();                                      //верификация обнаруженного объекта
  if (digitalRead(button_pin)) {                            //если кнопка нажата
    device_control();                                       //управляем устройством
  } else {
    analogWrite(motor_pin, 0);  //гасим вибромотор
  }
  Serial.println(v.Ob_t);

  print_range(v.R[0], v.R[1], v.R[2], v.R[3], v.Ob_t);
  //device_sleep();
}
void int_interfaces() {  // Функция инициализации интерфесов и информационных линий.
  pinMode(adc_pin, INPUT);
  //Serial.println("АЦП ИНИЦИАЛИЗИРОВАН");

  pinMode(ppwr_en, OUTPUT);
  digitalWrite(ppwr_en, HIGH);
  delay(200);
  //Serial.println("Питание на датчики подано");

  pinMode(motor_pin, OUTPUT);  // ШИМ пин управления моторами
  pinMode(button_pin, INPUT);
  pinMode(nSTDBY_pin, INPUT); // режим портов зарядки
  pinMode(nCHRG_pin, INPUT); // режим портов зарядки
  if (int_gyro_pin != 99) {
    pinMode(int_gyro_pin, OUTPUT);
    digitalWrite(int_gyro_pin, HIGH);
  }
  //Serial.println("ШИМ и гиро");

  Serial.begin(9600);
  mySoftwareSerial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);  // use 400 kHz I2C
  delay(100);
  accgyro.initialize();

  Serial.println("Выход из void int_interfaces()");
}
void int_ToF() {  //Функция инициализации ToF датчиков.
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
      while (1)
        ;
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
void int_DFP() {  //Функция инициализации DFPlayer.
   Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
  }
  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.volume(28);  //Set volume value. From 0 to 30
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  myDFPlayer.play(10);  //0010 ассистент готов к работе, для начала использования нажмите кнопку сзади
  //isPlaying = true;
  delay(3500);
  for (int i = 0; i < 3; i++) {
    analogWrite(motor_pin, motor_max);
    delay(100);
    analogWrite(motor_pin, motor_min);
    delay(100);
  }
  /*for(int i=0;i<10;i++){
    myDFPlayer.play(i);
    delay(2000);
    }/**/
}
void gyro_data(float coef) {  // обновление данных с гироскопа.
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
  v.AngleZ = anglez1;  //собираем углы
}
void play_note(int note) {  //Функция проигрывания сообщения.

  if (digitalRead(button_pin)) {
    if ((millis() - timer[0] > 5000) && last_note == note) {
      myDFPlayer.play(note);
      //isPlaying = true;
      Serial.println("ЗВУК");
      timer[0] = millis();
    } else if (last_note != note) {
      myDFPlayer.play(note);
      last_note = note;
      timer[0] = millis();
    }

    if (myDFPlayer.available()) {
      printDetail(myDFPlayer.readType(), myDFPlayer.read());  //Print the detail message from DFPlayer to handle different errors and states.
    }
  }
}
int object_type(int R1, int R2, int R3, int R4, int Angle_nakl) {  //Функция определения типа объекта.
  // Функция для анализа ситуации и принятия решения об опасности
// % 1 - стена
  // % 2 - по курсу
  // % 3 - снизу
  // % 4 - сверху
  // % 5 - яма
  // % 6 - очень близко
  // % 7 - проход есть
  // % 8 - прохода нет
  // % 9 - неверный угол
  // % 10 - Готов к работе, нажмите кнопку сзади
  // % 11 - Режим прохода
  // % 12 - Базовый режим
  // % 13 - Низкий уровень заряда
  // % 14 - Батарея заряжена
  // % 15 - Подключено зарядное устройство
  // % 0 - чисто
  int type = 0;  //по умолчанию считаем что нет препядсвия
  int mode = mode_switch();

  if (mode == 0) {
    //int type = 0;  //по умолчанию считаем что нет препядсвия
    if ((abs(v.AngleY) > 40) || (abs(v.AngleY) < -40))
      return 9;                                                                                        // % 9 - неверный угол
    if (v.minR < 500)
      return 6;  // % 6 - очень близко
    if ( pit_detection_1() )
      return 5;  // % 5 - яма
    if ((R4 < 1000) || ((R3 < 1700)&&((R1 > 2700)&&(R2 > 2700))))
      return 3;  // % 3 - снизу
    if ((max(max(abs(R1 - R2), abs(R2 - R3)), max(abs(R1 - R2), abs(R1 - R4))) < 700) && (R3 < 1400))  //int maxDelta = max(max(abs(R1 - R2), abs(R2 - R3)), abs(R3 - R4))
      return 1;                                                                                        // % 1 - стена
    if ( top_detection_1() )
      return 4;  // % 4 - сверху
    if (R2 < 1300)
      return 2;  // % 2 - по курсу
  } else {
    if ((R3 > 1500)&& v.ErrR[2]!=7) return 7;//проход есть
    return 8; //прохода нет
  }

  return type;
}
void device_control() {  //Функция логики работы для разных препятствий.
  // Функция для анализа ситуации и принятия решения об опасности
// % 1 - стена
  // % 2 - по курсу
  // % 3 - снизу
  // % 4 - сверху
  // % 5 - яма
  // % 6 - очень близко
  // % 7 - проход есть
  // % 8 - прохода нет
  // % 9 - неверный угол
  // % 10 - Готов к работе, нажмите кнопку сзади
  // % 11 - Режим прохода
  // % 12 - Базовый режим
  // % 13 - Низкий уровень заряда
  // % 14 - Батарея заряжена
  // % 15 - Подключено зарядное устройство
  // % 0 - чисто

  if (v.Ob_t == 1) {
    Serial.println("стена по курсу");  // не говорим аудио
   play_note(v.Ob_t);
  } else if (v.Ob_t == 3) {
    Serial.println("снизу");  // Говорим аудио
    //play_note(v.Ob_t);
    vibration_panic();
    //vibration_mode(8);
  } else if (v.Ob_t == 4) {
    Serial.println("сверху");  // Говорим аудио
    play_note(v.Ob_t);
    vibration_panic();
    // vibration_mode(8);
  } else if (v.Ob_t == 5) {
    Serial.println("яма");  // Говорим аудио
    //play_note(v.Ob_t);
    vibration_panic();
    // vibration_mode(8);
  } else if (v.Ob_t == 6) {
    Serial.println("близко");  // Говорим аудио
    //play_note(11);
    vibration_panic();
    return;
  } else if (v.Ob_t == 7) {
    Serial.println("проход ");  // Говорим аудио
    play_note(v.Ob_t);
    vibration_panic2();
    return;
  } else if (v.Ob_t == 8) {
    Serial.println("прохода нет");  // Говорим аудио
    //play_note(v.Ob_t);
    vibration(0, 150, 300);
    return;
  } else if (v.Ob_t == 9) {
    Serial.println("неверный угол");  // Говорим аудио
    /*if (millis() - timer_angl_error > 10000) {
      play_note(v.Ob_t );
      timer_angl_error = millis();
    }*/
    play_note(v.Ob_t);
    vibration(200, 150, 300);
  } 
    else {
    Serial.println("чисто");
  }
  vibration(map(v.minR12, 0, 4000, 250, 95), map(v.minR12, 0, 4000, 1200, 300), map(v.minR12, 0, 4000, 1200, 3000));
  //vibration(map(v.minR12, 0, 4000, 90, 55), map(v.minR12, 0, 4000, 1200, 300),  1200) ;
}
int pit_detection_1()  // детекция под ногами, возвращает 1 при обнаружение препятствия типа яма или препятствия с низким отражением сигнала от датчика, иначе 0.
{

  int coef_det=0;
  if (v.AngleY < -5) return 0;
  int image_r_pit = v.R[3] * sin(((41 + v.AngleY) * 3.14) / 180);  //расчет высоты до поверхности в зависимости от наклона устройства
  if ((image_r_pit - R_PIT) > 0.2 * R_PIT) coef_det = 0.1;         // изменение коэффиента бегущего среднего(рост влияния нового значения)при скачке дальности более 20% от высоты
  if ((image_r_pit - R_PIT) <= 0.2 * R_PIT) coef_det = 0.9;         // изменение коэффиента бегущего среднего(уменьшения влияния нового значения)при скачке дальности менее 20% от высоты
  R_PIT = R_PIT * coef_det + image_r_pit * (1 - coef_det);         // бегущее среднее для компенсации не критичных коллебаний высоты до земли и увеличения окна обнаружения при единичном нахождения ямы
  //Serial.println(R_PIT);
  if (R_PIT > 1500) return 1;                             //порог обнаружения ямы (также срабатывает на препятствие под ногами с поверхностью с низким отражением.

  return 0;
}
int top_detection_1()  // детекция нависающего препятствия, возвращает 1 при обнаружение препятствия , иначе 0.
{
  
  int coef_det=0;
  if (v.AngleY > 10) return 0;
  int image_r_head = v.R[0] * sin(((33 - v.AngleY) * 3.14) / 180);  //расчет высоты до поверхности в зависимости от наклона устройства
  if (image_r_head < 1500) coef_det = 0.05;         // изменение коэффиента бегущего среднего(рост влияния нового значения)
  if (image_r_head >= 1500) coef_det = 0.95;         // изменение коэффиента бегущего среднего(уменьшения влияния нового значения)
  R_HEAD = R_HEAD * coef_det + image_r_head * (1 - coef_det);         // бегущее среднее для компенсации не критичных коллебаний высоты до земли и увеличения окна обнаружения при единичном нахождения верхнего препятствия
  //Serial.println(R_HEAD);
  if (R_HEAD < 700) return 1;                                //порог обнаружения нависающего (также срабатывает на препятствие под ногами с поверхностью с низким отражением.
  return 0;
}
int pass_detection_1() // детекция прохода, возвращает 1 при обнаружение прохода , иначе 0.
{

  if ((v.R[2] > 1000)&&(v.R[3] > 1000)) return 1;
  if ( ((v.R[2] < 1000)||(v.R[3] < 1000))&& ((v.R[1] > 1000)) ) return 2;
  return 0;
  
}
void all_sensor_data_write_sruct_dev() {  //Функция сбора показаний с датчиков и записи их в struct vibr.
  //Функция снятия данные с сенсоров
  //Опрашивает сначала УЗ потом ТОФ и возвращает в структуру 4 дальности R1,R2,R3,R4
  ///////////// Снятие замеров уз дальномера //////////////
  if (millis() - timer[1] > 60) {  // Проверям настало ли время для измерения
    timer[1] = millis();
    //valid_cor[3] = 10 * sonar.convert_cm(sonar.ping_median(3));
    valid_cor[3] = 10 * sonar.ping_cm();
    if (valid_cor[3] < 1)
      valid_cor[3] = 4000;
  }
  ///////////// Снятие замеров TOF //////////////
  for (int i = 0; i < 3; i++) {  //читаем лазеры
    sensors[i].read();
    last_timer = millis();
    range = sensors[i].ranging_data.range_mm;
    if (range == 0) {
      count++;
      if (count > 20) {
        true_range[i] = 0;
      }
    } else {
      true_range[i] = range;
      count = 0;
    }
    //short int j = i;
    //if (i > 0) j= i +1;
    //v.ErrR[j]=sensors[i].ranging_data.range_status;//Запоминаем значение ошибки
    
    
    if (sensors[i].ranging_data.range_status != 0) {
      valid_cor[i] = true_range[i];  //БЫЛО 4000  //sensors[i].ranging_data.range_mm+1000;//если низкий уровень сигнала возможно это темная поверхность добавляем 1000
    } else
      valid_cor[i] = true_range[i];
    memR[i] = valid_cor[i];

    // Измерение Напряжения
    v.Volt = (float)(analogRead(A0) * 5.0) / 1023.0;
  }

  
  v.R[0] = valid_cor[0];
  v.R[1] = valid_cor[3];
  v.R[2] = valid_cor[2];
  v.R[3] = valid_cor[1];

  v.ErrR[0]=sensors[0].ranging_data.range_status;
  v.ErrR[2]=sensors[2].ranging_data.range_status;
  v.ErrR[3]=sensors[1].ranging_data.range_status;
  
  for (int i = 0; i < 4; i++) {
    v.R[i] = findMedianN_optim(v.R[i], i);
    if (v.R[i] > 4000) v.R[i] = 4000;
  }

  v.minR = min(min(v.R[0], v.R[1]), min(v.R[2], v.R[3]));
  v.minR12 = min(v.R[1], v.R[2]);
}
void vibration(int Amp_10_235, int tau_ms, int T) {  //Функция реализации вибрации.
  Amp_10_235=Amp_10_235*motor_coef/100;
  if ((millis() - v.t_start_imp) <= tau_ms) {
    analogWrite(motor_pin, Amp_10_235);
  } else if ((millis() - v.t_start_imp) < T) {
    analogWrite(motor_pin, 0);
  } else {
    analogWrite(motor_pin, Amp_10_235);
    v.t_start_imp = millis();
  }
}
void vibration_panic() {              //функция для оповещения об измении сит\
  if (millis() - timer_PANIC > 3000)  //дрыгаем раз в 5 сек
  {
    for (int i = 0; i < 1; i++) {
      analogWrite(motor_pin, motor_max);
      delay(100);
      analogWrite(motor_pin, motor_min);
      delay(100);
    }
    timer_PANIC = millis();
  }
}
void vibration_panic2() {  //функция для оповещения об измении сит
                           //if (millis() - timer_PANIC2 > 200) //дрыгаем раз в 5 сек
                           //{
                           //for (int i = 0; i < 1; i++) {
  analogWrite(motor_pin, 230);
  delay(60);
  analogWrite(motor_pin, motor_min);
  delay(40);
  //}
  //  timer_PANIC = millis();
  //}
}
void print_range(int R1, int R2, int R3, int R4, int object_type) {  //диагностический вывод данных в дисплей порт.
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
  Serial.print(' ');
  /*Serial.print(" Err1: ");
  Serial.print(v.ErrR[0]);
  Serial.print(' ');*/
 /* Serial.print("Err3:");
  Serial.print(' ');
  Serial.print(v.ErrR[2]);
  Serial.print(' ');
  Serial.print("Err4:");
  Serial.print(' ');
  Serial.print(v.ErrR[3]);
  Serial.print(' ');*/
  /*Serial.print(' ');
    Serial.print("maxDelta:");
    Serial.print(max( max(abs(R1-R2),abs(R2-R3)),abs(R3-R4) ) );*/
  Serial.print(' ');
  Serial.print("object_type:");
  Serial.print(object_type * 1000);
  Serial.print(' '); 
  Serial.print("angle X:");
  Serial.print(v.AngleX);
  Serial.print(' ');
  Serial.print("angle Y:");
  Serial.print(v.AngleY);
  Serial.print(' ');
  Serial.print("angle Z:");
  Serial.print(v.AngleZ);
  Serial.print("Volt:");
  Serial.print(v.Volt);
  //Serial.print(' ');

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
int findMedianN_optim(int newVal, int sensNum) {  //Функция медианного фильтра.
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
void isButtonMulti(int count) {
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
  } else if (mode == 2) {
    myDFPlayer.volume(20);
  } else if (mode == 3) {
    myDFPlayer.volume(10);
  }
}
int mode_switch() {
  if ((v.AngleZ < (-80)) || (v.AngleZ > 50)) {
    if (sw == 0) {
      sw = 1;
      if (digitalRead(button_pin))  
        play_note(11);
    }
    return 1;
  }
  if (sw == 1) {
    sw = 0;
    if (digitalRead(button_pin))  
      play_note(12);
  }
  return 0;
}
/*void device_sleep() {

  if (digitalRead(button_pin) == 0) {
    timer[4] = millis();
  }
  else if ((millis() - timer[4]) > sleep_time) {
    attachInterrupt(1, isr, FALLING);
    power.sleep(SLEEP_FOREVER);
    detachInterrupt(1);
  }

}*/
void isr() {
  // пустая функция для пробуждения
}
void charge_control() {  //Функция контроля заряда аккумулятора.
  if ((digitalRead(nSTDBY_pin) == 0)&(digitalRead(nCHRG_pin) == 0)) {
    myDFPlayer.play(14); //Батарея заряжена

    }
       if ((digitalRead(nSTDBY_pin) == 1)&(digitalRead(nCHRG_pin) == 1)) {
    myDFPlayer.play(13); //Низкий уровень заряда

    }
    if ((digitalRead(nSTDBY_pin) == 1)&(digitalRead(nCHRG_pin) == 0)) {
    myDFPlayer.play(15); //  Подключено зарядное устройство
}
}
void verification_obj1(){  //НОВАЯ верификация изменения объекта. Функция проверки верности обнаружения объекта.
    verif[verif_count] = new_Obj;
    verif_count++;
    if(verif_count >= 4) verif_count = 0;

    for (int i=0;i<10;i++){
      proof = 0;
      if (new_Obj == 4) {// если угроза сверху и !абидиент лайт ниже определенного уровня! ориентируемся по 1 обнаружению "ветки"
      v.Ob_t = 4;
      break;
      }
      for (int j=0;j<5;j++){
       if (verif[j] == i) proof++;
      }
      if (proof >= 3) { // порог на котором считается что обстановка изменилась.
        v.Ob_t = i;//Определяем тип препядсвия
        break;
      }
    }
}
void verification_obj2(){  //!!!СТАРАЯ верификация изменения объекта. Функция проверки верности обнаружения объекта.
    if ((new_Obj == last_Obj)&&(new_Obj != 4)) {
    proof = proof + 1;
  }
  else if (new_Obj == 4){
    proof = 5;
  }
  else {
    proof = 0;
  }
  last_Obj = new_Obj;
  if (proof >= 3) { // порог на котором считается что обстановка изменилась.
    v.Ob_t = new_Obj;//Определяем тип препядсвия
  }
}
