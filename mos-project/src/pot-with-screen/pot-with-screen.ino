#include "EasyNextionLibrary.h" // Подключение библиотеки EasyNextionLibrary

#define ENC_A 2       // Пин S1 энкодера
#define ENC_B 4       // Пин S2 энкодера


int encCounter = 0;   // Счётчик значений энкодера
boolean state0, lastState;

EasyNex myNex(Serial);   // Создание объекта для работы с экраном Nextion

const int REFRESH_TIME = 100;           // Время для обновления страницы Nextion каждые 100 мс
unsigned long refresh_timer = millis(); // Таймер для обновления страницы Nextion

void setup() {
  Serial.begin(9600);   // Инициализация Serial для монитора порта
  myNex.begin(9600);    // Инициализация экрана Nextion с той же скоростью

  // Настройка пинов энкодера
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  lastState = digitalRead(ENC_A);  // Чтение начального состояния пина ENC_A

  // Переключаемся на нужную страницу экрана Nextion
  myNex.writeStr("page 1");
  delay(2000);  // Небольшая задержка для переключения страницы
}

void loop() {
  // Чтение текущего состояния пина ENC_A
  state0 = digitalRead(ENC_A);

  if (state0 != lastState) {  // Если состояние изменилось
    if (digitalRead(ENC_B) != state0) {
      encCounter--;  // Поворот против часовой стрелки
    } else {
      encCounter++;  // Поворот по часовой стрелке
    }
    lastState = state0;  // Обновляем последнее состояние
  }

  if ((millis() - refresh_timer) > REFRESH_TIME) { 
    // Отправляем значение энкодера на экран Nextion
    myNex.writeNum("n1.val", encCounter);

    // Отладочное сообщение в Serial Monitor
    Serial.print("Значение энкодера: ");
    Serial.println(encCounter);

    // Обновление таймера
    refresh_timer = millis();  
  }
}
