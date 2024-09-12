#include "EasyNextionLibrary.h"

EasyNex myNex(Serial);

const int REFRESH_TIME = 100;           // Время для обновления страницы Nextion каждые 100 мс
unsigned long refresh_timer = millis(); // Таймер для обновления страницы Nextion


void setup() {
    // Инициализация Serial для вывода данных
    Serial.begin(9600);
    myNex.begin(9600);

    check_connect();
    
    myNex.writeStr("page 2");
}



void loop() {
    //out_put_on_console();

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    // Получение данных акселерометра
    getAccelerometerData(ax, ay, az);

    // Получение данных гироскопа
    getGyroscopeData(gx, gy, gz);
   

    float angleX, angleY, angleZ, angleX1;

    // Получаем углы
    getMonitorWithDegrees(angleX, angleY, angleZ, angleX1);
    

    if ((millis() - refresh_timer) > REFRESH_TIME) { 
        
        //acc
        // myNex.writeNum("ax.val", ax);
        // myNex.writeNum("ay.val", ay);
        // myNex.writeNum("az.val", az);
        myNex.writeNum("ax.val", angleX);
        myNex.writeNum("ay.val", angleY);
        myNex.writeNum("az.val", angleZ);

        //gyro 
        myNex.writeNum("gx.val", gx);
        myNex.writeNum("gy.val", gy);
        myNex.writeNum("gz.val", gz);



        //gauge Nextion z0
        myNex.writeNum("z0.val", angleX);
        myNex.writeNum("z1.val", angleX1);



        // Обновление таймера
        refresh_timer = millis();  
    }

}