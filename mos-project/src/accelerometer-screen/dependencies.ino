void  initBMP280(){
  if (!bmp.begin(0x77)) {
    Serial.println(F("BMP280 not found!"));
    while (1);
}

if (!aht.begin()) {
    Serial.println("AHT20 not found!");
    while (1);
}

Serial.println("AHT20 initialized successfully!");

Serial.println("BMP280 initialized successfully!");
bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                Adafruit_BMP280::SAMPLING_X2,     /* Temperature. */
                Adafruit_BMP280::SAMPLING_X16,    /* Pressure. */
                Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}
void readAndDisplayBMP280Data() {
    // Получаем температуру в цельсиях
    float temperature = bmp.readTemperature();

    // Получаем давление и переводим в inHg
    float pressure_hPa = bmp.readPressure() / 100.0;
    float pressure_inHg = pressure_hPa * 0.02953;

    // Преобразование значений в строки и отправка их на дисплей
    myNex.writeStr("inHgVal.txt", String(pressure_inHg, 2)); // Давление с двумя знаками после запятой
    myNex.writeStr("tempVal.txt", String(temperature, 1)); // Температура с одним знаком после запятой

    
}

void readAndDisplayAHT20Data() {
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);

    // Преобразование значений в строки и отправка их на дисплей
    myNex.writeStr("tempVal.txt", String(temp.temperature, 1)); // Температура с одним знаком после запятой
    myNex.writeStr("humidityVal.txt", String(humidity.relative_humidity, 1)); // Влажность с одним знаком после запятой

   
}


void limitPotentiometer() {
    newPosition = myEncoder.read(); 

    if (newPosition != lastPosition) { 
        
        if (newPosition < 0) {
            newPosition = 0;
            myEncoder.write(newPosition);  
        } 
       
        else if (newPosition > maxValue) {
            newPosition = maxValue;
            myEncoder.write(newPosition);  
        }



        
        lastPosition = newPosition;
        
    }

    
}


void updateButtonColorsByPotValue(uint16_t potValue)
{
    selectedPage = potValue / (32 / maxButtonCount);  // Defining the currently selected page

    for (uint8_t i = 0; i < maxButtonCount; i++) 
    {
        if (i == selectedPage) 
        {
            myNex.writeNum(buttonNames[i], SELECTED_COLOR);
        } 
        else 
        {
            myNex.writeNum(buttonNames[i], UNSELECTED_COLOR);
        }
    }
}

void changePage(uint8_t page)
{
  // Navigating to a page depending on the index
  if (page == 0)
    myNex.writeStr("page 0");  // Switching to page 0
  else if (page == 1)
    myNex.writeStr("page 1");  // Switching to page 1
}




//this method is designed to test the connection and configure the IMU
void checkConnectIMUAndConfigure(){ 
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050, check the connection!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 initialized successfully!");


  // Configure accelerometer and gyroscope
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void degres() { 
  // Get data from MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate angles from accelerometer data
  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;

  // Calculating roll (rotation around X-axis) and pitch (rotation around Y-axis)
  float roll = atan2(accelY, accelZ) * 180 / M_PI;  // Roll in degrees
  float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / M_PI; // Pitch in degrees

  // Convert roll and pitch to range from 0 to 360
  if (roll < 0) {
    roll += 360;
  }
  
  if (pitch < 0) {
    pitch += 360;
  }

  // Round roll and pitch to the nearest integer
  int rollInt = round(roll);
  int pitchInt = round(pitch);

  // Calculate mirrored values by adding or subtracting 180 degrees
  int pitchMirrored = (pitchInt + 180) % 360; // Adding 180 to pitchInt and keeping it in range 0-360
  int rollMirrored = (rollInt + 180) % 360;   // Adding 180 to rollInt and keeping it in range 0-360

  // Write normal values to Nextion display
  myNex.writeNum("horizontalL.val", pitchInt);   // Main pitch value
  myNex.writeNum("verticalB.val", rollInt);      // Main roll value

  // Write mirrored values to Nextion display 
  myNex.writeNum("horizontalR.val", pitchMirrored);  // Mirrored pitch value (180 degrees offset)
  myNex.writeNum("verticalF.val", rollMirrored);     // Mirrored roll value (180 degrees offset)
}
