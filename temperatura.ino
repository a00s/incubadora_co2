#include <Arduino.h>
#include <SoftwareSerial.h>     
#include <SPI.h>
#include "Ucglib.h"
#include <NDIRZ16.h>
#include <avr/wdt.h>
#include <EEPROM.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

//#define ERROR_LOG_ADDRESS 0 // EEPROM address to store error log
#define EEPROM_CO2 0
#define EEPROM_AQUECIMENTO 12
#define EEPROM_CONTADOR 24

#define DHTPIN 4
#define DHT_TEMPERATURA_INT_PIN 2
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
DHT dhtint(DHT_TEMPERATURA_INT_PIN, DHTTYPE);

#define HEATER 6
#define CO2PUMP 7
#define BOTAO_CO2 1
#define BOTAO_AQUECEDOR 0
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)

#define CO2_LIMIT 50000
#define TEMPERATURE_LIMIT 37
#define POMP_WAITING_TIME 10 // Cicles, not time, later I have to fix for time
#define MAX_INTERNAL_TEMP 30 // Temperatura maxima antes de ligar o ventilador

#define VENTILADOR_PIN 3

SoftwareSerial mySerialCO2(12,5);
NDIRZ16 mySensorCO2 = NDIRZ16(&mySerialCO2);
Ucglib_ST7735_18x128x160_HWSPI ucg(/*cd=*/ 9, /*cs=*/ 10, /*reset=*/ 8);

int last_status;
int botao_co2_onoff = 0;
int botao_aquecedor_onoff = 0;
int last_botao_co2_onoff;
int last_botao_aquecedor_onoff;
float last_t = 0;
float last_h = 0;
int32_t last_co2 = 0;
int32_t last_co2i = 0;
int contador_pomp;
int32_t CO2i = 0;
int rgb_1 = 0;
int rgb_2 = 255;
int rgb_3 = 0;
int32_t eeprom_contador_int = 0;
int ventilador_status = 0;

unsigned long lastHeartbeatTime = 0; // To track the last heartbeat time
const unsigned long heartbeatInterval = 5000; // Heartbeat interval in milliseconds
bool screenResponsive = true; // Flag to check screen responsiveness



void setup()
{  
    Serial.begin(9600);                                     // Device to serial monitor feedback

    // -------------------- DEBUG --------------------------

    EEPROM.get(EEPROM_AQUECIMENTO, botao_aquecedor_onoff);
    EEPROM.get(EEPROM_CO2, botao_co2_onoff);
    Serial.print("PreCO2: ");
    Serial.print(botao_co2_onoff);
    Serial.print(" PreTemp: ");
    Serial.println(botao_aquecedor_onoff);

    EEPROM.get(EEPROM_CONTADOR, eeprom_contador_int);
    eeprom_contador_int++;
    EEPROM.put(EEPROM_CONTADOR, eeprom_contador_int);
   
    
    // Check if there's an error log
//    char error[50];
//    EEPROM.get(ERROR_LOG_ADDRESS, error);
//    error[49] = '\0';  // Ensure null-termination
//    if (strlen(error) > 0) {
//      Serial.print("Previous error: ");
//      Serial.println(error);
//      // Clear the error log
//      EEPROM.put(ERROR_LOG_ADDRESS, "");
//    }
   
    
    // -------------------------------------------
    mySerialCO2.begin(9600);
    dht.begin();
    dhtint.begin();
    pinMode(HEATER,OUTPUT);
    digitalWrite(HEATER,HIGH);
    pinMode(CO2PUMP,OUTPUT);
    digitalWrite(CO2PUMP,HIGH);

    pinMode(VENTILADOR_PIN,OUTPUT);
    digitalWrite(VENTILADOR_PIN,HIGH);

//    // Set SPI settings to full speed
    SPI.begin();
    SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0)); // 40 MHz is typically the highest SPI speed for Arduino

    
    ucg.begin(UCG_FONT_MODE_TRANSPARENT);
    ucg.clearScreen();
    ucg.setRotate180();
    ucg.setFont(ucg_font_inr16_mr);     

    ucg.setColor(0, 255, 0); // Set color to black
    ucg.drawBox(0, 0, ucg.getWidth(), ucg.getHeight());
    last_status = 0;
    contador_pomp = POMP_WAITING_TIME;
    Serial.println("Wait 10 seconds for the sensor to start up");
//    if (MCUSR & (1 << WDRF)) {
//      Serial.println("Watchdog reset detected.");
//      EEPROM.put(ERROR_LOG_ADDRESS, "Watchdog reset detected."); // Optionally log to EEPROM
//    }
    delay(10000);   
    mySerialCO2.listen();
    // Clear the reset flags
    MCUSR = 0;
    // Enable the Watchdog Timer
    wdt_enable(WDTO_8S);
    
}

void loop()
{
    wdt_reset(); // Reset the watchdog timer



//    if (Serial.available() > 0) {
//        String command = Serial.readStringUntil('\n');
//        command.trim(); // Remove any trailing newline characters
//
//        if (command.equals("reinitialize")) {
//            reinitializeScreen();
//        } else {
//            Serial.println("Unknown command");
//        }
//    }
//    
    
    // Heartbeat mechanism
//    if (millis() - lastHeartbeatTime >= heartbeatInterval) {
//        updateHeartbeat();
//    }
//    
//    // Check if the heartbeat character is still there
//    if (!screenResponsive) {
//        Serial.println("Screen is unresponsive!");
//    }


    if (mySensorCO2.getppm()) {
      CO2i = mySensorCO2.ppm;
    } else {
      Serial.println("Nao consegui coletar ppm");
    }

    float h = dht.readHumidity();
    float t = dht.readTemperature();
    float t_interna = dhtint.readTemperature();      
    int botao_co2_status = analogRead(BOTAO_CO2);
    int botao_aquecedor_status = analogRead(BOTAO_AQUECEDOR);
    Serial.println(t_interna);
    if(t_interna >= MAX_INTERNAL_TEMP && ventilador_status == 0){
      Serial.println("Ligando ventilador");
       digitalWrite(VENTILADOR_PIN,LOW);
       ventilador_status = 1;
    } else if (t_interna < (MAX_INTERNAL_TEMP - 1.0) && ventilador_status == 1){
      Serial.println("Desligando ventilador");     
       digitalWrite(VENTILADOR_PIN,HIGH); 
       ventilador_status = 0;
    }

    if(contador_pomp < 0){
      contador_pomp = 0;
    }
    
    int current_status = 0;
    if(CO2i == 0){
      Serial.println("\t\t Resetar arduino"); 
    }
    
//    Serial.print("CONT: ");
//    Serial.print(eeprom_contador_int);
////    Serial.print("\tMEM: ");
////    Serial.print(freeMemory());
//    Serial.print("\tVolt: ");
//    long voltage = readVCC();
//    Serial.print(voltage / 1000.0);
//    Serial.print("\tBotao CO2: ");
//    Serial.print(botao_co2_status);    
//    Serial.print("\tPompa CO2: ");
//    Serial.print(botao_co2_onoff);
//    Serial.print("\tBotao Aquecer: ");    
//    Serial.print(botao_aquecedor_status);    
//    Serial.print("\tAquecer: ");
//    Serial.print(botao_aquecedor_onoff);    
//    Serial.print("\tCO2: ");
//    Serial.println(CO2i);  

    if(CO2i < CO2_LIMIT && CO2i > 0){      
      if(botao_co2_onoff == 1 && contador_pomp == 0){ 
        if(CO2i < last_co2i){ // so adiciona mais CO2 se ele tiver caindo
          Serial.print(CO2i);
          Serial.println(last_co2i);  
          digitalWrite(CO2PUMP,LOW);  
          delay(10);
          digitalWrite(CO2PUMP,HIGH);
          contador_pomp = POMP_WAITING_TIME;
          reinitializeScreen();
        } else {
          contador_pomp = 10; // gera mais 10 loops pra poder ver se comeca a cair realmente
          Serial.println("---------------- CO2 estava ainda subindo ou estavel -----------------"); 
        }
      } else {
        digitalWrite(CO2PUMP,HIGH);
      }
      current_status = current_status + 1;    
    } else {
      digitalWrite(CO2PUMP,HIGH);
    }

    if(t < TEMPERATURE_LIMIT){ 
      if(botao_aquecedor_onoff == 1){
        digitalWrite(HEATER,LOW);

        ucg.setColor(rgb_1,rgb_2,rgb_3);
        ucg.drawBox(67, 125, 90, 34);
        ucg.setPrintPos(74,145);
        if(current_status == 2){
          ucg.setColor(0,255,255,255);
        } else {
          ucg.setColor(0,0,0,0);
        }
        ucg.print("/\\");
      } else {
        digitalWrite(HEATER,HIGH);
        ucg.setColor(rgb_1,rgb_2,rgb_3);
        ucg.drawBox(67, 125, 90, 34);    
      }
      current_status = current_status + 1;
    } else {
      digitalWrite(HEATER,HIGH);
      ucg.setColor(rgb_1,rgb_2,rgb_3);
      ucg.drawBox(67, 125, 90, 34);  
    }
//    Serial.println(current_status);
    if(botao_aquecedor_status > 500 && botao_co2_status > 500){
      Serial.println("Dois botoes apertados");
      reinitializeScreen();
      eeprom_contador_int = 0;
      EEPROM.put(EEPROM_CONTADOR, eeprom_contador_int);
    }

    if(last_status != current_status){
      if(current_status == 0){
        ucg.setColor(0, 255, 0); // Set color to GREEN
        ucg.drawBox(0, 0, ucg.getWidth(), ucg.getHeight());
        rgb_1 = 0;
        rgb_2 = 255;
        rgb_3 = 0;
      } else if(current_status == 1){
        ucg.setColor(255, 255, 0); // Set color to YELLOW
        ucg.drawBox(0, 0, ucg.getWidth(), ucg.getHeight());
        rgb_1 = 255;
        rgb_2 = 255;
        rgb_3 = 0;
      } else if(current_status > 1){
        ucg.setColor(255, 0, 0); // Set color to RED
        ucg.drawBox(0, 0, ucg.getWidth(), ucg.getHeight());
        rgb_1 = 255;
        rgb_2 = 0;
        rgb_3 = 0;
      }      
    }
    
    if(t != last_t || current_status != last_status){
      ucg.setColor(rgb_1,rgb_2,rgb_3);
      ucg.drawBox(10, 7, 100, 20);
      if(current_status == 2){
        ucg.setColor(0,255,255,255);
      } else {
        ucg.setColor(0,0,0,0);
      }
      ucg.setPrintPos(17,25);
      ucg.print(String(t)+" C");      
    }

    if(h != last_h || current_status != last_status){
      ucg.setColor(rgb_1,rgb_2,rgb_3);
      ucg.drawBox(10, 32, 100, 20);
      if(current_status == 2){
        ucg.setColor(0,255,255,255);
      } else {
        ucg.setColor(0,0,0,0);
      }
      ucg.setPrintPos(17,50);
      ucg.print(String(h)+" %");     
    }

    if(CO2i != last_co2i || current_status != last_status){
      ucg.setColor(rgb_1,rgb_2,rgb_3);
      ucg.drawBox(10, 58, 100, 24);
      if(current_status == 2){
        ucg.setColor(0,255,255,255);
      } else {
        ucg.setColor(0,0,0,0);
      }
      ucg.setPrintPos(17,75);
      ucg.print(String(CO2i)+"ppm");
    } 
    
    if(botao_co2_status > 500 && botao_aquecedor_status < 500){
      ucg.setColor(rgb_1,rgb_2,rgb_3);
      ucg.drawBox(10, 90, 80, 24);
      ucg.setPrintPos(17,110);
      if(current_status == 2){
        ucg.setColor(0,255,255,255);
      } else {
        ucg.setColor(0,0,0,0);
      }
      if(botao_co2_onoff == 1){
        botao_co2_onoff = 0;        
      } else {
        botao_co2_onoff = 1;
        ucg.print("CO2");
      }
      Serial.println("Alterando status do CO2"); 
      EEPROM.put(EEPROM_CO2, botao_co2_onoff);
    }

    if(botao_aquecedor_status > 500 && botao_co2_status < 500){
//    if(botao_aquecedor_status > 500){
//      ucg.clearScreen();
      ucg.setColor(rgb_1,rgb_2,rgb_3);
      ucg.drawBox(10, 125, 80, 24);
      ucg.setPrintPos(17,145);
      if(current_status == 2){
        ucg.setColor(0,255,255,255);
      } else {
        ucg.setColor(0,0,0,0);
      }
      if(botao_aquecedor_onoff == 1){
        botao_aquecedor_onoff = 0;
      } else {
        botao_aquecedor_onoff = 1;
        ucg.print("Heat");
      }
      Serial.println("Alterando status do aquecimento");
      EEPROM.put(EEPROM_AQUECIMENTO, botao_aquecedor_onoff);
    }
    
    if(current_status != last_status){
      ucg.setColor(rgb_1,rgb_2,rgb_3);
      ucg.drawBox(10, 90, 80, 24);
      ucg.drawBox(10, 125, 80, 24);
      ucg.setColor(rgb_1,rgb_2,rgb_3);      
      if(current_status == 2){
        ucg.setColor(0,255,255,255);
      } else {
        ucg.setColor(0,0,0,0);
      }
      if(botao_co2_onoff == 1){
        ucg.setPrintPos(17,110);
        ucg.print("CO2");
      }
      if(botao_aquecedor_onoff == 1) {
        ucg.setPrintPos(17,145);
        ucg.print("Heat");
      }
    }

      // ucg.drawBox(10, 90, 80, 24);
      // ucg.setPrintPos(17,110);

    ucg.setColor(rgb_1,rgb_2,rgb_3);
    ucg.drawBox(67, 90, 80, 34);
    ucg.setPrintPos(74,110);
    if(current_status == 2){
      ucg.setColor(0,255,255,255);
    } else {
      ucg.setColor(0,0,0,0);
    }
    ucg.print("R:" + String(eeprom_contador_int));

    // USAR pro heating
    // ucg.setColor(rgb_1,rgb_2,rgb_3);
    // ucg.drawBox(67, 125, 90, 34);
    // ucg.setPrintPos(74,145);
    // if(current_status == 2){
    //   ucg.setColor(0,255,255,255);
    // } else {
    //   ucg.setColor(0,0,0,0);
    // }
    // ucg.print(eeprom_contador_int);


 

    last_t = t;
    last_h = h;
    last_co2i = CO2i;
    last_botao_co2_onoff = botao_co2_onoff;
    last_botao_aquecedor_onoff = botao_aquecedor_onoff;
    last_status = current_status;
    contador_pomp = contador_pomp -1;
    delay(2000);
}



void reinitializeScreen() {
    // ucg.begin(UCG_FONT_MODE_TRANSPARENT);
    // delay(1000);
    // ucg.setRotate180();
    // delay(1000);
    // ucg.setRotate180();

    ucg.begin(UCG_FONT_MODE_TRANSPARENT);
    ucg.setRotate180();
    ucg.setFont(ucg_font_inr16_mr);
    ucg.setRotate180();
}

//void wakeUpScreen() {
//    ucg.sendCommand(0x11); // Sleep Out command
//    delay(120);            // Wait for the screen to wake up
//    ucg.sendCommand(0x29); // Display ON
//    Serial.println("Screen awakened");
//}

// funciona
//void reinitializeScreen() {
//    ucg.begin(UCG_FONT_MODE_TRANSPARENT);
//    ucg.setRotate90();
//    ucg.setFont(ucg_font_inr16_mr);
//    Serial.println("Screen reinitialized");
//}

// funciona 2
// ucg.begin(UCG_FONT_MODE_TRANSPARENT);
// ucg.setRotate180();
// ucg.setFont(ucg_font_inr16_mr);
// ucg.setRotate180();

//funciona 3
//ucg.begin(UCG_FONT_MODE_TRANSPARENT);
//ucg.setRotate180();
//ucg.setRotate180();