#include <Arduino.h>
#include <SoftwareSerial.h>     
#include "DHT.h"
#include <SPI.h>
#include "Ucglib.h"
#include <NDIRZ16.h>

#define DHTPIN 4
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

#define HEATER 6
#define CO2PUMP 7
#define BOTAO_CO2 1
#define BOTAO_AQUECEDOR 0
//#define RX_PIN 2                                          // Rx pin which the MHZ19 Tx pin is attached to
//#define TX_PIN 3                                          // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)

// -------------------------------- 
#define CO2_LIMIT 50000
#define TEMPERATURE_LIMIT 37
#define POMP_WAITING_TIME 100 // Cicles, not time, later I have to fix for time

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

void setup()
{  
    Serial.begin(9600);                                     // Device to serial monitor feedback
    mySerialCO2.begin(9600);
    dht.begin();
    dht.begin();
    pinMode(HEATER,OUTPUT);
    digitalWrite(HEATER,HIGH);
    pinMode(CO2PUMP,OUTPUT);
    digitalWrite(CO2PUMP,HIGH);
    ucg.begin(UCG_FONT_MODE_TRANSPARENT);
    ucg.clearScreen();
    ucg.setRotate180();
    ucg.setFont(ucg_font_inr16_mr);     

    ucg.setColor(0, 255, 0); // Set color to black
    ucg.drawBox(0, 0, ucg.getWidth(), ucg.getHeight());
    last_status = 0;
    contador_pomp = POMP_WAITING_TIME;
    Serial.println("Wait 10 seconds for the sensor to starup");
    delay(10000);   
    mySerialCO2.listen();
}

void loop()
{
    
    
//    mySensorCO2.measure();
//    mySensorCO2.getppm();
//    delay(5000);
//    Serial.println(mySensorCO2.ppm);
//    CO2i = mySensorCO2.ppm;
    if (mySensorCO2.getppm()) {
//      delay(5000);
      Serial.println(mySensorCO2.ppm);
      CO2i = mySensorCO2.ppm;
    } else {
      Serial.println("Nao consegui coletar ppm");
    }
    float h = dht.readHumidity();
    float t = dht.readTemperature();      
    int botao_co2_status = analogRead(BOTAO_CO2);
    int botao_aquecedor_status = analogRead(BOTAO_AQUECEDOR);
    if(contador_pomp < 0){
      contador_pomp = 0;
    }
    
    int current_status = 0;
    if(CO2i == 0){
      Serial.print("\t\t Resetar arduino"); 
    }
//    CO2i = CO2i+1;
    Serial.println(CO2i);

    if(CO2i < CO2_LIMIT && CO2i > 0){      
      if(botao_co2_onoff == 1 && contador_pomp == 0){ 
        digitalWrite(CO2PUMP,LOW);  
        delay(100);
        digitalWrite(CO2PUMP,HIGH);
        contador_pomp = POMP_WAITING_TIME;
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
      } else {
        digitalWrite(HEATER,HIGH);
      }
      current_status = current_status + 1;
    } else {
      digitalWrite(HEATER,HIGH);
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
    
    if(botao_co2_status > 0){
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
    }

    if(botao_aquecedor_status > 0){
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

       
    last_t = t;
    last_h = h;
    last_co2i = CO2i;
    last_botao_co2_onoff = botao_co2_onoff;
    last_botao_aquecedor_onoff = botao_aquecedor_onoff;
    last_status = current_status;
    contador_pomp = contador_pomp -1;
    delay(2000);
}
