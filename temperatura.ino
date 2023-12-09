#include <Arduino.h>
#include "MHZ19.h"                                        
#include <SoftwareSerial.h>     
#include "DHT.h"
#include <SPI.h>
#include "Ucglib.h"
#define DHTPIN 4
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

#define HEATER 6
#define CO2PUMP 7
// #define BOTAO_CO2 5
#define BOTAO_CO2 1
// #define BOTAO_AQUECEDOR 12
#define BOTAO_AQUECEDOR 0
#define RX_PIN 2                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 3                                          // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)


// -------------------------------- 
#define CO2_LIMIT 800
#define TEMPERATURE_LIMIT 30
#define POMP_WAITING_TIME 30 // Cicles, not time, later I have to fix for time

MHZ19 myMHZ19;                                             // Constructor for library
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // (Uno example) create device to MH-Z19 serial
Ucglib_ST7735_18x128x160_HWSPI ucg(/*cd=*/ 9, /*cs=*/ 10, /*reset=*/ 8);

int last_status;
int botao_co2_onoff = 0;
int botao_aquecedor_onoff = 0;
int last_botao_co2_onoff;
int last_botao_aquecedor_onoff;
float last_t = 0;
float last_h = 0;
int last_co2 = 0;
int contador_pomp;

int rgb_1 = 0;
int rgb_2 = 255;
int rgb_3 = 0;

void setup()
{  
    Serial.begin(9600);                                     // Device to serial monitor feedback
    dht.begin();
    mySerial.begin(BAUDRATE);                               // (Uno example) device to MH-Z19 serial start   
    myMHZ19.begin(mySerial);                                // *Serial(Stream) refence must be passed to library begin(). 

    myMHZ19.autoCalibration();                              // Turn auto calibration ON (OFF autoCalibration(false))
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
    //pinMode(BOTAO_CO2, INPUT);
    //pinMode(BOTAO_AQUECEDOR, INPUT);
    contador_pomp = POMP_WAITING_TIME;
}

void loop()
{
    float h = dht.readHumidity();
    float t = dht.readTemperature();      
    int CO2; 
    int botao_co2_status = analogRead(BOTAO_CO2);
    Serial.print(botao_co2_status); 
    //int botao_aquecedor_status = digitalRead(BOTAO_AQUECEDOR);
    int botao_aquecedor_status = analogRead(BOTAO_AQUECEDOR);
    Serial.println("Aquecedor botao:");
    Serial.print(botao_aquecedor_status);
    Serial.println("------------");
    if(contador_pomp < 0){
      contador_pomp = 0;
    }
    
    CO2 = myMHZ19.getCO2();                             // Request CO2 (as ppm)

    int8_t Temp;
    Temp = myMHZ19.getTemperature();                     // Request Temperature (as Celsius)
    int current_status = 0;
    

    if(CO2 == 0){
      Serial.print("\t\t Resetar arduino"); 
    }
    

    if(CO2 < CO2_LIMIT && CO2 > 0){      
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
      Serial.println("Temperatura baixa");
      if(botao_aquecedor_onoff == 1){
        Serial.println("Botao temperatura was pressed");
        digitalWrite(HEATER,LOW);
      } else {
        digitalWrite(HEATER,HIGH);
      }
      current_status = current_status + 1;
      //Serial.print("\t\t Turn on the heater");      
    } else {
       Serial.println("Temperatura alta "+botao_aquecedor_onoff);
      digitalWrite(HEATER,HIGH);
    }

    if(last_status != current_status){
      Serial.println("Mudanca de estatus");  
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

    if(CO2 != last_co2 || current_status != last_status){
      ucg.setColor(rgb_1,rgb_2,rgb_3);
      ucg.drawBox(10, 58, 100, 24);
      if(current_status == 2){
        ucg.setColor(0,255,255,255);
      } else {
        ucg.setColor(0,0,0,0);
      }
      ucg.setPrintPos(17,75);
      ucg.print(String(CO2)+"ppm");   
    }    
    
    if(botao_co2_status > 0){
      Serial.println("Botao CO2 status apertado");
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
        Serial.println("Desliga");
      } else {
        botao_co2_onoff = 1;
        ucg.print("CO2");
        Serial.println("Liga CO2");
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
        Serial.println("Desliga");
      } else {
        botao_aquecedor_onoff = 1;
        ucg.print("Heat");
        Serial.println("Liga aquecedor");
      }
    }
    
    if(current_status != last_status){
      Serial.println("Status mudou");
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
    last_co2 = CO2;
    last_botao_co2_onoff = botao_co2_onoff;
    last_botao_aquecedor_onoff = botao_aquecedor_onoff;
    last_status = current_status;
    contador_pomp = contador_pomp -1;
    delay(2000);
}
