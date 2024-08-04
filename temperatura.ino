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
#define POMP_WAITING_TIME 100 // Cicles, not time, later I have to fix for time
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

// Variáveis para calcular a taxa de aumento de CO2
unsigned long lastTime = 0; // Armazena o tempo da última leitura
float co2Rate = 0; // Armazena a taxa de aumento do CO2
float maxRate = 0;
int32_t co2_before_pump = 0;
int32_t max_co2_increase = 0;
// Limites de taxa e delay para liberacao da valvula de CO2 proporcional a pressao
// analise: 1436ppm/s pra 0.5kg/cm2 com a valvula aperta por 5ms
const float TAXA_MAXIMA = 1000.0; // ppm/s
const float TAXA_MINIMA = 10.0;   // ppm/s
const int DELAY_MAXIMO = 30;      // ms
const int DELAY_MINIMO = 1;       // ms
int32_t delayProporcionalAnt = 1;     // Define o delay inicial de abertura da valvula
// const float fatorProximidade = 0.01; // Ajuste este fator para calibrar o impacto da proximidade
bool first_pomp = true; // need to be set so the first pomp uses the minimal delay

// Variáveis para calcular a média de aumento ou queda de CO2 por minuto
int32_t co2AccumulatedChange = 0; // Acumula as mudanças de CO2
unsigned long lastMinuteTime = 0; // Armazena o tempo do último cálculo de média
const unsigned long oneMinuteInterval = 60000; // Intervalo de um minuto em milissegundos

// Parâmetros de controle
// const float Kp = 0.1; // Ganho proporcional inicial
const int32_t min_ms = 1; // Tempo mínimo de abertura
const int32_t max_ms = 20; // Limite máximo de tempo de abertura em milissegundos

// Variáveis globais para aprendizado adaptativo
// float media_eficiencia = 0.0;
// int contagem_aberturas = 0;

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
    // Disable auto-calibration
  disableAutoCalibration();

  // Verify if auto-calibration is disabled
  verifyAutoCalibration();
  //   mySerialCO2.listen();
  //  // Disable auto-calibration
    // byte autoCalibOff[] = {0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86};
    // mySerialCO2.write(autoCalibOff, 9);

  //   // Wait for a response (optional, for debugging)
    // delay(500);
  //   while (mySerialCO2.available()) {
  //       byte response = mySerialCO2.read();
  //       Serial.print(response, HEX);
  //       Serial.print(" ");
  //   }
  //   Serial.println();
  //   // Check the auto-calibration status from the response
  //   if (response[1] == 0x79 && response[2] == 0x00) {
  //     Serial.println("Auto-calibration is disabled.");
  //   } else if (response[1] == 0x79 && response[2] == 0x01) {
  //     Serial.println("Auto-calibration is enabled.");
  //   } else {
  //     Serial.println("Unknown response.");
  //   }
    // Send the command to check auto-calibration status
    // byte checkAutoCalib[] = {0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86}; // Example command, may vary based on sensor documentation
    // mySerialCO2.write(checkAutoCalib, 9);

    // // Wait for a response
    // delay(500);
    // byte response[9];
    // int index = 0;
    // while (mySerialCO2.available() && index < 9) {
    //   response[index++] = mySerialCO2.read();
    // }

    // // Print the response
    // Serial.print("Response: ");
    // for (int i = 0; i < 9; i++) {
    //   Serial.print(response[i], HEX);
    //   Serial.print(" ");
    // }
    // Serial.println();

    // // Check the auto-calibration status from the response
    // if (response[1] == 0x79 && response[2] == 0x00) {
    //   Serial.println("Auto-calibration is disabled.");
    // } else if (response[1] == 0x79 && response[2] == 0x01) {
    //   Serial.println("Auto-calibration is enabled.");
    // } else {
    //   Serial.println("Unknown response.");
    // }
    // Clear the reset flags
    MCUSR = 0;
    // Enable the Watchdog Timer
    wdt_enable(WDTO_8S);
    // Set range to 0-100000 ppm
    // setMeasurementRange100000();
    requestMeasurementRange();
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

    // CO2i = mySensorCO2.getppm();
    CO2i = static_cast<int32_t>(mySensorCO2.getppm());
    if (mySensorCO2.getppm()) {
      CO2i = mySensorCO2.ppm;
    } else {
      CO2i = -1;
      Serial.println("Nao consegui coletar ppm");
    }
    // Cálculo do tempo desde a última leitura
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - lastTime; // Diferença de tempo em milissegundos

    // Cálculo da taxa de aumento de CO2 se o deltaTime não for zero
    if (deltaTime > 0) {
        co2Rate = (CO2i - last_co2i) / (deltaTime / 1000.0); // ppm por segundo
        if(co2Rate > maxRate && last_co2i > 0){
          maxRate = co2Rate;
        }
        // Acumular a mudança de CO2
        co2AccumulatedChange += (CO2i - last_co2i);
    } else {
        co2Rate = 0;
    }

    if((CO2i - co2_before_pump) > max_co2_increase){
      max_co2_increase = CO2i - co2_before_pump;
    }
    // float averageChangePerMinute = 0.0;
    // Verificar se passou um minuto para calcular a média
    if (currentTime - lastMinuteTime >= oneMinuteInterval) {
        float averageChangePerMinute = co2AccumulatedChange / (oneMinuteInterval / 1000.0); // ppm/min

        Serial.print("Média de aumento/queda de CO2 por minuto: ");
        // Serial.print(averageChangePerMinute);
        Serial.print(co2AccumulatedChange);
        Serial.println(" ppm/min");

        // Reiniciar acumulador e tempo
        co2AccumulatedChange = 0;
        lastMinuteTime = currentTime;
    }
    // int delayProporcional = calcularDelayProporcional(maxRate, CO2i);
    int32_t delayProporcional = calcularDelayProporcional(CO2i, CO2_LIMIT, delayProporcionalAnt, max_co2_increase);
    
    // Imprimir a taxa de aumento de CO2 no monitor serial
    Serial.print(co2AccumulatedChange);
    Serial.print(" ppm/accumulated\t");
    Serial.print(co2Rate);
    Serial.print(" ppm/s\t");
    Serial.print("Maxrate: ");
    Serial.print(maxRate);
    Serial.print("\t Contador: ");
    lastTime = currentTime; // Atualiza o tempo da última leitura
    Serial.print(contador_pomp);
    Serial.print("\t");
    // Serial.println(mySensorCO2.getppm());
    Serial.print(CO2i);
    Serial.print(" ppm\t Valvula: ");
    Serial.print(delayProporcional);
    Serial.print(" ms\t Ultimaabertura: ");
    Serial.print(delayProporcionalAnt);
    Serial.print(" ms\t MAX_CO2_Increased: ");
    Serial.print(max_co2_increase);
    Serial.println(" ppm");

    

    
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    float t_interna = dhtint.readTemperature();      
    int botao_co2_status = analogRead(BOTAO_CO2);
    int botao_aquecedor_status = analogRead(BOTAO_AQUECEDOR);
    // Serial.println(t_interna);
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
          // int delayProporcional = calcularDelayProporcional(maxRate, CO2i);
          co2_before_pump = last_co2i; 
          digitalWrite(CO2PUMP,LOW);  
          // delay(10);
          if(delayProporcional > DELAY_MAXIMO){
            delayProporcional = DELAY_MAXIMO;
          } else if (delayProporcional < DELAY_MINIMO){
            delayProporcional = DELAY_MINIMO;
          }
          if(first_pomp){
            first_pomp = false;
            delay(DELAY_MINIMO);
            delayProporcionalAnt = DELAY_MINIMO;
            Serial.print("Primeiro POMP: ");
            Serial.println(DELAY_MINIMO);
          } else {            
            Serial.print("Delay de ");
            Serial.println(delayProporcional);
            delay(delayProporcional);
            delayProporcionalAnt = delayProporcional;
          }
          max_co2_increase = 0;
          digitalWrite(CO2PUMP,HIGH);
          contador_pomp = POMP_WAITING_TIME;
          reinitializeScreen();
          maxRate = 0;
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
      contador_pomp = 5;
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
        first_pomp = true;
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
    // delay(60000);
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

void disableAutoCalibration() {
  // Command to disable auto-calibration
  // byte disableAutoCalib[] = {0xFF, 0x01, 0x79, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x86}; // Adjust checksum as needed
  byte disableAutoCalib[] = {0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86}; // Adjust checksum as needed
  mySerialCO2.write(disableAutoCalib, 9);

  // Wait for a response (optional, for debugging)
  delay(500);
  while (mySerialCO2.available()) {
    byte response = mySerialCO2.read();
    Serial.print(response, HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void verifyAutoCalibration() {
  // Command to check auto-calibration status
  // byte checkAutoCalib[] = {0xFF, 0x01, 0x7D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82}; // Adjust checksum as needed
  // mySerialCO2.write(checkAutoCalib, 9);

  // Wait for a response
  delay(500);
  byte response[9];
  int index = 0;
  while (mySerialCO2.available() && index < 9) {
    response[index++] = mySerialCO2.read();
  }

  // Print the response
  Serial.print("Auto-Calibration Response: ");
  for (int i = 0; i < 9; i++) {
    Serial.print(response[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Check the auto-calibration status from the response
  if (response[2] == 0x79 && response[3] == 0x00) {
    Serial.println("Auto-calibration is disabled.");
  } else if (response[1] == 0x7D && response[2] == 0x01) {
    Serial.println("Auto-calibration is enabled.");
  } else {
    Serial.println("Unknown response.");
  }
}


void requestMeasurementRange() {
    // Command to request the measurement range (example command, adjust as per documentation)
    byte requestRangeCommand[] = {0xFF, 0x01, 0x7D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82};

    mySerialCO2.write(requestRangeCommand, 9);
    delay(500); // Allow time for the sensor to respond

    // Read the response
    byte response[9];
    int index = 0;
    while (mySerialCO2.available() && index < 9) {
        response[index++] = mySerialCO2.read();
    }

    // Log all response bytes
    Serial.print("Response: ");
    for (int i = 0; i < index; i++) {
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // Parse the response to determine the range
    if (index == 9 && response[0] == 0xFF && response[1] == 0x7D) {
        int range = (response[2] << 8) | response[3]; // Combine two bytes for range
        Serial.print("Range: ");
        Serial.println(range);
    } else {
        Serial.println("Failed to read measurement range");
    }
}

void setMeasurementRange100000() {
    // Command to set the measurement range to 100000 ppm
    // The command bytes here are hypothetical and need to be adjusted based on the sensor's documentation
    byte setRange100000[] = {0xFF, 0x01, 0x99, 0x00, 0x86, 0xA0, 0x00, 0x00, 0x00};

    // Calculate the checksum
    setRange100000[8] = calculateChecksum(setRange100000);

    mySerialCO2.write(setRange100000, 9);
    delay(100); // Allow some time for sensor to process

    // Read response (if any)
    while (mySerialCO2.available()) {
        byte response = mySerialCO2.read();
        Serial.print(response, HEX);
        Serial.print(" ");
    }
    Serial.println();
}

byte calculateChecksum(byte packet[]) {
    byte checksum = 0xFF;
    for (int i = 1; i < 8; i++) {
        checksum -= packet[i];
    }
    return checksum;
}


// Função para calcular o tempo de abertura
int32_t calcularDelayProporcional(int32_t ppm_atual, int32_t ppm_objetivo, int32_t ms_ultima_vez, int32_t aumento_ultima_vez) {
  Serial.print(ppm_atual);
  Serial.print("\t");
  Serial.print(ppm_objetivo);
  Serial.print("\t");
  Serial.print(ms_ultima_vez);
  Serial.print("\t");
  Serial.print(aumento_ultima_vez);
  Serial.println("\t");
  int32_t preciso_aumentar = ppm_objetivo - ppm_atual;
  int32_t tempo_valvula = preciso_aumentar * ms_ultima_vez / aumento_ultima_vez;
 
  return tempo_valvula;
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