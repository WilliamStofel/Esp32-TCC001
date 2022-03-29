//Programa: NodeMCU e MQTT - Controle e Monitoramento IoT
//Autor: Pedro Bertoleti
//Autor: Fábio Henrique Cabrini
//Resumo: Esse programa possibilita ligar e desligar o led onboard, além de mandar o status para o Broker MQTT possibilitando o Helix saber
//se o led está ligado ou desligado.
#include <DHT.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
//#include <esp_ipc.h>



 #include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>

#include <PubSubClient.h> // Importa a Biblioteca PubSubClient
#include <Arduino_JSON.h>


#define DHTPIN 4          // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11     // DHT 11
DHT dht(DHTPIN, DHTTYPE); // Initialing DHT Class
 
//defines:
//defines de id mqtt e tópicos para publicação e subscribe
#define TOPICO_SUBSCRIBE "/iot/smartirrigation005/cmd"     //tópico MQTT de escuta
#define TOPICO_PUBLISH_AIRTEMPARATURE_HUMIDITY   "urn:ngsi-ld:smartirrigation:005"    //tópico MQTT de envio de informações para Broker
#define TOPICO_PUBLISH   "/iot/smartirrigation005/attrs"    //tópico MQTT de envio de informações para Broker

                                                   //IMPORTANTE: recomendamos fortemente alterar os nomes
                                                   //            desses tópicos. Caso contrário, há grandes
                                                   //            chances de você controlar e monitorar o NodeMCU
                                                   //            de outra pessoa.
#define ID_MQTT  "helix"     //id mqtt (para identificação de sessão)
                               //IMPORTANTE: este deve ser único no broker (ou seja, 
                               //            se um client MQTT tentar entrar com o mesmo 
                               //            id de outro já conectado ao broker, o broker 
                               //            irá fechar a conexão de um deles).


HTTPClient http;
#define BOMBA 17
#define SENSOR  15 //sensor vazao
#define SENSORSOLO  35 //sensor umidade da terra
float porcentagem;
boolean StatusOn = false;
boolean StatusOff = true;

// Setting variables to measurement
long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
int Qtd = 0;
boolean ledState = LOW;
String Aux = "NAO";
float calibrationFactor = 4.5;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;

// Pulse counter
void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}

float meanTemperature;
float meanHumidity;


String sensorReadings;


const char* serverName = "http://18.216.218.224:4041/iot/devices/smartirrigation005";

const char *orionAddressPath = "18.216.218.224:1026/v2";


// WIFI
const char* SSID = "MAIA"; // SS0eseja se conectar
  
// MQTT
const char* BROKER_MQTT = "18.216.218.224"; //URL do broker MQTT que se deseja utilizar
int BROKER_PORT = 1883; // Porta do Broker MQTT
 
 
//Variáveis e objetos globais
WiFiClient espClient; // Cria o objeto espClient
PubSubClient MQTT(espClient); // Instancia o Cliente MQTT passando o objeto espClient
char EstadoSaida = '0';  //variável que armazena o estado atual da saída
  
//Prototypes
void initSerial();
void initWiFi();
void initMQTT();
void reconectWiFi(); 
void mqtt_callback(char* topic, byte* payload1, unsigned int length);
void VerificaConexoesWiFIEMQTT(void);
void InitOutput(void);
 
/* 
 *  Implementações das funções
 */
void setup() 
{
    //inicializações:
    InitOutput();
    initSerial();
    initWiFi();
    initMQTT();
    pinMode(SENSOR, INPUT_PULLUP);
   
    pulseCount = 0;
    flowRate = 0.0;
    flowMilliLitres = 0;
    totalMilliLitres = 0;
    previousMillis = 0;

    attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);
    
     xTaskCreatePinnedToCore(
                    SensorUmidadeSolo,   /* Task function. */
                    "SensorUmidadeSolo",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    NULL,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
    delay(500); 
    
    xTaskCreatePinnedToCore(
                    SensorTemperatura,   /* Task function. */
                    "SensorTemperatura",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    8,           /* priority of the task */
                    NULL,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
    delay(500); 
    
    
    xTaskCreatePinnedToCore(
                    SensorVazao,   /* Task function. */
                    "SensorVazao",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    4,           /* priority of the task */
                    NULL,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
    delay(500); 
}
  
//Função: inicializa comunicação serial com baudrate 115200 (para fins de monitorar no terminal serial 
//        o que está acontecendo.
//Parâmetros: nenhum
//Retorno: nenhum
void initSerial() 
{
   
    Serial.begin(115200);
    dht.begin();
}






 
//Função: inicializa e conecta-se na rede WI-FI desejada
//Parâmetros: nenhum
//Retorno: nenhum
void initWiFi() 
{
    delay(10);
    Serial.println("------Conexao WI-FI------");
    Serial.print("Conectando-se na rede: ");
    Serial.println(SSID);
    Serial.println("Aguarde");
     
    reconectWiFi();
}
  
//Função: inicializa parâmetros de conexão MQTT(endereço do 
//        broker, porta e seta função de callback)
//Parâmetros: nenhum
//Retorno: nenhum
void initMQTT() 
{
    
    MQTT.setServer(BROKER_MQTT, BROKER_PORT);   //informa qual broker e porta deve ser conectado
    MQTT.setCallback(mqtt_callback);            //atribui função de callback (função chamada quando qualquer informação de um dos tópicos subescritos chega)
}
  
//Função: função de callback 
//        esta função é chamada toda vez que uma informação de 
//        um dos tópicos subescritos chega)
//Parâmetros: nenhum
//Retorno: nenhum
void mqtt_callback(char* topic, byte* payload1, unsigned int length) 
{
    
    
    String msg;

    Serial.println("Entrou no mqtt callback");
    //obtem a string do payload recebido
    for(int i = 0; i < length; i++) 
    {
       char c = (char)payload1[i];
       msg += c;
    }

   
   Serial.println("mqtt callback: " + msg);
        
    //toma ação dependendo da string recebida:
    //verifica se deve colocar nivel alto de tensão na saída D0:
    //IMPORTANTE: o Led já contido na placa é acionado com lógica invertida (ou seja,
    //enviar HIGH para o output faz o Led apagar / enviar LOW faz o Led acender)
    if (msg.equals("smartirrigation005@on|"))
    {
        
        Serial.println("Ligando Bomba");
        digitalWrite(BOMBA, LOW);
        EstadoSaida = '0';
        Aux = "SIM";
        StatusOn = true;
        StatusOff = false;
    }
 
    //verifica se deve colocar nivel alto de tensão na saída D0:
    if (msg.equals("smartirrigation005@off|"))
    {
        Serial.println("Desligando Bomba");
        digitalWrite(BOMBA, HIGH);
        EstadoSaida = '1';
        Aux = "SIM";
        StatusOn = false;
        StatusOff = true;
    }
     
}
  
//Função: reconecta-se ao broker MQTT (caso ainda não esteja conectado ou em caso de a conexão cair)
//        em caso de sucesso na conexão ou reconexão, o subscribe dos tópicos é refeito.
//Parâmetros: nenhum
//Retorno: nenhum
void reconnectMQTT() 
{
    while (!MQTT.connected()) 
    {
        Serial.print("* Tentando se conectar ao Broker MQTT: ");
        initMQTT();
        Serial.println(BROKER_MQTT);
        if (MQTT.connect(ID_MQTT)) 
        {
            Serial.println("Conectado com sucesso ao broker MQTT!");
            MQTT.subscribe(TOPICO_SUBSCRIBE); 
        } 
        else
        {
            Serial.println("Falha ao reconectar no broker.");
            Serial.println("Havera nova tentatica de conexao em 2s");
            delay(2000);
        }
    }
}
  
//Função: reconecta-se ao WiFi
//Parâmetros: nenhum
//Retorno: nenhum
void reconectWiFi() 
{
    //se já está conectado a rede WI-FI, nada é feito. 
    //Caso contrário, são efetuadas tentativas de conexão
    if (WiFi.status() == WL_CONNECTED)
        return;
         
    WiFi.begin(SSID, PASSWORD); // Conecta na rede WI-FI
     
    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(100);
        Serial.print(".");
    }
   
    Serial.println();
    Serial.print("Conectado com sucesso na rede ");
    Serial.print(SSID);
    Serial.println("IP obtido: ");
    Serial.println(WiFi.localIP());
}
 
//Função: verifica o estado das conexões WiFI e ao broker MQTT. 
//        Em caso de desconexão (qualquer uma das duas), a conexão
//        é refeita.
//Parâmetros: nenhum
//Retorno: nenhum
void VerificaConexoesWiFIEMQTT(void)
{
    
    if (!MQTT.connected()) 
        reconnectMQTT(); //se não há conexão com o Broker, a conexão é refeita
     
     reconectWiFi(); //se não há conexão com o WiFI, a conexão é refeita
}
 
//Função: envia ao Broker o estado atual do output 
//Parâmetros: nenhum
//Retorno: nenhum
void EnviaEstadoOutputMQTT(void)
{
    Serial.println(Aux);
    
    if(Aux == "SIM")
    {
      if (EstadoSaida == '1')
      {
        Aux = "NAO";
        Serial.println("ON");
        MQTT.publish(TOPICO_PUBLISH,"on_status|1");
        MQTT.publish(TOPICO_PUBLISH,"off_status|0");
      }
      else 
      {
        Aux = "NAO";
        Serial.println("OFF");
        MQTT.publish(TOPICO_PUBLISH, "off_status|1");
        MQTT.publish(TOPICO_PUBLISH, "on_status|0");
      }
    }
      
 
    //Serial.println("- Estado do LED onboard enviado ao broker!");
    delay(1000);
}
 
//Função: inicializa o output em nível lógico baixo
//Parâmetros: nenhum
//Retorno: nenhum
void InitOutput(void)
{
    //IMPORTANTE: o Led já contido na placa é acionado com lógica invertida (ou seja,
    //enviar HIGH para o output faz o Led apagar / enviar LOW faz o Led acender)
    pinMode(BOMBA, OUTPUT);
    digitalWrite(BOMBA, HIGH);          
}


 void SensorTemperatura(void *arg)
 {
  while(1) {
    Serial.print("SensorTemperatura running on core ");
    Serial.println(xPortGetCoreID());
    int AuxiliarTemperatura =5;
    int AuxiliarHumidade = 5;
    float totalTemperature = 0;
    float totalHumidity = 0;
    //meanTemperature = 0;
    //meanHumidity = 0;
    
    for (short i = 0; i < 5; i++)
    {
        // Wait a few seconds before measurements
        delay(5000);

        float actualTemperature = dht.readTemperature(false);
        // Wait a few seconds between measurements
        delay(10);
        float actualHumidity = dht.readHumidity();

        if (isnan(actualTemperature))
        {
            actualTemperature = 0;
        }
        if (isnan(actualHumidity))
        {
            actualHumidity = 0;
        }

        totalHumidity += actualHumidity;
        totalTemperature += actualTemperature;

        Serial.print("COUNT[" + String(i + 1) + "] - ");
        Serial.print("Humidity: " + String(actualHumidity) + "% - ");
        Serial.print("Total Humidity: " + String(totalHumidity) + "°C  - ");
        Serial.print("Temperature: " + String(actualTemperature) + "°C - ");
        Serial.println("Total Temperature: " + String(totalTemperature) + "°C");

        
        if(actualTemperature == 0.00) 
          AuxiliarTemperatura--;
        if(actualHumidity == 0.00)
          AuxiliarHumidade --;
    }

    
    // Calculation of average values
    meanTemperature = totalTemperature /  AuxiliarTemperatura;
    meanHumidity = totalHumidity /  AuxiliarHumidade;
  
    Serial.println("Mean after 5 reads is Humidity: " + String(meanHumidity) + "% - Temperature: " + String(meanTemperature) + "°C");

    char msgHumidity[10];
    char msgTemperature[10];
    sprintf(msgHumidity, "%d", meanHumidity);
    sprintf(msgTemperature, "%d", meanTemperature);


   
    // Update
    if(meanHumidity > -1 and meanTemperature > -1 )
    {
      Serial.println("Updating data in orion...");
      //orionUpdate(TOPICO_PUBLISH_AIRTEMPARATURE_HUMIDITY, msgTemperature, msgHumidity); 
    }
    
    }
 }

//programa principal
void loop() 
{   
    
    VerificaConexoesWiFIEMQTT();
 
    //envia o status de todos os outputs para o Broker no protocolo esperado
    EnviaEstadoOutputMQTT();
     
    //keep-alive da comunicação com broker MQTT
    MQTT.loop();
  
}


void SensorVazao(void *arg)
{
    
   while(1) {
    Serial.print("SensorVazao running on core ");
    Serial.println(xPortGetCoreID());
    delay(1500);
   currentMillis = millis();

   if ((currentMillis - previousMillis) > interval){
      pulse1Sec = pulseCount;
      pulseCount = 0;

      flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
      previousMillis = millis();

      flowMilliLitres = (flowRate / 60) * 1000;

      totalMilliLitres += flowMilliLitres;

      Serial.print("Flow rate: ");
      Serial.print(int(flowRate));  // Print the integer part of the variable
      Serial.print("L/min");
      Serial.print("\t"); 

      Serial.print("Output Liquid Quantity: ");
      Serial.print(totalMilliLitres);
      Serial.print("mL / ");
      Serial.print(totalMilliLitres / 1000);
      Serial.println("L");

      if (flowRate < 30){
         flowRate = 0;
        
      }
      
        
   }

}
}

void SensorUmidadeSolo(void *arg)
{
   //String bodyRequest = "";
   
   char StringFinalHumiditySoil[19];
   char ConvertHumiditySoil[6];
   
   char StringFinalFlowRate[45];
   char ConvertFlowRate[12];
   
   char StringFinalTotalMilliLitres[45];
   char ConvertTotalMilliLitres[12];
   
   char StringFinalHumidity[19];
   char ConvertHumidity[12];
   
   char StringFinalTemperature[19];
   char ConvertTemperature[12];
  
  

   porcentagem = 0;
   while(1) {
    Qtd++;
    Serial.println("SensorUmidade do Solo running on core ");
    Serial.println(xPortGetCoreID());
    Serial.println("QTD:");
    Serial.println(Qtd);
    int valor_analogico = analogRead(SENSORSOLO);
    
    if(SENSORSOLO > 0 )
    {
      porcentagem = (valor_analogico/40.95)-100;
      porcentagem = porcentagem * (-1);
    }
    
    Serial.println(String("Valor sensor: ")+ valor_analogico +String(" Valor Porcentagem: ")+ porcentagem + String("%"));
    delay(1000);
    if(Qtd == 60)
    { 
      Qtd=0; 

 
   
    
    strcpy(StringFinalHumiditySoil,"humiditySoil|");
    strcpy(StringFinalFlowRate,"flow_rate|");
    strcpy(StringFinalTotalMilliLitres,"total_flow|");
    strcpy(StringFinalHumidity,"humidity|");
    strcpy(StringFinalTemperature,"temperature|");

    dtostrf(porcentagem, 4, 2, ConvertHumiditySoil);
    dtostrf(flowRate, 2, 0, ConvertFlowRate);
    dtostrf(totalMilliLitres, 2, 0, ConvertTotalMilliLitres);
    dtostrf(meanHumidity, 4, 2, ConvertHumidity);
    dtostrf(meanTemperature, 4, 2, ConvertTemperature);

    
 

    strcat(StringFinalHumiditySoil,ConvertHumiditySoil);
    strcat(StringFinalFlowRate,ConvertFlowRate);
    strcat(StringFinalTotalMilliLitres,ConvertTotalMilliLitres);
    strcat(StringFinalHumidity,ConvertHumidity);
    strcat(StringFinalTemperature,ConvertTemperature);

    
      
      
      MQTT.publish(TOPICO_PUBLISH, StringFinalHumiditySoil);
      MQTT.publish(TOPICO_PUBLISH, StringFinalTemperature);
      MQTT.publish(TOPICO_PUBLISH, StringFinalHumidity);
      MQTT.publish(TOPICO_PUBLISH, StringFinalTotalMilliLitres);
      MQTT.publish(TOPICO_PUBLISH, StringFinalFlowRate);
    
      Serial.println(String("VALOR ENVIADO MQTT ") + StringFinalHumiditySoil + String("VALOR PORCENTAGEM ") + porcentagem);
      
      Serial.println(String("Temperatura") + meanTemperature + String("Umidade") + meanHumidity);
      Serial.println(String("Temperatura") + StringFinalTemperature + String("Umidade") + StringFinalHumidity);

    }   


     
    
    
   }
}
