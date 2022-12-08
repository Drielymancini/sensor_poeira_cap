/* ==============================================================================

  Projeto Sensor de Poeira

  ESP-WROOM-32
  Board: DevKitV1
  Compilador: Arduino IDE 1.8.4

  Autor: Driely Mancini
  Data: Setembro de 2022

================================================================================== */  

// ==================================================================================
// --- Bibliotecas ---
#include <Arduino.h>
#include <nvs_flash.h>
#include <string>
#include <stdio.h>
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
// ==================================================================================

// *************************** Variaveis para memoria flash *****************************//
/* Definição - baudrate da serial de debug */
//#define BAUDRATE_SERIAL_DEBUG    115200
 
/* Chave atribuida ao valor a ser escrito e lido
   da partição NVS */
#define CHAVE_NVS  "dado_leitura"

/* Protótipos */
void grava_dado_nvs(float dado);
float le_dado_nvs(void);
//===================================================================================================


// *************************** CRIAÇÃO DOS HANDLES DAS TAREFAS 1, 2 , 3 E 4 *****************************//

TaskHandle_t TaskGerenciaLeitura;
//===================================================================================================

// *************************** FUNÇÕES DE TASK ***********************************************//
void gerenciaLeitura(void * pvParameters);

// ==================================================================================
// --- Mapeamento de Hardware ---
#define sensor_VA 27

// --- Variáveis Globais ---
int     timer_val=15;                               //valor de contagem do timer
bool    timer_running=1,                              //flag para indicar quando start foi pressionado
        primeira_leitura = 0;
int sujo = 0;
uint8_t val=0;
bool    ctrl=0;

static uint8_t mydata[4];
static osjob_t sendjob;
void do_send(osjob_t* j);

const unsigned TX_INTERVAL = 10;

// ==================================================================================
// --- Configuração LoraWan ---

static const u1_t PROGMEM APPEUI[8]= { 0x12, 0x10, 0x03, 0x10, 0x16, 0x05, 0x26, 0x18 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]= {0x48, 0x74, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x4F, 0x31, 0xFE, 0x9D, 0xA0, 0x59, 0xFE, 0x43, 0xC8, 0x31, 0x72, 0x18, 0x6A, 0xCE, 0x2A, 0xB6};
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}


// ==================================================================================
// --- Mapeamento de funções ---

uint8_t  sine_wave[256] = {
  0x80, 0x83, 0x86, 0x89, 0x8C, 0x90, 0x93, 0x96,
  0x99, 0x9C, 0x9F, 0xA2, 0xA5, 0xA8, 0xAB, 0xAE,
  0xB1, 0xB3, 0xB6, 0xB9, 0xBC, 0xBF, 0xC1, 0xC4,
  0xC7, 0xC9, 0xCC, 0xCE, 0xD1, 0xD3, 0xD5, 0xD8,
  0xDA, 0xDC, 0xDE, 0xE0, 0xE2, 0xE4, 0xE6, 0xE8,
  0xEA, 0xEB, 0xED, 0xEF, 0xF0, 0xF1, 0xF3, 0xF4,
  0xF5, 0xF6, 0xF8, 0xF9, 0xFA, 0xFA, 0xFB, 0xFC,
  0xFD, 0xFD, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFD,
  0xFD, 0xFC, 0xFB, 0xFA, 0xFA, 0xF9, 0xF8, 0xF6,
  0xF5, 0xF4, 0xF3, 0xF1, 0xF0, 0xEF, 0xED, 0xEB,
  0xEA, 0xE8, 0xE6, 0xE4, 0xE2, 0xE0, 0xDE, 0xDC,
  0xDA, 0xD8, 0xD5, 0xD3, 0xD1, 0xCE, 0xCC, 0xC9,
  0xC7, 0xC4, 0xC1, 0xBF, 0xBC, 0xB9, 0xB6, 0xB3,
  0xB1, 0xAE, 0xAB, 0xA8, 0xA5, 0xA2, 0x9F, 0x9C,
  0x99, 0x96, 0x93, 0x90, 0x8C, 0x89, 0x86, 0x83,
  0x80, 0x7D, 0x7A, 0x77, 0x74, 0x70, 0x6D, 0x6A,
  0x67, 0x64, 0x61, 0x5E, 0x5B, 0x58, 0x55, 0x52,
  0x4F, 0x4D, 0x4A, 0x47, 0x44, 0x41, 0x3F, 0x3C,
  0x39, 0x37, 0x34, 0x32, 0x2F, 0x2D, 0x2B, 0x28,
  0x26, 0x24, 0x22, 0x20, 0x1E, 0x1C, 0x1A, 0x18,
  0x16, 0x15, 0x13, 0x11, 0x10, 0x0F, 0x0D, 0x0C,
  0x0B, 0x0A, 0x08, 0x07, 0x06, 0x06, 0x05, 0x04,
  0x03, 0x03, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x03,
  0x03, 0x04, 0x05, 0x06, 0x06, 0x07, 0x08, 0x0A,
  0x0B, 0x0C, 0x0D, 0x0F, 0x10, 0x11, 0x13, 0x15,
  0x16, 0x18, 0x1A, 0x1C, 0x1E, 0x20, 0x22, 0x24,
  0x26, 0x28, 0x2B, 0x2D, 0x2F, 0x32, 0x34, 0x37,
  0x39, 0x3C, 0x3F, 0x41, 0x44, 0x47, 0x4A, 0x4D,
  0x4F, 0x52, 0x55, 0x58, 0x5B, 0x5E, 0x61, 0x64,
  0x67, 0x6A, 0x6D, 0x70, 0x74, 0x77, 0x7A, 0x7D
};
/* Função: grava na NVS um dado do tipo interio 32-bits
 *         sem sinal, na chave definida em CHAVE_NVS
 * Parâmetros: dado a ser gravado
 * Retorno: nenhum
 */
void grava_dado_nvs(float dado)
{
    nvs_handle handler_particao_nvs;
    esp_err_t err;

    char dado_8b[8];
    //gcvt(dado, 6, dado_8b)
    dtostrf(dado, 6, 2, dado_8b);
    //char dado_convertido = dado_8b[5];

    err = nvs_flash_init_partition("nvs");
     
    if (err != ESP_OK)
    {
        Serial.println("[ERRO] Falha ao iniciar partição NVS.");           
        return;
    }
 
    err = nvs_open_from_partition("nvs", "ns_nvs", NVS_READWRITE, &handler_particao_nvs);
    if (err != ESP_OK)
    {
        Serial.println("[ERRO] Falha ao abrir NVS como escrita/leitura"); 
        return;
    }
 
    /* Atualiza valor do horimetro total */
    err = nvs_set_str(handler_particao_nvs, CHAVE_NVS, dado_8b);
    Serial.println("GRAVADO : ");  
    Serial.println("\n");
    Serial.println(dado_8b);  
 
    if (err != ESP_OK)
    {
        Serial.println("[ERRO] Erro ao gravar horimetro");                   
        nvs_close(handler_particao_nvs);
        return;
    }
    else
    {
        Serial.println("Dado gravado com sucesso!");     
        nvs_commit(handler_particao_nvs);    
        nvs_close(handler_particao_nvs);      
    }
}
 
/* Função: le da NVS um dado do tipo interio 32-bits
 *         sem sinal, contido na chave definida em CHAVE_NVS
 * Parâmetros: nenhum
 * Retorno: dado lido
 */
float le_dado_nvs(void)
{
    nvs_handle handler_particao_nvs;
    esp_err_t err;
    char dado_lido;
    size_t required_size;

    err = nvs_flash_init_partition("nvs");
     
    if (err != ESP_OK)
    {
        Serial.println("[ERRO] Falha ao iniciar partição NVS.");         
        return 0;
    }
 
    err = nvs_open_from_partition("nvs", "ns_nvs", NVS_READWRITE, &handler_particao_nvs);
    if (err != ESP_OK)
    {
        Serial.println("[ERRO] Falha ao abrir NVS como escrita/leitura");         
        return 0;
    }
 
    /* Faz a leitura do dado associado a chave definida em CHAVE_NVS */
    err = nvs_get_str(handler_particao_nvs, CHAVE_NVS, &dado_lido, &required_size);
     
    if (err != ESP_OK)
    {
        Serial.println("[ERRO] Falha ao fazer leitura do dado");         
        return 0;
    }
    else
    {
        Serial.println("Dado lido com sucesso!");  
        nvs_close(handler_particao_nvs);  
        float dado_convertido = atof(&dado_lido); 
        Serial.println("LIDO : ");  
        Serial.println(dado_convertido);  
        return dado_convertido;
    }
} //end le_dado_nvs

float medirCAP () {
  

  static double volts_f = 0.0000;
  volts_f = ((analogRead(sensor_VA)* 3.3) / 4095) + 0.1; //calculada a tensao a partir da média
  Serial.println("leitura CAP:");
  Serial.println(volts_f);
  Serial.print("entrada cap digital:");
  Serial.println(analogRead(sensor_VA));

  return volts_f;
  //Serial.println(analogRead(sensor_VB)-analogRead(sensor_VA));
  //return (analogRead(sensor_VB)-analogRead(sensor_VA));
}
void envia_Dados (uint32_t leitura1, int estado) {
 Serial.println("Leitura1");
  Serial.println(leitura1);
  
  mydata[0] = (leitura1 >> 16) & 0xFF;
  mydata[1] = (leitura1 >> 8) & 0xFF;
  mydata[2] = leitura1 & 0xFF;

  mydata[3] = (estado >> 8) & 0xFF;

  Serial.println("envia_Dados");
  Serial.println("\n");

};
void compara (double val1) {
 
  if(val1>0.2)
  {
    sujo = 1;
  }

   if(val1<=0.2)
  {
    sujo = 0;
  }
 
 
  //Serial.println("temporizador");
  //Serial.println("\n");
};

const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {2, 15,4},
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {  
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


// *************************** CONFIGURAÇÕES DO PROGRAMA *****************************//
void setup() {
  Serial.begin(9600);                   // Inicia a comunicação serial
  Serial.begin(115200);

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setLinkCheckMode(0);
  LMIC_setDrTxpow(DR_SF7, 14);
  LMIC_selectSubBand(1);
  do_send(&sendjob);
#ifdef VCC_ENABLES
   // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

  // Cria uma tarefa que será executada na função gerenciaLeitura(), com  prioridade 1 e execução no núcleo 0
    xTaskCreatePinnedToCore(
    gerenciaLeitura,     /* Função da tarefa */
    "TaskGerenciaLeitura",   /* nome da tarefa */
    10000,               /* Tamanho (bytes) */
    NULL,               /* parâmetro da tarefa */
    1,                 /* prioridade da tarefa */
    &TaskGerenciaLeitura,        /* observa a tarefa criada */
    0);            /* tarefa alocada ao núcleo 0 */

  delay(500);

  // Cria uma tarefa que será executada na função gerenciaLeitura(), com  prioridade 1 e execução no núcleo 0
  //   xTaskCreatePinnedToCore(
  //   DAC,                  /* Função da tarefa */
  //   "TaskDAC",           /* nome da tarefa */
  //   10000,              /* Tamanho (bytes) */
  //   NULL,              /* parâmetro da tarefa */
  //   1,                /* prioridade da tarefa */
  //   &TaskDAC,        /* observa a tarefa criada */
  //   1);             /* tarefa alocada ao núcleo 0 */

  // delay(500);

}//end setup

// *************************** DEFINIÇÃO DA TAREFA gerenciaLeitura  *****************************//
void gerenciaLeitura( void * pvParameters ) {
  for (;;) {                                                  // Cria um loop infinito, para a tarefa sempre ser executada quando estiver disponível.
    timer_val--;                           
  delay(1000);                          

  if(timer_val <= 0)                         
  {                                       
    timer_val=0;    
      double leitura1 = medirCAP();
      grava_dado_nvs(leitura1);
      timer_val = 30;
      Serial.println("PRIMEIRA LEITURA");
      Serial.println("\n");

      double valorArmazenado = le_dado_nvs();

      compara(valorArmazenado);
      // buscar leitura armazenada e comparar com leitura2
      //comparar leituras sujo = compara(val1, val2)
       
      if(sujo ==1){
        Serial.println("SUJO");
        Serial.println("\n");
        //reset de dados
        nvs_flash_erase_partition("nvs");
      } else {
        Serial.println("LIMPO");
        Serial.println("\n");
        
        //reset de dados
        nvs_flash_erase_partition("nvs"); 
      }
      envia_Dados(100.0*valorArmazenado, sujo);
      timer_val = 30; //somente setar novo tempo quando nescessario aguardar 15 min
    }
  } //end if !timer_val
    vTaskDelay(1000);
  }

// *************************** DEFINIÇÃO LOOP *****************************//
void loop() {
  val = val+2;
  dacWrite(25,sine_wave[val]);
   os_runloop_once();  
}//end loop