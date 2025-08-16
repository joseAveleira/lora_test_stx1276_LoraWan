#include <Arduino.h>
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>

// === Claves OTAA ===
// DevEUI y JoinEUI en LSB (invertido byte a byte respecto a TTN)
// ATENCIÓN: Verifica que estos valores estén invertidos (LSB) respecto a TTN
static const u1_t PROGMEM DEVEUI[8]  = { 0x34, 0x12, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // DevEUI LSB
static const u1_t PROGMEM APPEUI[8]  = { 0xCD, 0xAB, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // JoinEUI LSB
// AppKey en MSB (igual que TTN)
static const u1_t PROGMEM APPKEY[16] = {
  0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,
  0x99,0x00,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF
};

void os_getDevEui (u1_t* buf){ memcpy_P(buf, DEVEUI, 8); }
void os_getArtEui (u1_t* buf){ memcpy_P(buf, APPEUI, 8); }
void os_getDevKey (u1_t* buf){ memcpy_P(buf, APPKEY, 16); }

// Pinout SX1276 ↔ ESP32
// ATENCIÓN: Verifica que estos pines coincidan con tu wiring y tu placa
const lmic_pinmap lmic_pins = {
  .nss  = 5,                 // NSS (CS)
  .rxtx = LMIC_UNUSED_PIN,
  .rst  = 14,                // RESET
  .dio  = { 26, 33, 32 }     // DIO0, DIO1, DIO2
};

// Pin táctil capacitivo
const int touchPin = 4; // T0 en GPIO4
const int touchThreshold = 40; // Umbral de sensibilidad (ajusta si es necesario)

static osjob_t sendjob;
static osjob_t keepaliveJob;              // job para uplinks vacíos
const unsigned KEEPALIVE_INTERVAL = 15;   // segundos entre keepalives (reduce latencia)

void do_send(osjob_t* j){
  // Asegurarse de que no haya una transmisión en curso
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("[WARN] Radio ocupada, no se puede enviar ahora."));
  } else {
    static const char msg[] = "Hola LoRaWAN!";
    Serial.println(F("[INFO] Enviando paquete por pulso táctil..."));
    LMIC_setTxData2(1, (uint8_t*)msg, sizeof(msg)-1, 0); // port=1, unconfirmed
  }
  // Ya no se agenda el siguiente envío automáticamente
}

// Uplink vacío periódico para abrir las ventanas RX (Clase A)
void do_keepalive(osjob_t* j){
  if (!(LMIC.opmode & OP_TXRXPEND)){
    Serial.println(F("[INFO] Keepalive: uplink vacío para abrir RX"));
    LMIC_setTxData2(223, NULL, 0, 0); // puerto 223, payload 0 bytes
  }
  os_setTimedCallback(&keepaliveJob, os_getTime()+sec2osticks(KEEPALIVE_INTERVAL), do_keepalive);
}

void onEvent (ev_t ev){
  Serial.print(F("[LMIC] Evento: "));
  Serial.println(ev);
  switch(ev){
    case EV_JOINING:   Serial.println(F("OTAA: uniendo...")); break;
    case EV_JOINED:
      Serial.println(F("OTAA: unido!"));
      LMIC_setLinkCheckMode(0); // recomendado
      // Arranca keepalive para abrir RX sin interacción
      os_setTimedCallback(&keepaliveJob, os_getTime()+sec2osticks(KEEPALIVE_INTERVAL), do_keepalive);
      Serial.println(F("Clase A: keepalive activo. Downlinks llegarán tras cada uplink."));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("OTAA: fallo de join"));
      Serial.println(F("[ERROR] Revisa claves, cobertura, gateway y frecuencias."));
      break;
    case EV_TXCOMPLETE:
      Serial.print(F("TX completo. "));
      if (LMIC.txrxFlags & TXRX_ACK) Serial.print(F("ACK "));
      
      if (LMIC.dataLen){
        Serial.print(F("Downlink recibido de "));
        Serial.print(LMIC.dataLen);
        Serial.print(F(" bytes en el puerto "));
        Serial.println(LMIC.frame[LMIC.dataBeg-1]);
        
        Serial.print(F("  Datos (HEX): "));
        for (uint8_t i=0; i<LMIC.dataLen; i++){
          Serial.print(LMIC.frame[LMIC.dataBeg+i], HEX); Serial.print(' ');
        }
        Serial.println();

        Serial.print(F("  Datos (ASCII): "));
        for (uint8_t i=0; i<LMIC.dataLen; i++){
          Serial.print((char)LMIC.frame[LMIC.dataBeg+i]);
        }
        Serial.println();
      }
      
      Serial.println(F("Esperando siguiente toque..."));
      break;
    default:
      Serial.println(F("[INFO] Otro evento LMIC."));
      break;
  }
}

void setup(){
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("LMIC OTAA EU868 - Envio por pulso tactil"));

  // SPI por VSPI con tus pines (verifica que coincidan con tu wiring)
  SPI.begin(18, 19, 23); // SCK=18, MISO=19, MOSI=23

  os_init();
  LMIC_reset();

  // Compensación reloj en ESP32 (más tolerancia para sincronizar RX)
  LMIC_setClockError(MAX_CLOCK_ERROR / 10); // 10% tolerancia

  // DR/Potencia inicial (TTN EU868: RX2 SF9 por defecto)
  LMIC_setDrTxpow(DR_SF7, 14);
  // Asegura parámetros RX2 por claridad (TTN EU868)
  LMIC.dn2Dr = DR_SF9;
  LMIC.dn2Freq = 869525000;

  // EU868: deja todos los canales por defecto (mejor con gateways multicanal)

  // EU868: NO uses LMIC_selectSubBand (es para US915)
  LMIC_startJoining();

  Serial.println(F("[INFO] Esperando join OTAA solo por canal 0 (868.1 MHz)..."));
}

void loop(){
  os_runloop_once();

  // Leer el pin táctil
  if (touchRead(touchPin) < touchThreshold) {
    // Llamar a do_send solo si la radio está libre
    do_send(&sendjob);
    // Esperar un poco para evitar envíos múltiples por un solo toque (debounce)
    delay(2000);
  }
}
