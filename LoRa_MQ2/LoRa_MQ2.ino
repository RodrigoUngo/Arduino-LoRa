/*************************************************** *******************************
 * Copyright (c) 2015 Thomas Telkamp y Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Por la presente se concede permiso, de forma gratuita, a cualquier persona
 * obtener una copia de este documento y los archivos adjuntos,
 * hacer lo que quieran con ellos sin ninguna restricción,
 * incluyendo, entre otros, copia, modificación y redistribución.
 *NO SE OFRECE NINGUNA GARANTÍA DE NINGÚN TIPO.
 *
 * Este ejemplo envía un paquete LoRaWAN válido con el payload "Hola,
 * ¡mundo!", utilizando configuraciones de frecuencia y cifrado que coincidan con las de
 * The Things Network.
 *
 * Esto utiliza OTAA (Over-the-air activation), donde un DevEUI y
 * AppKey se configuran, las cuales se utilizan de forma Over-the-air
 * activation donde un DevAddr y las claves de sesión son
 * asignados/generados para su uso con todas las comunicaciones posteriores.
 *
 * Nota: Se aplica la limitación del ciclo de trabajo de LoRaWAN por subbanda (1% en
 * g1, 0,1% en g2), pero no la política de uso justo de TTN (que probablemente sea
 * ¡violado por este boceto cuando se deja funcionando por más tiempo)!

 * Para utilizar este boceto, primero registre su aplicación y dispositivo con
 * The Things Network, para configurar o generar una AppEUI, DevEUI y AppKey.
 * Varios dispositivos pueden usar la misma AppEUI, pero cada dispositivo tiene la suya propia
 * DevEUI y AppKey.
 *
 * No olvides definir correctamente el tipo de radio en
 * arduino-lmic/project_config/lmic_project_config.h o desde tu BOARDS.txt.
 *
 ************************************************** *******************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

//
// Para uso normal, requerimos que edites el boceto para reemplazar FILLMEIN
// con valores asignados por la consola TTN. Sin embargo, para las pruebas de regresión,
// queremos poder compilar estos scripts. Las pruebas de regresión definen
// COMPILE_REGRESSION_TEST, y en ese caso definimos FILLMEIN como un no-
// valor funcional pero inofensivo.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "¡Debes reemplazar los valores marcados como FILLMEIN con valores reales del panel de control de TTN!"
# define FILLMEIN (#no edites esto, edita las líneas que usan FILLMEIN)
#endif

// Este EUI debe estar en formato little-endian, por lo que es el byte menos significativo
// primero. Al copiar un EUI del output ttnctl, esto significa invertir
// los bytes. En TTN se invierten los bytes con la opción lsb.
static const u1_t PROGMEM APPEUI[8]={ 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// Esto también debe de estar en formato little endian, ver el comentario anterior.
static const u1_t PROGMEM DEVEUI[8]={ 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// Esta clave debe estar en formato big endian (o, dado que en realidad no es un
// número pero un bloque de memoria, la endianidad realmente no se aplica). En
// práctica, una clave tomada de ttnctl se puede copiar tal cual está.
static const u1_t PROGMEM APPKEY[16] = { 0xDC, 0x78, 0xDF, 0xA9, 0x14, 0x88, 0x69, 0xCE, 0x15, 0x19, 0x05, 0x53, 0x30, 0xD9, 0x4A, 0xCB };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// Programe TX cada tantos segundos (puede alargarse debido a las
// limitaciones del ciclo de trabajo).
const unsigned TX_INTERVAL = 60;

// Variables usadas para la lectura y cálculo de las
// partes por millón de gas en el ambiente

// Pin análogo A0
int MQ2 = A0;
// Voltaje recibido por el pin A0
float sensor_volt; 
// Lectura de la cantidad de gas
float RS_gas;
// Ratio de cantidad de gas vs ambiente
float ratio;
// Resistencia de carga de 1k Ohm
// Varia de modelo en modelo
float RL=1000;  
// Partes por millon de gas metano, butano, LPG y humo
float PPM;
// String que contiene el valor que sera enviado por LoRa
String payload;

// Mapeo de pines
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, LMIC_UNUSED_PIN},
};


void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

// Función que detecta el caso a ejecutar dependiendo
// de lo que envíe el módulo LoRa
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        // Tiempo de espera máximo alcanzado
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        // Fuente encontrada
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        // Fuente perdida
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        // Fuente rastreada
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        // Incorporándose al gateway LoRaWAN
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        // Incorporación exitosamente al gateway LoRaWAN
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              // Configuración de autenticación con el gateway LoRaWAN
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
            // Deshabilitar la validación de verificación de enlaces (habilitada automáticamente
            // durante la unión, pero debido a que las velocidades de datos lentas cambian el tamaño
            // máximo de TX, no lo usamos en este ejemplo.
            LMIC_setLinkCheckMode(0);
            break;
        // Incorporación al gateway LoRaWAN fallida
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        // Reincorporación al gateway LoRaWAN fallida
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        //Enlace con el gateway LoRaWAN completa
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            //Si 
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Programar la próxima transmisión
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        // Sincornización perdida
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        // Reiniciando
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        // Transmición de datos con RX Completado
        case EV_RXCOMPLETE:
            // Datos recibidos en la ranura de ping
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        // Enlace caído
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        // Enlace conectado
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        // Enlace TX iniciado
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        // Enlace TX cancelado
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        // Inicialización de comunicación con RX
        case EV_RXSTART:
            /* no imprimas nada aqui -- arruina el enlace */
            break;
        // Incorporación al gateway hecha pero no acepetada
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        // Evento desconocido
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Comprobar si no hay un trabajo TX/RX actual en ejecución
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Preparar la transmisión de datos ascendentes en el próximo momento posible.
        // El valor debe de ser descompuesto para ser enviado byte  por byte.
        float R0 = 1100; 
        int sensorValue = analogRead(MQ2); 
        sensor_volt = ((float)sensorValue / 1024) * 5.0; 
        RS_gas = ((5.0-sensor_volt)/sensor_volt)*RL; // Depende de RL en tu módulo
        ratio = RS_gas / R0; // ratio = RS/R0
        PPM = 585.19*(pow(ratio,-2.042));
        // Convertir a String el valor que se va a mandar
        // para descomponerlo por bytes.
        payload = String(PPM);
        // Función de LMIC para descomponer el valor
        // a enviar en formato de bytes.
        LMIC_setTxData2(1, payload.c_str(), payload.length(), 0);
        Serial.println(payload);
        Serial.println(F("Packet queued"));
    }
    // El próximo TX está programado después del evento TX_COMPLETE.
}

// Función de iniziaiación de programa
void setup() {
    //Pin de detección de datos
    pinMode(MQ2,INPUT);
    Serial.begin(9600);
    Serial.println(F("Starting"));

    #ifdef VCC_ENABLE
    // Para placas Pinoccio Scout
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

     os_init();
    // Restablecer el estado de MAC. Se descartarán las transferencias de sesión y de datos pendientes.
    LMIC_reset();
    // Deshabilite el modo de verificación de enlaces y ADR, porque ADR tiende a complicar las pruebas.
    LMIC_setLinkCheckMode(0);
    // Establezca la velocidad de datos en el factor de dispersión 7. Esta es la velocidad más rápida admitida para canales de 125 kHz y
    // minimiza el tiempo de aire y la energía de la batería. Establezca la potencia de transmisión en 14 dBi (25 mW).
    LMIC_setDrTxpow(DR_SF12,14);
    // en EE.UU., con TTN, se ahorra tiempo de incorporación si comenzamos en la subbanda 1 (canales 8-15). Esta voluntad
    // se anula después de la unión por parámetros de la red. Si trabaja con otros
    // redes o en otras regiones, esto deberá cambiarse.
    LMIC_selectSubBand(1);

    // Iniciar trabajo (el envío también inicia automáticamente el OTAA)
    do_send(&sendjob);
}

void loop() {
    // Correr el programa completo en bucle
    os_runloop_once();
}
