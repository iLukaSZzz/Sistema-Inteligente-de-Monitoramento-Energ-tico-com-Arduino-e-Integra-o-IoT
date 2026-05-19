/*
 * =============================================================
 *  Monitor Energético Residencial — ESP32 + ACS712 + Blynk 2.0
 * =============================================================
 *
 * Circuitos monitorados:
 *   0 → Iluminação   (pino 34, relé 25)
 *   1 → Tomadas      (pino 35, relé 26)
 *   2 → Cozinha      (pino 32, relé 27)
 *   3 → Banheiro     (pino 33, relé 14)
 *
 * Mapa de pinos virtuais Blynk:
 *   V0–V3   → Corrente (A) por circuito
 *   V10–V13 → Energia acumulada (kWh) por circuito
 *   V20     → Tarifa (R$/kWh) — escrita pelo app
 *   V21     → Reset de energia — escrita pelo app (1 = resetar)
 *   V30     → Energia total (kWh)
 *   V31     → Custo total (R$)
 *   V40–V43 → Controle dos relés — escritos pelo app
 *
 * Dependências (Arduino IDE):
 *   - Blynk (>=1.3.2)
 *   - ESP32 board package
 *
 * Hardware:
 *   - ESP32 DevKit
 *   - 4× ACS712-20A (sensibilidade 100 mV/A)
 *   - 4× módulo relé (ativo em LOW)
 * =============================================================
 */

// --- Credenciais Blynk 2.0 -------------------------
#define BLYNK_TEMPLATE_ID   "TMPLxxxxxxxx"
#define BLYNK_TEMPLATE_NAME "Monitor Energia"
#define BLYNK_AUTH_TOKEN    "TOKEN"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// ====================================================
// CONFIGURAÇÕES
// ====================================================

const char WIFI_SSID[] = "WIFI";
const char WIFI_PASS[] = "SENHA";

// Pinos analógicos dos sensores ACS712
const int PINO_SENSOR[4] = {34, 35, 32, 33};

// Pinos digitais dos relés
const int PINO_RELE[4] = {25, 26, 27, 14};

// Nome dos circuitos
const char* NOME_CIRCUITO[4] = {
  "Iluminacao",
  "Tomadas",
  "Cozinha",
  "Banheiro"
};

// Tensão da rede elétrica
const float TENSAO_REDE = 127.0;

// Sensibilidade do ACS712-20A
const float SENSIBILIDADE_MV_A = 100.0;

// Quantidade de amostras RMS
const int AMOSTRAS_RMS = 1000;

// Limite máximo de corrente por circuito
const float LIMITE_CORRENTE[4] = {
  10.0,
  16.0,
  20.0,
  10.0
};

// Intervalo de atualização
const unsigned long INTERVALO_MS = 2000UL;

// Tarifa energética
float tarifa = 0.95;

// ====================================================
// VARIÁVEIS GLOBAIS
// ====================================================

float energiaAcumulada[4] = {0.0, 0.0, 0.0, 0.0};

bool alertaDisparado[4] = {
  false,
  false,
  false,
  false
};

bool releAtivo[4] = {
  true,
  true,
  true,
  true
};

unsigned long ultimaAtualizacao = 0;

BlynkTimer timer;

// ====================================================
// FUNÇÃO DE LEITURA RMS
// ====================================================

float lerCorrenteRMS(int pino) {

  const float VREF       = 3.3;
  const float ADC_MAX    = 4095.0;
  const float OFFSET_ADC = ADC_MAX / 2.0;

  const float V_POR_CNT  = VREF / ADC_MAX;

  const float A_POR_V =
      1000.0 / SENSIBILIDADE_MV_A;

  double somaQuadrados = 0.0;

  for (int i = 0; i < AMOSTRAS_RMS; i++) {

    int adc = analogRead(pino);

    float delta = (float)(adc - OFFSET_ADC);

    float iInst =
        delta * V_POR_CNT * A_POR_V;

    somaQuadrados +=
        (double)(iInst * iInst);
  }

  float irms =
      sqrt(somaQuadrados / AMOSTRAS_RMS);

  // Filtro de ruído
  if (irms < 0.05f) {
    irms = 0.0f;
  }

  return irms;
}

// ====================================================
// PROTEÇÃO DE SOBRECORRENTE
// ====================================================

void verificarSobrecarga(int circuito,
                         float corrente) {

  if (corrente > LIMITE_CORRENTE[circuito]) {

    // Desliga o relé
    digitalWrite(PINO_RELE[circuito], LOW);

    releAtivo[circuito] = false;

    // Atualiza botão no app
    Blynk.virtualWrite(V40 + circuito, 0);

    if (!alertaDisparado[circuito]) {

      String msg =
          String("Sobrecarga no circuito ")
          + NOME_CIRCUITO[circuito]
          + " ("
          + String(corrente, 1)
          + " A)";

      Blynk.logEvent("sobrecarga", msg);

      Serial.println("[ALERTA] " + msg);

      alertaDisparado[circuito] = true;
    }

  } else {

    alertaDisparado[circuito] = false;
  }
}

// ====================================================
// ENVIO DE DADOS AO BLYNK
// ====================================================

void enviarDadosBlynk(int circuito,
                      float corrente) {

  Blynk.virtualWrite(V0 + circuito,
                     corrente);

  Blynk.virtualWrite(V10 + circuito,
                     energiaAcumulada[circuito]);
}

// ====================================================
// ROTINA PRINCIPAL
// ====================================================

void sistema() {

  unsigned long agora = millis();

  float deltaHoras =
      (agora - ultimaAtualizacao)
      / 3600000.0;

  ultimaAtualizacao = agora;

  float energiaTotal = 0.0;

  for (int i = 0; i < 4; i++) {

    // =================================
    // 1. Leitura RMS
    // =================================

    float corrente =
        lerCorrenteRMS(PINO_SENSOR[i]);

    // =================================
    // 2. Potência elétrica
    // =================================

    float potencia =
        corrente * TENSAO_REDE;

    // =================================
    // 3. Energia acumulada
    // =================================

    energiaAcumulada[i] +=
        (potencia / 1000.0)
        * deltaHoras;

    energiaTotal +=
        energiaAcumulada[i];

    // =================================
    // 4. Proteção
    // =================================

    verificarSobrecarga(i, corrente);

    // =================================
    // 5. Atualiza Blynk
    // =================================

    enviarDadosBlynk(i, corrente);

    // =================================
    // 6. Debug Serial
    // =================================

    Serial.printf(
      "[%s] I=%.2fA  P=%.1fW  E=%.4fkWh\n",
      NOME_CIRCUITO[i],
      corrente,
      potencia,
      energiaAcumulada[i]
    );
  }

  // =================================
  // Totais gerais
  // =================================

  float custo =
      energiaTotal * tarifa;

  Blynk.virtualWrite(V30,
                     energiaTotal);

  Blynk.virtualWrite(V31,
                     custo);

  Serial.printf(
      ">>> Total: %.4f kWh | R$ %.2f\n\n",
      energiaTotal,
      custo
  );
}

// ====================================================
// CALLBACKS BLYNK
// ====================================================

// Ajuste da tarifa
BLYNK_WRITE(V20) {

  tarifa = param.asFloat();

  if (tarifa <= 0.0f) {
    tarifa = 0.01f;
  }

  Serial.printf(
      "[Config] Tarifa: R$ %.4f/kWh\n",
      tarifa
  );
}

// Reset do consumo
BLYNK_WRITE(V21) {

  if (param.asInt() == 1) {

    for (int i = 0; i < 4; i++) {
      energiaAcumulada[i] = 0.0;
    }

    Serial.println(
        "[Config] Energia resetada."
    );

    for (int i = 0; i < 4; i++) {

      Blynk.virtualWrite(
          V10 + i,
          0.0
      );
    }

    Blynk.virtualWrite(V30, 0.0);
    Blynk.virtualWrite(V31, 0.0);
  }
}

// Controle dos relés
BLYNK_WRITE(V40) {

  digitalWrite(
      PINO_RELE[0],
      param.asInt() ? HIGH : LOW
  );

  releAtivo[0] = param.asInt();
}

BLYNK_WRITE(V41) {

  digitalWrite(
      PINO_RELE[1],
      param.asInt() ? HIGH : LOW
  );

  releAtivo[1] = param.asInt();
}

BLYNK_WRITE(V42) {

  digitalWrite(
      PINO_RELE[2],
      param.asInt() ? HIGH : LOW
  );

  releAtivo[2] = param.asInt();
}

BLYNK_WRITE(V43) {

  digitalWrite(
      PINO_RELE[3],
      param.asInt() ? HIGH : LOW
  );

  releAtivo[3] = param.asInt();
}

// ====================================================
// SETUP
// ====================================================

void setup() {

  Serial.begin(115200);

  Serial.println(
      "\n=== Monitor Energético ESP32 ==="
  );

  // Configuração dos relés
  for (int i = 0; i < 4; i++) {

    pinMode(PINO_RELE[i], OUTPUT);

    // Relé invertido:
    // HIGH = ligado
    digitalWrite(PINO_RELE[i], HIGH);
  }

  // ADC em 12 bits
  analogReadResolution(12);

  // Conecta no Blynk
  Blynk.begin(
      BLYNK_AUTH_TOKEN,
      WIFI_SSID,
      WIFI_PASS
  );

  // Timer principal
  ultimaAtualizacao = millis();

  timer.setInterval(
      INTERVALO_MS,
      sistema
  );

  Serial.println(
      "Sistema iniciado."
  );
}

// ====================================================
// LOOP PRINCIPAL
// ====================================================

void loop() {

  Blynk.run();

  timer.run();
}