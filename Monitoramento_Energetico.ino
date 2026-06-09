/*
=========================================================
  MONITOR ENERGÉTICO IoT - v11 (FINAL)
  Hardware : ESP32 + 2x ACS712-30A + 2x Relé + LEDs
  Plataforma: Blynk + Monitor Serial + Telnet WiFi
  Alimentação sensor: 4.5V via VIN USB
=========================================================

  MAPA DE PINOS:
  ┌─────────────────┬──────┐
  │ ACS712 C1 (OUT) │ G32  │
  │ ACS712 C2 (OUT) │ G34  │
  │ Relé C1         │ G25  │
  │ Relé C2         │ G26  │
  │ LED Verde C1    │ G18  │
  │ LED Vermelho C1 │ G19  │
  │ LED Verde C2    │ G21  │
  │ LED Vermelho C2 │ G22  │
  └─────────────────┴──────┘

  MAPA BLYNK:
  ┌──────────────────────────┬─────┐
  │ Corrente C1              │ V0  │
  │ Corrente C2              │ V1  │
  │ Energia C1               │ V10 │
  │ Energia C2               │ V11 │
  │ Tarifa kWh (entrada)     │ V20 │
  │ Reset energia (botão)    │ V21 │
  │ Potência C1              │ V50 │
  │ Potência C2              │ V51 │
  │ Custo C1                 │ V60 │
  │ Custo C2                 │ V61 │
  │ Energia Total            │ V30 │
  │ Custo Total              │ V31 │
  │ Comando Relé C1          │ V40 │
  │ Comando Relé C2          │ V41 │
  └──────────────────────────┴─────┘

  CALIBRAÇÃO POR FAIXA:
  Para ajustar: SENS_nova = SENS_atual × (ESP / Físico)
  Faixas C1: < 0.20A | 0.20~0.40A | > 0.40A
  Faixas C2: < 0.20A | 0.20~0.40A | > 0.40A

  TELNET (backup do Blynk):
  Conecte na porta 23 do IP do ESP32
  Notebook : telnet <IP>
  Celular  : app "Telnet Client" ou "JuiceSSH"
=========================================================
*/

// ─── BLYNK ────────────────────────────────────────────────────────
#define BLYNK_TEMPLATE_ID   "TMPL2Km9O6sIc"
#define BLYNK_TEMPLATE_NAME "Monitoramento Energético"
#define BLYNK_AUTH_TOKEN    "Q0a9c5m0rv4PpArBdyMJQtmcElu1zjAD"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// ─── REDE ─────────────────────────────────────────────────────────
char ssid[] = "iLukaS";
char pass[] = "998308357@Lucas";

// ─── TELNET (Monitor Serial via WiFi, porta 23) ───────────────────
WiFiServer telnetServer(23);
WiFiClient telnetClient;

// ─── PINOS ────────────────────────────────────────────────────────
#define ACS_C1          32
#define ACS_C2          34
#define RELE_C1         25
#define RELE_C2         26
#define LED_VERDE_C1    18
#define LED_VERMELHO_C1 19
#define LED_VERDE_C2    21
#define LED_VERMELHO_C2 22

// ─── CONSTANTES FÍSICAS ───────────────────────────────────────────
const float TENSAO_REDE = 127.0;  // Tensão da rede elétrica (V)
float tarifaKwh         = 0.95;   // R$/kWh — ajustável pelo Blynk V20
float limiteC1          = 10.0;    // Limite de sobrecarga C1 (A)
float limiteC2          = 10.0;    // Limite de sobrecarga C2 (A)

// ─── SENSIBILIDADE INDIVIDUAL POR CANAL ──────────────────────────
// Para recalibrar: SENS_nova = SENS_atual × (leitura_ESP / leitura_fisica)
const float SENSIBILIDADE_C1 = 0.114;
const float SENSIBILIDADE_C2 = 0.123;

// ─── ADC ──────────────────────────────────────────────────────────
const float VOLTS_PER_STEP = 3.3 / 4095.0;
const float ZONA_MORTA     = 0.10;

// ─── OFFSETS (calibrados automaticamente no boot) ─────────────────
float ADC_ZERO_C1 = 2964.24;
float ADC_ZERO_C2 = 2954.64;

bool calibrando = false;

// ─── VARIÁVEIS DE ENERGIA ─────────────────────────────────────────
float correnteC1 = 0, correnteC2 = 0;
float potenciaC1 = 0, potenciaC2 = 0;
float energiaC1  = 0, energiaC2  = 0;
float custoC1    = 0, custoC2    = 0;
float correnteFiltradaC1 = 0;
float correnteFiltradaC2 = 0;
unsigned long ultimoTempo = 0;
// Controle de estabilização dos relés
unsigned long ultimoAcionamentoReleC1 = 0;
unsigned long ultimoAcionamentoReleC2 = 0;
const unsigned long TEMPO_ESTABILIZACAO_RELE = 3000; // 3 segundos
BlynkTimer timer;

// ─── TABELA DE CALIBRAÇÃO POR FAIXA ──────────────────────────────
// Ajuste estes fatores conforme medições com amperímetro físico.
// Fórmula: fator = corrente_fisica / corrente_ESP_nessa_faixa

// C1 — medições base:
// Físico 0.23A → ESP 0.25A → fator = 0.92
// Físico 0.32A → ESP 0.32A → fator = 1.00
float C1_FATOR_BAIXO = 0.88;  // faixa < 0.20A
float C1_FATOR_MEDIO = 0.95;  // faixa 0.20A ~ 0.40A
float C1_FATOR_ALTO  = 1.00;  // faixa > 0.40A

// C2 — medições base:
// Físico 0.11A → ESP 0.19A → fator = 0.58
// Físico 0.26A → ESP 0.23A → fator = 1.13
float C2_FATOR_BAIXO = 0.60;  // faixa < 0.20A
float C2_FATOR_MEDIO = 1.05;  // faixa 0.20A ~ 0.40A
float C2_FATOR_ALTO  = 2.00;  // faixa > 0.40A

// ─── CORREÇÃO POR FAIXA — C1 ─────────────────────────────────────
float corrigirC1(float corrente) {
  if (corrente <= 0.0)  return 0.0;
  if (corrente < 0.20)  return corrente * C1_FATOR_BAIXO;
  if (corrente < 0.40)  return corrente * C1_FATOR_MEDIO;
  return corrente * C1_FATOR_ALTO;
}

// ─── CORREÇÃO POR FAIXA — C2 ─────────────────────────────────────
float corrigirC2(float corrente) {
  if (corrente <= 0.0)  return 0.0;
  if (corrente < 0.20)  return corrente * C2_FATOR_BAIXO;
  if (corrente < 0.40)  return corrente * C2_FATOR_MEDIO;
  return corrente * C2_FATOR_ALTO;
}

// ─── CALIBRAÇÃO DINÂMICA DO OFFSET ───────────────────────────────
void calibrarOffset() {
  digitalWrite(RELE_C1,         HIGH);
  digitalWrite(RELE_C2,         HIGH);
  digitalWrite(LED_VERDE_C1,    HIGH);
  digitalWrite(LED_VERMELHO_C1, LOW);
  digitalWrite(LED_VERDE_C2,    HIGH);
  digitalWrite(LED_VERMELHO_C2, LOW);

  Serial.println("\n>>> CALIBRANDO — Relés em ON, sem carga! <<<");
  delay(1000);

  for (int i = 0; i < 200; i++) {
    analogRead(ACS_C1);
    analogRead(ACS_C2);
    delayMicroseconds(500);
  }

  const int RODADAS  = 3;
  const int AMOSTRAS = 1000;
  float somaZeroC1   = 0;
  float somaZeroC2   = 0;

  for (int r = 0; r < RODADAS; r++) {
    long acumulaC1 = 0, acumulaC2 = 0;
    for (int i = 0; i < AMOSTRAS; i++) {
      acumulaC1 += analogRead(ACS_C1);
      acumulaC2 += analogRead(ACS_C2);
      delayMicroseconds(200);
    }
    float mediaC1 = acumulaC1 / (float)AMOSTRAS;
    float mediaC2 = acumulaC2 / (float)AMOSTRAS;
    somaZeroC1 += mediaC1;
    somaZeroC2 += mediaC2;
    Serial.print("  Rodada "); Serial.print(r + 1);
    Serial.print(" → C1: "); Serial.print(mediaC1, 1);
    Serial.print(" | C2: "); Serial.println(mediaC2, 1);
    delay(100);
  }

  float novoC1 = somaZeroC1 / RODADAS;
  float novoC2 = somaZeroC2 / RODADAS;

  if (abs(novoC1 - 2795) < 300) ADC_ZERO_C1 = novoC1;
  else Serial.println("  ⚠️  C1: offset suspeito, mantendo 2795");

  if (abs(novoC2 - 2795) < 300) ADC_ZERO_C2 = novoC2;
  else Serial.println("  ⚠️  C2: offset suspeito, mantendo 2795");

  Serial.print("\n>>> Offset FINAL C1: "); Serial.println(ADC_ZERO_C1, 2);
  Serial.print(">>> Offset FINAL C2: "); Serial.println(ADC_ZERO_C2, 2);
  Serial.println(">>> Calibração concluída! <<<\n");

  calibrando = false;
}

// ─── LEITURA RMS ─────────────────────────────────────────────────
float lerCorrenteRMS(int pino, float offset, float sensibilidade) {
  analogRead(pino);
  delay(5);

  unsigned long tempoInicio = millis();
  float somaQuadrados = 0;
  long  n             = 0;

  while ((millis() - tempoInicio) < 120) {
    float diferenca  = (float)analogRead(pino) - offset;
    somaQuadrados   += diferenca * diferenca;
    n++;
    delayMicroseconds(100);
  }

  float rmsADC    = sqrt(somaQuadrados / n);
  float tensaoRMS = rmsADC * VOLTS_PER_STEP;
  float corrente  = tensaoRMS / sensibilidade;

  return (corrente < ZONA_MORTA) ? 0.0 : corrente;
}

// ─── PROTEÇÃO CONTRA SOBRECARGA ───────────────────────────────────
void verificarProtecao() {
  if (correnteC1 > limiteC1) {
    digitalWrite(RELE_C1,         LOW);
    digitalWrite(LED_VERDE_C1,    LOW);
    digitalWrite(LED_VERMELHO_C1, HIGH);
    Blynk.logEvent("sobrecarga", "Sobrecarga Circuito 1");
    Serial.println("⚠️  SOBRECARGA C1 — Relé desligado!");
  }
  if (correnteC2 > limiteC2) {
    digitalWrite(RELE_C2,         LOW);
    digitalWrite(LED_VERDE_C2,    LOW);
    digitalWrite(LED_VERMELHO_C2, HIGH);
    Blynk.logEvent("sobrecarga", "Sobrecarga Circuito 2");
    Serial.println("⚠️  SOBRECARGA C2 — Relé desligado!");
  }
}

// ─── CALLBACKS BLYNK ──────────────────────────────────────────────
BLYNK_WRITE(V20) {
  if (calibrando) return;
  tarifaKwh = param.asFloat();
}

BLYNK_WRITE(V40) {
  if (calibrando) return;
  int estado = param.asInt();
  digitalWrite(RELE_C1,         estado ? HIGH : LOW);
  digitalWrite(LED_VERDE_C1,    estado ? HIGH : LOW);
  digitalWrite(LED_VERMELHO_C1, estado ? LOW  : HIGH);

    ultimoAcionamentoReleC1 = millis();
}

BLYNK_WRITE(V41) {
  if (calibrando) return;
  int estado = param.asInt();
  digitalWrite(RELE_C2,         estado ? HIGH : LOW);
  digitalWrite(LED_VERDE_C2,    estado ? HIGH : LOW);
  digitalWrite(LED_VERMELHO_C2, estado ? LOW  : HIGH);

    ultimoAcionamentoReleC2 = millis();
}

BLYNK_WRITE(V21) {
  if (calibrando) return;
  if (param.asInt() == 1) {
    energiaC1 = 0;
    energiaC2 = 0;
    Serial.println(">>> Energia zerada pelo usuário <<<");
  }
}

// ─── LÓGICA PRINCIPAL (executa a cada 2s) ────────────────────────
void sistema() {
  if (calibrando) return;

  unsigned long agora = millis();
  float deltaHoras    = (agora - ultimoTempo) / 3600000.0;
  ultimoTempo         = agora;

    // 1. Leitura RMS com sensibilidade individual
  correnteC1 = lerCorrenteRMS(ACS_C1, ADC_ZERO_C1, SENSIBILIDADE_C1);
  delay(5);
  correnteC2 = lerCorrenteRMS(ACS_C2, ADC_ZERO_C2, SENSIBILIDADE_C2);

  // 2. Filtro EMA
  correnteFiltradaC1 = (correnteFiltradaC1 * 0.80) + (correnteC1 * 0.20);
  correnteFiltradaC2 = (correnteFiltradaC2 * 0.80) + (correnteC2 * 0.20);

  correnteC1 = correnteFiltradaC1;
  correnteC2 = correnteFiltradaC2;

  // 3. Correção por faixa
  correnteC1 = corrigirC1(correnteC1);
  correnteC2 = corrigirC2(correnteC2);

  // 4. Zona morta
const float LIMIAR_RUIDO = 0.08;

if (correnteC1 < LIMIAR_RUIDO)
  correnteC1 = 0;

if (correnteC2 < LIMIAR_RUIDO)
  correnteC2 = 0;

// 5. Bloqueio por estado do relé
if (digitalRead(RELE_C1) == LOW) {
  correnteC1 = 0;
  correnteFiltradaC1 = 0;
}

if (digitalRead(RELE_C2) == LOW) {
  correnteC2 = 0;
  correnteFiltradaC2 = 0;
}

// 5.1 Ignora leituras logo após acionar o relé
if ((millis() - ultimoAcionamentoReleC1) < TEMPO_ESTABILIZACAO_RELE) {
  correnteC1 = 0;
  correnteFiltradaC1 = 0;
}

if ((millis() - ultimoAcionamentoReleC2) < TEMPO_ESTABILIZACAO_RELE) {
  correnteC2 = 0;
  correnteFiltradaC2 = 0;
}

  // 6. Potência, energia e custo
  potenciaC1 = (correnteC1 > 0) ? correnteC1 * TENSAO_REDE : 0.0;
  potenciaC2 = (correnteC2 > 0) ? correnteC2 * TENSAO_REDE : 0.0;

  energiaC1 += (potenciaC1 / 1000.0) * deltaHoras;
  energiaC2 += (potenciaC2 / 1000.0) * deltaHoras;

  custoC1 = energiaC1 * tarifaKwh;
  custoC2 = energiaC2 * tarifaKwh;

  // 7. Proteção
  verificarProtecao();

  // 8. Envio Blynk
  Blynk.virtualWrite(V0,  correnteC1);
  Blynk.virtualWrite(V1,  correnteC2);
  Blynk.virtualWrite(V50, potenciaC1);
  Blynk.virtualWrite(V51, potenciaC2);
  Blynk.virtualWrite(V10, energiaC1);
  Blynk.virtualWrite(V11, energiaC2);
  Blynk.virtualWrite(V60, custoC1);
  Blynk.virtualWrite(V61, custoC2);
  Blynk.virtualWrite(V30, energiaC1 + energiaC2);
  Blynk.virtualWrite(V31, custoC1   + custoC2);
  
  // 9. Painel completo via Telnet (backup do Blynk)
  if (telnetClient && telnetClient.connected()) {
    telnetClient.println("\n==========================================");
    telnetClient.println("      MONITOR ENERGÉTICO IoT - ESP32      ");
    telnetClient.println("==========================================");

    telnetClient.println("--- CIRCUITO 1 ---");
    telnetClient.print("  Corrente : "); telnetClient.print(correnteC1, 2);   telnetClient.println(" A");
    telnetClient.print("  Potência : "); telnetClient.print(potenciaC1, 1);   telnetClient.println(" W");
    telnetClient.print("  Energia  : "); telnetClient.print(energiaC1,  4);   telnetClient.println(" kWh");
    telnetClient.print("  Custo    : R$ ");                                    telnetClient.println(custoC1, 4);
    telnetClient.print("  Status   : "); telnetClient.println(digitalRead(RELE_C1) ? "LIGADO" : "DESLIGADO");

    telnetClient.println("--- CIRCUITO 2 ---");
    telnetClient.print("  Corrente : "); telnetClient.print(correnteC2, 2);   telnetClient.println(" A");
    telnetClient.print("  Potência : "); telnetClient.print(potenciaC2, 1);   telnetClient.println(" W");
    telnetClient.print("  Energia  : "); telnetClient.print(energiaC2,  4);   telnetClient.println(" kWh");
    telnetClient.print("  Custo    : R$ ");                                    telnetClient.println(custoC2, 4);
    telnetClient.print("  Status   : "); telnetClient.println(digitalRead(RELE_C2) ? "LIGADO" : "DESLIGADO");

    telnetClient.println("--- TOTAL GERAL ---");
    telnetClient.print("  Corrente : "); telnetClient.print(correnteC1 + correnteC2, 2); telnetClient.println(" A");
    telnetClient.print("  Potência : "); telnetClient.print(potenciaC1 + potenciaC2, 1); telnetClient.println(" W");
    telnetClient.print("  Energia  : "); telnetClient.print(energiaC1  + energiaC2,  4); telnetClient.println(" kWh");
    telnetClient.print("  Custo    : R$ ");                                               telnetClient.println(custoC1 + custoC2, 4);
    telnetClient.print("  Tarifa   : R$ "); telnetClient.print(tarifaKwh, 2);            telnetClient.println("/kWh");
    telnetClient.println("==========================================");
  }

  // 8. Monitor Serial
  Serial.println("==========================================");
  Serial.print("C1    | "); Serial.print(correnteC1, 2); Serial.print(" A | ");
  Serial.print(potenciaC1, 1); Serial.print(" W | ");
  Serial.print(energiaC1, 4); Serial.print(" kWh | R$ ");
  Serial.println(custoC1, 4);
  Serial.print("C2    | "); Serial.print(correnteC2, 2); Serial.print(" A | ");
  Serial.print(potenciaC2, 1); Serial.print(" W | ");
  Serial.print(energiaC2, 4); Serial.print(" kWh | R$ ");
  Serial.println(custoC2, 4);
  Serial.println("------------------------------------------");
  Serial.print("TOTAL | ");
  Serial.print(correnteC1 + correnteC2, 2); Serial.print(" A | ");
  Serial.print(potenciaC1 + potenciaC2, 1); Serial.print(" W | ");
  Serial.print(energiaC1  + energiaC2,  4); Serial.print(" kWh | R$ ");
  Serial.println(custoC1  + custoC2,    4);
  Serial.println("==========================================");
}

// ─── SETUP ───────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(RELE_C1,         OUTPUT);
  pinMode(RELE_C2,         OUTPUT);
  pinMode(LED_VERDE_C1,    OUTPUT);
  pinMode(LED_VERMELHO_C1, OUTPUT);
  pinMode(LED_VERDE_C2,    OUTPUT);
  pinMode(LED_VERMELHO_C2, OUTPUT);

  // Estado inicial: relés desligados, LEDs vermelhos (indica boot)
  digitalWrite(RELE_C1,         LOW);
  digitalWrite(RELE_C2,         LOW);
  digitalWrite(LED_VERDE_C1,    LOW);
  digitalWrite(LED_VERMELHO_C1, HIGH);
  digitalWrite(LED_VERDE_C2,    LOW);
  digitalWrite(LED_VERMELHO_C2, HIGH);

  analogReadResolution(12);
  analogSetPinAttenuation(ACS_C1, ADC_11db);
  analogSetPinAttenuation(ACS_C2, ADC_11db);

  // Calibração do offset (sem carga nos circuitos)
  delay(500);
  calibrarOffset();

  // Conexão WiFi
  WiFi.begin(ssid, pass);
  Serial.print("Conectando ao WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP do ESP32: "); Serial.println(WiFi.localIP());

  // Blynk
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect();

  // Telnet
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  Serial.print("Telnet disponível em: ");
  Serial.print(WiFi.localIP()); Serial.println(":23");

  ultimoTempo = millis();
  timer.setInterval(2000L, sistema);

}

// ─── LOOP ────────────────────────────────────────────────────────
void loop() {
  // Gerencia conexão Telnet
  if (telnetServer.hasClient()) {
    if (telnetClient && telnetClient.connected()) {
      WiFiClient novoCliente = telnetServer.available();
      novoCliente.stop();
    } else {
      telnetClient = telnetServer.available();
      Serial.println("Cliente Telnet conectado");
      telnetClient.println("==========================================");
      telnetClient.println("      MONITOR ENERGÉTICO IoT - ESP32      ");
      telnetClient.println("      Conexão estabelecida com sucesso     ");
      telnetClient.println("==========================================");
    }
  }

  if (telnetClient && !telnetClient.connected()) {
    telnetClient.stop();
    Serial.println("Cliente Telnet desconectado");
  }

  Blynk.run();
  timer.run();
}
