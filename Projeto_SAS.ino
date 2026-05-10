// BIBLIOTECAS UTILIZADAS
#include <Wire.h> // Comunicação I2C
#include <SPI.h> // Comunicação SPI
#include <Adafruit_MPU6050.h> // Biblioteca do MPU6050
#include <Adafruit_Sensor.h> // Biblioteca base de sensores da Adafruit
#include <Adafruit_BMP280.h> // Biblioteca do BMP280
#include <ESP32Servo.h> // Biblioteca para controle de servo no ESP32
#include <SPIMemory.h> // Biblioteca para memória Flash SPI

// OBJETOS DOS SENSORES E ATUADORES
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
Servo motor;


// DEFINIÇÃO DOS PINOS 

// LEDS
#define LED1 12 // LED indicador principal (Azul)
#define LED2 13 // LED indicador de inclinação excessiva (Vermelho)

// MEMÓRIA FLASH SPI
#define SPI_SCK_PIN   18 // Pino de clock SPI
#define SPI_MISO_PIN  19 // Pino MISO (Master In Slave Out)
#define SPI_MOSI_PIN  23 // Pino MOSI (Master Out Slave In)
#define FLASH_CS_PIN  32 // Chip Select da memória Flash
SPIFlash flash(FLASH_CS_PIN); // Objeto da memória flash 

// CONFIGURAÇÕES GERAIS
#define SPI_FLASH_SEC_SIZE 4096 // Tamanho de um setor da memória Flash
const unsigned long LIMITE_MEMORIA = 4194304; // Limite total de memória utilizado

// CONFIGURAÇÕES DA MÉDIA MÓVEL
#define MOVING_AVG_SIZE 20 // Quantidade de amostras usadas na média
float alt_buffer[MOVING_AVG_SIZE]; // Vetor que armazena as últimas leituras
float alt_sum = 0; // Soma acumulada das leituras
float alt_med = 0; // Resultado final da média móvel
int alt_index = 0; // Índice atual do buffer circular
int alt_count = 0; // Quantidade de leituras válidas

// MÁQUINA DE ESTADOS
enum Estado {
  SOLO,
  VOO,
  RECUPERACAO
};

Estado estadoAtual = SOLO; // Estado inicial do sistema

// ESTRUTURA DA TELEMETRIA
struct Dados {
  unsigned long tempo; // Tempo desde o início do programa
  float altitude; // Altitude medida
  float roll; // Inclinação do foguete
  int posServo; // Posição do servo
};

// Variável que armazenará os dados antes da gravação
Dados dado;

// VARIÁVEIS GLOBAIS
uint32_t endereco = 0; // Endereço atual de gravação
float altitudeBase = 0; // Altitude inicial do foguete
float altitudeAtual = 0; // Altitude atual relativa ao solo
float roll = 0; // Inclinação lateral do foguete
int posServo = 90; // Posição inicial do servo

void setup() {

  Serial.begin(115200);// Inicialização da comunicação serial

  // CONFIGURAÇÃO DOS LEDs
  pinMode(LED1, OUTPUT); // Define LED1 como saída
  pinMode(LED2, OUTPUT);  // Define LED2 como saída

  // INICIALIZAÇÃO DO MPU6050
  if (!mpu.begin()) {
    Serial.println("Erro ao encontrar MPU6050");
    while (1);
  }

  // Configurações do MPU6050 
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 iniciado!");

  // INICIALIZAÇÃO DO BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 nao encontrado!");
    while (1);
  }

  altitudeBase = bmp.readAltitude(1013.25); // Medição da altitude inicial do local

  Serial.println("BMP280 iniciado!");

  // INICIALIZAÇÃO DO BUFFER DA MÉDIA MÓVEL
  for (int i = 0; i < MOVING_AVG_SIZE; i++) {
    alt_buffer[i] = 0;
  }

  // CONFIGURAÇÃO DO SERVO MOTOR
  motor.attach(26);
  motor.write(90);

  // CONFIGURAÇÃO DO BARRAMENTO SPI
  SPI.begin(
    SPI_SCK_PIN,
    SPI_MISO_PIN,
    SPI_MOSI_PIN,
    FLASH_CS_PIN
  );

  // INICIALIZAÇÃO DA MEMÓRIA FLASH
  if (!flash.begin()) {
    Serial.println("Erro na memoria flash!");
    while (1);
  }

  Serial.println("Flash iniciada!");

  delay(1000);
}


void loop() {

  // Atualiza altitude continuamente
  altitudeAtual = lerAltitudeRelativa();

  // MÁQUINA DE ESTADOS
  switch (estadoAtual) {
    case SOLO:

      estadoSolo(); // Executa funções do estado SOLO

      // Se altitude passar de 1 metro, foguete entra em voo
      if (altitudeAtual > 1) {
        estadoAtual = VOO;
      }
      break;

    case VOO:

      estadoVoo(); // Executa funções do voo

      // Se altitude cair abaixo de 1 metro, considera recuperação/pouso
      if (altitudeAtual < 1) {
        estadoAtual = RECUPERACAO;
      }
      break;

    case RECUPERACAO:

      estadoRecuperacao(); // Executa funções da recuperação
      break;
  }

  // Delay geral do sistema
  delay(100);
}

// FUNÇÃO DO ESTADO SOLO
void estadoSolo() {

  digitalWrite(LED1, HIGH);  // Liga LED1
  digitalWrite(LED2, LOW);   // Desliga LED2
  Serial.print("SOLO | Altitude: ");
  Serial.println(altitudeAtual);
}

// FUNÇÃO DO ESTADO VOO
void estadoVoo() {

  digitalWrite(LED1, HIGH); // Liga LED1

  // LEITURA DA INCLINAÇÃO
  roll = lerInclinacaoX(); // Calcula inclinação do foguete

  // CONTROLE DO SERVO
  posServo = 90 + roll; // Ajusta posição do servo
  posServo = constrain(posServo, 0, 180); // Garante limites físicos do servo
  motor.write(posServo);  // Move servo
  delay(5);

  // LED DE ALERTA
  // Se inclinação ultrapassar 20° (inclinação crítica)
  if (abs(roll) > 20) {
    digitalWrite(LED2, HIGH);
  }
  else {
    digitalWrite(LED2, LOW);
  }

  salvarTelemetria();
}

// FUNÇÃO DO ESTADO RECUPERAÇÃO
void estadoRecuperacao() {

  digitalWrite(LED1, HIGH);   // Liga LED1
  digitalWrite(LED2, LOW); // Desliga LED2
  motor.write(90); // Servo retorna para 90°
  Serial.print("RECUPERACAO | Altitude: ");
  Serial.println(altitudeAtual);
  delay(100);
}

// LEITURA DA ALTITUDE RELATIVA COM FILTRO DE MÉDIA MÓVEL
float lerAltitudeRelativa() {

  float altitudeAbsoluta =  bmp.readAltitude(1013.25); // leitura da Altitude Absoluta
  float altitudeRelativa =  altitudeAbsoluta - altitudeBase; // Calcula a altitude relativa com base na do solo

  // MÉDIA MÓVEL
  alt_sum -= alt_buffer[alt_index]; // Remove valor mais antigo da soma
  alt_buffer[alt_index] = altitudeRelativa; // Coloca nova leitura no buffer
  alt_sum += altitudeRelativa; // Adiciona nova leitura à soma

  // Avança índice circularmente
  // Quando chega ao final,volta para o início
  alt_index = (alt_index + 1) % MOVING_AVG_SIZE;

  // Conta quantas leituras já existem
  if (alt_count < MOVING_AVG_SIZE) {
    alt_count++;
  }

  alt_med = alt_sum / alt_count; // Calcula média filtrada

  return alt_med;
}

// LEITURA DA INCLINAÇÃO
float lerInclinacaoX() {

  // Estruturas que armazenam dados do sensor
  sensors_event_t a, g, temp;

  // Atualiza leituras do MPU6050
  mpu.getEvent(&a, &g, &temp);

  // ACELERAÇÕES NOS EIXOS
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  // CÁLCULO DO ROLL
  float roll = atan2( -ax, sqrt(ay * ay + az * az));
  roll = roll * 180.0 / M_PI; // Conversão pra graus
  
  return roll;
}

// FUNÇÃO DE TELEMETRIA
void salvarTelemetria() {

  // PREENCHE ESTRUTURA
  dado.tempo = millis();
  dado.altitude = altitudeAtual;
  dado.roll = roll;
  dado.posServo = posServo;

  // VERIFICA LIMITE DA MEMÓRIA
  if (endereco + sizeof(Dados) > LIMITE_MEMORIA) {
    Serial.println("Memoria cheia!");
    return;
  }

  // VERIFICA TROCA DE SETOR
  // Calcula endereço final da gravação
  uint32_t endAddress =  endereco + sizeof(Dados);

  // Verifica se dados ultrapassam setor atual
  if (endAddress > ((endereco / SPI_FLASH_SEC_SIZE) + 1) * SPI_FLASH_SEC_SIZE) {
    endereco = ((endereco / SPI_FLASH_SEC_SIZE) + 1) * SPI_FLASH_SEC_SIZE; // Move para próximo setor
  }

  // APAGA SETOR DA FLASH
  // Flash só pode gravar corretamente em setores apagados
  if (endereco % SPI_FLASH_SEC_SIZE == 0) {
    if (!flash.eraseSector(endereco)) {
      Serial.println("Erro ao apagar setor!");
      return;
    }
  }

  // GRAVAÇÃO DA TELEMETRIA
   if (flash.writeAnything(endereco, dado)) {

    Serial.println("Telemetria gravada!");

    Serial.print("Tempo: ");
    Serial.print(dado.tempo);

    Serial.print(" ms | Altitude: ");
    Serial.print(dado.altitude);

    Serial.print(" m | Roll: ");
    Serial.print(dado.roll);

    Serial.print(" deg | Servo: ");
    Serial.println(dado.posServo);

    endereco += sizeof(Dados); // Avança para próximo endereço livre
  }
  else {
    Serial.println("Falha na gravacao!");
  }
}