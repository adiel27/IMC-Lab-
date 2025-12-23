#include <WiFi.h>
#include <PubSubClient.h>

// WiFi
const char* ssid = "Redmi Note 11";
const char* password = "mywifi123";

// MQTT broker
const char* mqtt_server = "broker.hivemq.com";

WiFiClient espClient;
PubSubClient client(espClient);

// Pin motor
const int motor1Pin1 = 27;
const int motor1Pin2 = 26;
const int enable1Pin = 12;

// Pin sensor (encoder / hall effect)
const int sensorPin = 34;
volatile uint32_t pulseCount = 0;

// Encoder config
const uint16_t pulsesPerRev = 1;
const uint32_t rpmIntervalMs = 1000;

// Target RPM per mode
const int targetLowRPM = 200;
const int targetMediumRPM = 300;
const int targetHighRPM = 600;

// State kontrol
enum SpeedMode { MODE_OFF, MODE_LOW, MODE_MEDIUM, MODE_HIGH };
volatile SpeedMode mode = MODE_OFF;
int targetRPM = 0;

// LED indikator
const int ledPin = 2;

// PWM setup
const int pwmChannel = 0;
const int freq = 20000;     // frekuensi PWM 20kHz
const int resolution = 8;   // 8-bit (0–255)

// Waktu
unsigned long lastRpmTime = 0;

// Interrupt untuk hitung pulsa
void IRAM_ATTR countPulse() {
  pulseCount++;
}

// Koneksi WiFi
void setup_wifi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // LED berkedip cepat tanda WiFi tersambung
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
}

// Callback MQTT
void callback(char* topic, byte* message, unsigned int length) {
  String msg;
  msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)message[i];
  msg.trim();
  Serial.print("Pesan: "); Serial.println(msg);

  if (msg == "off") {
    mode = MODE_OFF;
    targetRPM = 0;
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel, 0);
    Serial.println("Motor OFF");
  } else if (msg == "LOW") {
    mode = MODE_LOW;
    targetRPM = targetLowRPM;
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    Serial.println("Motor LOW target 200 RPM");
  } else if (msg == "MEDIUM") {
    mode = MODE_MEDIUM;
    targetRPM = targetMediumRPM;
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    Serial.println("Motor MEDIUM target 2000 RPM");
  } else if (msg == "HIGH" || msg == "on") {
    mode = MODE_HIGH;
    targetRPM = targetHighRPM;
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    Serial.println("Motor HIGH target 8000 RPM");
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32MotorClient")) {
      Serial.println("connected");
      client.subscribe("esp32/motor/control");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retry in 5s...");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(ledPin, OUTPUT);

  pinMode(sensorPin, INPUT_PULLUP);
  attachInterrupt(sensorPin, countPulse, RISING);

  // Setup PWM
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(enable1Pin, pwmChannel);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  mode = MODE_OFF;
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  ledcWrite(pwmChannel, 0);

  lastRpmTime = millis();
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  unsigned long now = millis();

  // Hitung RPM tiap interval
  if (now - lastRpmTime >= rpmIntervalMs) {
    noInterrupts();
    uint32_t pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float revs = (float)pulses / (float)pulsesPerRev;
    float rpm = revs * (60000.0 / rpmIntervalMs);

    Serial.print("RPM: "); Serial.println(rpm);

    char buf[16];
    dtostrf(rpm, 0, 1, buf);
    client.publish("esp32/motor/speed", buf);

    // kontrol PWM adaptif
    if (mode != MODE_OFF) {
      int error = targetRPM - rpm;
      int duty = constrain(error, 0, 255); // duty cycle sesuai selisih RPM
      ledcWrite(pwmChannel, duty);
    } else {
      ledcWrite(pwmChannel, 0);
    }

    lastRpmTime = now;
  }

  // LED indikator idle
  if (WiFi.status() == WL_CONNECTED && mode == MODE_OFF) {
    digitalWrite(ledPin, HIGH); // menyala stabil saat idle
  } else if (WiFi.status() == WL_CONNECTED && mode != MODE_OFF) {
    // motor aktif → LED kedip cepat
    if (now % 400 < 200) digitalWrite(ledPin, HIGH);
    else digitalWrite(ledPin, LOW);
  } else {
    // belum connect WiFi → kedip pelan
    if (now % 1000 < 500) digitalWrite(ledPin, HIGH);
    else digitalWrite(ledPin, LOW);
  }
}
