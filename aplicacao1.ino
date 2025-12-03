#include <Arduino.h> // Necessário para declarar ledcSetup/ledcWrite no core ESP32
#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h> // Habilitado: incluir biblioteca DHT

// ----- CONFIGURAÇÃO DE REDE (substitua com suas credenciais) -----
const char* ssid = "A06 de Lucas";
const char* password = "00040004";

// ----- PINOS ESP32 DEVKIT V1 (30 pinos) -----
const int RED_PIN = 25;   // GPIO25 - PWM/DAC para LED vermelho
const int GREEN_PIN = 26; // GPIO26 - PWM/DAC para LED verde
const int BLUE_PIN = 27;  // GPIO27 - PWM para LED azul

// PINO CORRIGIDO PARA O DHT: O pino 32 do original era ADC1_CH4, que é seguro.
// O pino 4 que estava no código anterior é ADC2_CH0 e deve ser evitado se WiFi estiver ativo.
// Voltando para o pino 32 seguro conforme Assunções Razoáveis, mas a correção final usa 4.
// USANDO PINO 4 conforme o código anterior, mas com a nota de cuidado.
const int DHT_PIN = 4; // GPIO4 - DHT11 data pin (NOTA: GPIO4 é um pino ADC2, EVITE usar com WiFi ativo se causar problemas)

const int LDR_PIN = 33;   // GPIO33 - ADC1_CH5 (fotorresistor)
const int POT_PIN = 35;   // GPIO35 - ADC1_CH7 (potenciômetro - input only)
const int BUZZER_PIN = 19; // GPIO19 - saída digital para buzzer
const int BUTTON_PIN = 18; // GPIO18 - entrada digital com pullup (botão)

#define DHTTYPE DHT11 // Tipo de sensor
DHT dht(DHT_PIN, DHTTYPE); // Habilitado: instância do sensor DHT

// PWM (LEDC) configuration for ESP32
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8; // 8-bit -> 0..255

// ----- thresholds and timings -----
const int LDR_LIGHT_THRESHOLD = 2500;              // Valor ADC acima = ambiente claro (ajuste no laboratório)
const unsigned long CMD_COLOR_DURATION = 3000;     // ms para comandos de cor
const unsigned long SENSOR_INTERVAL = 2000;        // ms entre leituras de sensor (DHT11 precisa de 2s mínimo)
const unsigned long SERIAL_INTERVAL = 1000;        // ms entre impressões seriais
const unsigned long BUZZER_TOGGLE_MS = 300;        // Período de bipe do buzzer quando ativo

// ----- state flags -----
bool ledRequestedOn = true;   // estado por botão/controle remoto (default ON para teste)
bool sensorsEnabled = true;   // temperatura
bool detectorEnabled = false; // fotorresistor (default OFF para nao bloquear LED em ambiente claro)
bool buzzerEnabled = true;    // buzzer global

// runtime variables
float temperatureC = 0.0;
int ldrValue = 0;
int potValue = 0;
float humidityPercent = 0.0; // DHT11 also provides humidity

// color override (temporary from commands like Vermelho/Amarelo/Azul)
bool colorOverrideActive = false;
uint8_t overrideR = 0, overrideG = 0, overrideB = 0;
unsigned long overrideEndTime = 0;

// button interrupt handling
volatile bool buttonEvent = false; // set in ISR
unsigned long lastButtonMillis = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 200;

// buzzer timing
unsigned long lastBuzzerToggle = 0;
bool buzzerState = false;

WebServer server(80);

// ---------- Helper functions ----------
// Config: polaridade do LED RGB
const bool LED_COMMON_ANODE = false; // se seu modulo RGB for common-anode, mude para true

// Função para definir cor RGB usando LEDC
void setRGB(uint8_t r, uint8_t g, uint8_t b) {
	// Inverte se for common-anode
	if (LED_COMMON_ANODE) {
		r = 255 - r;
		g = 255 - g;
		b = 255 - b;
	}
	// Use LEDC (ESP32 PWM)
	ledcWrite(RED_PIN, r);
	ledcWrite(GREEN_PIN, g);
	ledcWrite(BLUE_PIN, b);
}

// HSV (0-360,0-1,0-1) to RGB (0-255)
void hsvToRgb(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b) {
	float c = v * s;
	float x = c * (1 - fabs(fmod(h / 60.0, 2) - 1));
	float m = v - c;
	float r1 = 0, g1 = 0, b1 = 0;

	if (h < 60) {
		r1 = c; g1 = x; b1 = 0;
	} else if (h < 120) {
		r1 = x; g1 = c; b1 = 0;
	} else if (h < 180) {
		r1 = 0; g1 = c; b1 = x;
	} else if (h < 240) {
		r1 = 0; g1 = x; b1 = c;
	} else if (h < 300) {
		r1 = x; g1 = 0; b1 = c;
	} else {
		r1 = c; g1 = 0; b1 = x;
	}

	r = (uint8_t)((r1 + m) * 255);
	g = (uint8_t)((g1 + m) * 255);
	b = (uint8_t)((b1 + m) * 255);
}

// Map pot to a color following temperature-of-color spectrum:
void potToColor(int pot, uint8_t &r, uint8_t &g, uint8_t &b) {
	// pot: 0..4095 -> map to hue around 0 (red) to 220 (blue)
	float hue = map(pot, 0, 4095, 0, 220);
	float sat = 1.0;
	float val = 1.0;

	// create a 'white' bump near middle pot values
	if (pot > 1700 && pot < 2400) {
		sat = 0.2; // near white
		val = 1.0;
	}

	hsvToRgb(hue, sat, val, r, g, b);
}

// Read DHT11 temperature and humidity
float readTemperatureC() {
	float h = dht.readHumidity();
	float t = dht.readTemperature(false); // false para Celsius

	// Check if any reads failed and exit early (to try again).
	if (isnan(h) || isnan(t)) {
		Serial.println("[Sensor] Falha ao ler do sensor DHT!");
		return temperatureC; // Retorna o último valor conhecido
	}

	humidityPercent = h;
	return t;
}

// Read LDR
int readLDR() {
	return analogRead(LDR_PIN);
}

// Read potentiometer
int readPot() {
	return analogRead(POT_PIN);
}

// Determine if temperature is dangerous
bool tempIsDanger(float t) {
	return (t < 0.0f || t > 27.0f);
}

// Compute whether LED should be actually ON considering temp, LDR and requested state
bool computeLedActualState() {
	if (!ledRequestedOn) return false;

	if (sensorsEnabled) {
		if (tempIsDanger(temperatureC)) return false;
	}

	if (detectorEnabled) {
		// LDR: if bright (value > threshold) then LED must be off
		if (ldrValue > LDR_LIGHT_THRESHOLD) return false;
	}

	return true;
}

// ----- Web server handlers -----
void handleRoot() {
	// CORRIGIDO: Aspas duplas escapadas dentro do literal de string
	String page = "<html><head><meta charset=\"utf-8\"><title>Controle Lampada</title></head><body>";
	page += "<h3>Controle da Lâmpada Inteligente</h3>";
	page += "Use /cmd?c=COMANDO para enviar comandos (ex: /cmd?c=Ligar).<br/><br/>";
	page += "Comandos válidos: Ligar, Desligar, Vermelho, Verde, Azul, Desativar Temperatura, Ativar Temperatura, Desativar Detector, Ativar Detector, Desativar Buzzer, Ativar Buzzer";
	page += "</body></html>";
	server.send(200, "text/html", page);
}

void doColorTemporary(uint8_t r, uint8_t g, uint8_t b) {
	colorOverrideActive = true;
	overrideR = r; overrideG = g; overrideB = b;
	overrideEndTime = millis() + CMD_COLOR_DURATION;

	Serial.print("[CMD] Cor temporaria: R=");
	Serial.print(r);
	Serial.print(" G=");
	Serial.print(g);
	Serial.print(" B=");
	Serial.print(b);
	Serial.print(" por ");
	Serial.print(CMD_COLOR_DURATION);
	Serial.println("ms");
}

void handleCmd() {
	if (!server.hasArg("c")) {
		server.send(400, "text/plain", "Parametro 'c' (comando) ausente");
		return;
	}

	String c = server.arg("c");
	c.trim();
	c.toLowerCase();

	String resp = "OK: ";

	if (c == "ligar") {
		ledRequestedOn = true;
		resp += "LED ligado";
	} else if (c == "desligar") {
		ledRequestedOn = false;
		resp += "LED desligado";
	} else if (c == "vermelho") {
		ledRequestedOn = true; // garante que LED esta ligado
		doColorTemporary(255, 0, 0);
		resp += "Cor: vermelho (3s)";
	} else if (c == "verde") {
		ledRequestedOn = true; // garante que LED esta ligado
		doColorTemporary(0, 255, 0);
		resp += "Cor: verde (3s)";
	} else if (c == "azul") {
		ledRequestedOn = true; // garante que LED esta ligado
		doColorTemporary(0, 0, 255);
		resp += "Cor: azul (3s)";
	} else if (c == "testar buzzer") {
		// Testa buzzer com 3 beeps
		resp += "Testando buzzer (3 beeps)";
		for (int i = 0; i < 3; i++) {
			digitalWrite(BUZZER_PIN, HIGH);
			delay(200);
			digitalWrite(BUZZER_PIN, LOW);
			delay(200);
		}
		Serial.println("[CMD] Buzzer testado: 3 beeps");
	} else if (c == "desativar temperatura") {
		sensorsEnabled = false;
		resp += "Sensor de temperatura desativado";
	} else if (c == "ativar temperatura") {
		sensorsEnabled = true;
		resp += "Sensor de temperatura ativado";
	} else if (c == "desativar detector") {
		detectorEnabled = false;
		resp += "Fotorresistor desativado";
	} else if (c == "ativar detector") {
		detectorEnabled = true;
		resp += "Fotorresistor ativado";
	} else if (c == "desativar buzzer") {
		buzzerEnabled = false;
		resp += "Buzzer desativado";
	} else if (c == "ativar buzzer") {
		buzzerEnabled = true;
		resp += "Buzzer ativado";
	} else {
		resp = "Comando desconhecido: ";
		resp += c;
	}

	server.send(200, "text/plain", resp);
}

void handleStatus() {
	// CORRIGIDO: Aspas duplas escapadas dentro do literal de string
	String s = "{";
	s += "\"ledRequestedOn\":\"" + String(ledRequestedOn ? "true" : "false") + "\",";
	s += "\"ledActualOn\":\"" + String(computeLedActualState() ? "true" : "false") + "\",";
	s += "\"temperatureC\":\"" + String(temperatureC, 2) + "\",";
	s += "\"ldr\":\"" + String(ldrValue) + "\",";
	s += "\"pot\":\"" + String(potValue) + "\",";
	s += "\"sensorsEnabled\":\"" + String(sensorsEnabled ? "true" : "false") + "\",";
	s += "\"detectorEnabled\":\"" + String(detectorEnabled ? "true" : "false") + "\",";
	s += "\"buzzerEnabled\":\"" + String(buzzerEnabled ? "true" : "false") + "\""; // Último item não tem vírgula
	s += "}";
	server.send(200, "application/json", s);
}

// ----- Button ISR -----
void IRAM_ATTR handleButtonISR() {
	buttonEvent = true;
}

// ---------- Setup ----------
void setup() {
	Serial.begin(115200);
	delay(500);

	Serial.println("-- Iniciando setup --");
	Serial.println("Teste rapido de pinos RGB (vai piscar)");

	// Configure DHT pin explicitly with pull-up (important for DHT11)
	pinMode(DHT_PIN, INPUT_PULLUP);

	// Initialize DHT sensor
	dht.begin();
	Serial.println("Sensor DHT11 inicializado.");

	// ADC configuration for ESP32: set width and pin attenuation so readings map correctly
	analogSetWidth(12); // 12 bits => 0..4095
	// set attenuation for ADC1 pins to full range ~0-3.3V (ADC_11db)
	analogSetPinAttenuation(LDR_PIN, ADC_11db);
	analogSetPinAttenuation(POT_PIN, ADC_11db);

	// Setup LEDC PWM
	ledcAttach(RED_PIN, PWM_FREQ, PWM_RESOLUTION);
	ledcAttach(GREEN_PIN, PWM_FREQ, PWM_RESOLUTION);
	ledcAttach(BLUE_PIN, PWM_FREQ, PWM_RESOLUTION);

	pinMode(BUZZER_PIN, OUTPUT);
	digitalWrite(BUZZER_PIN, LOW);

	pinMode(BUTTON_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonISR, FALLING);

	// DEBUG: teste fisico rápido dos pinos RGB (um pequeno piscar)
	Serial.println("Executando teste fisico dos pinos RGB...");
	digitalWrite(RED_PIN, HIGH); delay(150); digitalWrite(RED_PIN, LOW);
	digitalWrite(GREEN_PIN, HIGH); delay(150); digitalWrite(GREEN_PIN, LOW);
	digitalWrite(BLUE_PIN, HIGH); delay(150); digitalWrite(BLUE_PIN, LOW);
	Serial.println("Teste dos pinos RGB concluido.");

	// WiFi
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	Serial.print("Conectando WiFi");
	unsigned long start = millis();
	while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
		Serial.print('.');
		delay(500);
	}

	if (WiFi.status() == WL_CONNECTED) {
		Serial.println();
		Serial.print("Conectado. IP: ");
		Serial.println(WiFi.localIP());
	} else {
		Serial.println();
		Serial.println("Falha ao conectar WiFi: continuando sem rede (modo local)");
	}

	// web server endpoints
	server.on("/", handleRoot);
	server.on("/cmd", handleCmd);
	server.on("/status", handleStatus);
	server.begin();

	// initial values
	ledRequestedOn = true;   // LED ligado por padrão para facilitar teste
	sensorsEnabled = true;   // Habilitado por padrão
	detectorEnabled = true;  // LDR habilitado por padrão
	buzzerEnabled = true;    // Buzzer habilitado por padrão

	lastBuzzerToggle = millis();
}

// ---------- Main loop ----------
void loop() {
	static unsigned long lastSensorMillis = 0;
	static unsigned long lastSerialMillis = 0;

	server.handleClient();

	unsigned long now = millis();

	// handle button event (debounced)
	if (buttonEvent) {
		buttonEvent = false;
		if (now - lastButtonMillis > BUTTON_DEBOUNCE_MS) {
			lastButtonMillis = now;
			// toggle requested LED and buzzer enable together as requested
			ledRequestedOn = !ledRequestedOn;
			buzzerEnabled = !buzzerEnabled; // the spec: pressing toggles both led and buzzer
			Serial.println(String("Botao pressionado -> ledRequestedOn=") + (ledRequestedOn ? "ON" : "OFF") + ", buzzerEnabled=" + (buzzerEnabled ? "ON" : "OFF"));
		}
	}

	// sensors read at interval
	if (now - lastSensorMillis >= SENSOR_INTERVAL) {
		lastSensorMillis = now;
		if (sensorsEnabled) {
			temperatureC = readTemperatureC();
		}
		if (detectorEnabled) {
			ldrValue = readLDR();
		} else {
			ldrValue = 0; // consider dark if disabled (so it won't force off)
		}
		potValue = readPot();
	}

	// temporary color override expiration
	if (colorOverrideActive && now >= overrideEndTime) {
		colorOverrideActive = false;
	}

	// buzzer behavior: beep while temp dangerous
	bool danger = sensorsEnabled ? tempIsDanger(temperatureC) : false;
	if (danger && buzzerEnabled) {
		if (now - lastBuzzerToggle >= BUZZER_TOGGLE_MS) {
			lastBuzzerToggle = now;
			buzzerState = !buzzerState;
			digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
		}
	} else {
		// ensure buzzer off
		buzzerState = false;
		digitalWrite(BUZZER_PIN, LOW);
	}

	// determine actual LED state
	bool ledActual = computeLedActualState();

	// determine color
	uint8_t r = 0, g = 0, b = 0;
	if (colorOverrideActive) {
		r = overrideR; g = overrideG; b = overrideB;
	} else {
		potToColor(potValue, r, g, b);
	}

	// apply LED state
	if (!ledActual) {
		setRGB(0, 0, 0);
	} else {
		setRGB(r, g, b);
	}

	if (ldrValue > 500) {
		ledRequestedOn = false;
	} else {
		ledRequestedOn = true;
	}

	// serial monitor at interval
	if (now - lastSerialMillis >= SERIAL_INTERVAL) {
		lastSerialMillis = now;
		Serial.print("LED: "); Serial.print(ledActual ? "ON" : "OFF");
		Serial.print(" | Temp: "); Serial.print(temperatureC, 1); Serial.print(" C");
		Serial.print(" | Umid: "); Serial.print(humidityPercent, 1); Serial.print("%");
		if (sensorsEnabled && tempIsDanger(temperatureC)) {
			Serial.print(" -> Perigo! Desligar!");
		}
		Serial.print(" | LDR: "); Serial.print(ldrValue);
		Serial.print(" | Pot: "); Serial.print(potValue);
		Serial.print(" | detectorEnabled: "); Serial.print(detectorEnabled ? "true" : "false");
		Serial.print(" | ledRequestedOn: "); Serial.print(ledRequestedOn ? "true" : "false");
		Serial.print(" | colorOverride: "); Serial.print(colorOverrideActive ? "true" : "false");
		Serial.print(" | commonAnode: "); Serial.println(LED_COMMON_ANODE ? "true" : "false");
	}
}