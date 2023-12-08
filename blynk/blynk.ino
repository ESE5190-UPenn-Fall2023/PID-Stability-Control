/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

/* Fill-in your Template ID (only if using Blynk.Cloud) */
#define BLYNK_TEMPLATE_ID "..."
#define BLYNK_TEMPLATE_NAME "..."
#define BLYNK_AUTH_TOKEN "..."

#define KP_PIN 25
#define KI_PIN 26
#define KD_PIN 19

#define MAX_KP_VALUE 10.0
#define MAX_KI_VALUE 5.0
#define MAX_KD_VALUE 5.0

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = BLYNK_AUTH_TOKEN;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "...";
char pass[] = "...";

void setup() {
  Serial.begin(9600); // Start serial communication

  // Set pins as output
  pinMode(KP_PIN, OUTPUT);
  pinMode(KI_PIN, OUTPUT);
  pinMode(KD_PIN, OUTPUT);

  // Set initial PID values
  analogWrite(KP_PIN, (int)(1.0 / MAX_KP_VALUE * 255));
  analogWrite(KI_PIN, (int)(0.4 / MAX_KI_VALUE * 255));
  analogWrite(KD_PIN, (int)(0.0 / MAX_KD_VALUE * 255));

  // Connect to WiFi
  WiFi.begin(ssid, pass);

  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Blynk.begin(auth, ssid, pass);
}

BLYNK_WRITE(V0) {
    float KP = param.asFloat();
    Serial.println("KP set: ");
    Serial.println(KP);
    analogWrite(KP_PIN, (int)(KP / MAX_KP_VALUE * 255));
}

BLYNK_WRITE(V1) {
    float KI = param.asFloat();
    Serial.println("KI set: ");
    Serial.println(KI);
    analogWrite(KI_PIN, (int)(KI / MAX_KI_VALUE * 255));
}

BLYNK_WRITE(V2) {
    float KD = param.asFloat();
    Serial.println("KD set: ");
    Serial.println(KD);
    analogWrite(KD_PIN, (int)(KD / MAX_KD_VALUE * 255));
}

void loop() {
  Blynk.run();
}
