/*
 Payload format
 
  function Decoder(bytes, port) {
  var colors = ["white", "red", "green", "blue"];
  var decoded = {
    temperature: ((bytes[0] << 8) | bytes[1]) / 100,
    state: {
      color: colors[bytes[2]]
    }
  };

  return decoded;
}
*/


#include <TheThingsNetwork.h>

// Set your AppEUI and AppKey
const char *appEui = "0000000000000000";
const char *appKey = "00000000000000000000000000000000";

#define loraSerial Serial2
#define debugSerial SerialUSB

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan REPLACE_ME

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);
uint8_t color = 0;
  
void setup()
{
  loraSerial.begin(57600);
  debugSerial.begin(9600);

  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000)
    ;

  debugSerial.println("-- STATUS");
  ttn.showStatus();

  debugSerial.println("-- JOIN");
  ttn.join(appEui, appKey);
 
  // Set callback for incoming messages
  ttn.onMessage(message);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
}

void loop()
{
  uint16_t temp = getTemperature();
//  uint8_t color = getColor();
  
  byte payload[3];
  payload[0] = highByte(temp);
  payload[1] = lowByte(temp);
  payload[2] = color;

  debugSerial.print("Color: ");
  debugSerial.println(color);


  ttn.sendBytes(payload, sizeof(payload));

  // Delay between readings - 10 000 = 10 secons
  delay(10000);
}


uint16_t getTemperature()
{
  //10mV per C, 0C is 500mV
  float mVolts = (float)analogRead(TEMP_SENSOR) * 3300.0 / 1023.0;
  int temp = (mVolts - 500) * 10;
  
  debugSerial.print((mVolts - 500) / 10);
  debugSerial.println(" Celcius");
  return int(temp);
}

void message(const uint8_t *payload, size_t size, port_t port)
{  
  if (payload[0] == 1) {
    RED();
  }
  else if (payload[0] == 2) {
    GREEN();
  }
  else if (payload[0] == 3) {
    BLUE();
  }
  else {
    WHITE();
  }
}

void RED() {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  color = 1;
}

void GREEN() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
  color = 2;
}

void BLUE() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW);
  color = 3;
}

void WHITE() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  color = 0;
}
