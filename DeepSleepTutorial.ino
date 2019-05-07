#include <Arduino.h>
#include <Sodaq_RN2483.h>
#include <Sodaq_wdt.h>
#include <SPI.h>
#include <RN487x_BLE.h>
#include <RTCTimer.h>
#include <RTCZero.h>

#define CONSOLE_STREAM  SERIAL_PORT_MONITOR

#define debugSerial     SerialUSB       // debug on USB
#define bleSerial       Serial1         // Bluetooth module Serial
#define loraSerial      Serial2         // LoRa module Serial

#define LORA_BAUD       57600
#define DEBUG_BAUD      57600

#define NIBBLE_TO_HEX_CHAR(i) ((i <= 9) ? ('0' + i) : ('A' - 10 + i))
#define HIGH_NIBBLE(i) ((i >> 4) & 0x0F)
#define LOW_NIBBLE(i) (i & 0x0F)

RTCZero rtc;
RTCTimer timer;

volatile bool minuteFlag;

//
//                          setup your constants here
//
const uint8_t records_to_send = 2;           // set this to change the amount of records to send
const uint8_t record_every_x_minutes = 1;     // set this to the desired interval in minutes
const uint8_t spreading_factor = 7;           // set this to the desired LoRa spreading factor


// *****************************************************************************************
// LoRa communication setup! 

// true: use OTAA
// false: use ABP
bool OTAA = true;

// ABP setup (device address)
// USE YOUR OWN KEYS!
const uint8_t devAddr[4] =
{ 
  0x00, 0x00, 0x00, 0x00 
};

// application session key
// USE YOUR OWN KEYS!
const uint8_t appSKey[16] =
{
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// network session key
// USE YOUR OWN KEYS!
const uint8_t nwkSKey[16] =
{
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// OTAA (device EUI) - use msb formatting
// With using the GetHWEUI() function the HWEUI will be used
static uint8_t DevEUI[8]
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

const uint8_t AppEUI[8] =
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

const uint8_t AppKey[16] =
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00x2A };


// *****************************************************************************************
// setup

bool LoRa_sleeps = false;
uint8_t message[records_to_send*2];

void setup()
{
  sodaq_wdt_enable(WDT_PERIOD_8X);       // Enable the wdt at maximum interval
  sodaq_wdt_reset();                                
  sodaq_wdt_safe_delay(5000);

  pinMode(TEMP_SENSOR, INPUT);
  
  initRtc();

  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;     // sets SAMD sleep mode to deep sleep

  // networks
  loraSerial.begin(LoRaBee.getDefaultBaudRate());
  LoRaBee.init(loraSerial, LORA_RESET);             
  setupLoRa();                                      // set the gears in motion

  initRtcTimer();                                   // timer interrupt > 1 minute interval

  sodaq_wdt_reset();

  SerialUSB.flush();
  SerialUSB.end();
  USBDevice.detach(); 
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;       // Disable USB
  
  sleep_setup();
}

// *****************************************************************************************
// loop

// array for temperature we wish to send; 
// here we're taking a 6-value (12 bytes) array to send at once
// intended to measure every n*60 seconds, send every x messages.

const uint8_t measurements_to_send = 6;
int temperature_array[measurements_to_send];
uint8_t list_iter = 0;    // variable to keep track of array index and when to send

void loop() 
{  
  if (sodaq_wdt_flag) {
    sodaq_wdt_reset();
    sodaq_wdt_flag = false;
  }

  if (minuteFlag) {
    timer.update();
    minuteFlag = false;
  }

  systemSleep();
}

// *****************************************************************************************
// Sleep commands

void BT_powerdown()
{
  rn487xBle.hwInit();
  bleSerial.begin(rn487xBle.getDefaultBaudRate());
  rn487xBle.initBleStream(&bleSerial);
  rn487xBle.enterCommandMode();
  rn487xBle.hwSleep();
  bleSerial.end();
}

void sleep_setup()
{
  // set FLASH to deep sleep & reset SPI pins for min. energy consumption
  DFlashUltraDeepSleep();

  sleep_LoRa();

  // RN4871 BT/BLE module sleep
  BT_powerdown();
}

void systemSleep()                    // Since only LoRa and MCU awake, only set those to sleep
{
  if (!LoRa_sleeps)                   // Skip if LoRa is asleep
  {
    sleep_LoRa();
  }
  
  noInterrupts();
  if (!(sodaq_wdt_flag || minuteFlag)) {
    interrupts();
    debugSerial.println("Sleeping");
    __WFI();              // SAMD sleep
  }
  interrupts();
}

void sleep_LoRa()
{
    loraSerial.flush();
    LoRaBee.sleep();
    LoRa_sleeps = true;
    sodaq_wdt_safe_delay(5);
}

void wake_LoRa()
{
  LoRa_sleeps = false;
  LoRaBee.wakeUp();
}


// *****************************************************************************************
// SST25PF040C Flash functions (SPI)

void DFlashUltraDeepSleep()
{
  static const uint8_t SS_DFLASH  = 44 ;
  // SPI initialisation
  SPI.begin();

  // Initialise the CS pin for the data flash
  pinMode(SS_DFLASH, OUTPUT);
  digitalWrite(SS_DFLASH, HIGH);

  transmit(0xB9);
  
  SPI.end();

  // Resets the pins used
  resetSPIPins();
}

void transmit(uint8_t val)
{
  SPISettings settings;
  digitalWrite(SS_DFLASH, LOW);
  SPI.beginTransaction(settings);
  
  SPI.transfer(val);

  SPI.endTransaction();
  digitalWrite(SS_DFLASH, HIGH);

  delayMicroseconds(1000);
}

void resetSPIPins()
{
  resetPin(MISO);
  resetPin(MOSI);
  resetPin(SCK);
  resetPin(SS_DFLASH);
}

void resetPin(uint8_t pin)
{
  PORT->Group[g_APinDescription[pin].ulPort].
  PINCFG[g_APinDescription[pin].ulPin].reg=(uint8_t)(0);
  PORT->Group[g_APinDescription[pin].ulPort].
  DIRCLR.reg = (uint32_t)(1<<g_APinDescription[pin].ulPin);
  PORT->Group[g_APinDescription[pin].ulPort].
  OUTCLR.reg = (uint32_t) (1<<g_APinDescription[pin].ulPin);
}

// *****************************************************************************************
// RN2483/RN2903 LoRa commands
// for RN2903: uncomment the setFsbChannels line.

void setupLoRa()
{
  getHWEUI();

  if(!OTAA){
    setupLoRaABP();   // ABP setup
  } else {
    setupLoRaOTAA();  // OTAA setup
  }
  // Uncomment the following line to for the RN2903 with the Actility Network. 
  // For OTAA, update the DEFAULT_FSB in the library
  // LoRaBee.setFsbChannels(1);

  LoRaBee.setSpreadingFactor(spreading_factor);
  
}

void setupLoRaABP(){  
  if (LoRaBee.initABP(loraSerial, devAddr, appSKey, nwkSKey, true))
  {
    debugSerial.println("Communication to LoRaBEE successful.");
  }
  else
  {
    debugSerial.println("Communication to LoRaBEE failed!");
  }
}

void setupLoRaOTAA(){
  if (LoRaBee.initOTA(loraSerial, DevEUI, AppEUI, AppKey, true))
  {
    debugSerial.println("Network connection successful.");
  }
  else
  {
    debugSerial.println("Network connection failed!");
  }
}

static void getHWEUI() // gets + stores HWEUI
{
  uint8_t len = LoRaBee.getHWEUI(DevEUI, sizeof(DevEUI)); 
}

void send_message(uint8_t*val, size_t val_size) {
  wake_LoRa();

  // since the debug port is not enabled in this example, the debug message is not printed
  switch (LoRaBee.send(1, (uint8_t*)val, val_size))    // send(port, payload, length)
  {
  case NoError:
    debugSerial.println("Successful");
    break;
  case NoResponse:
    debugSerial.println("No Response");
    break;
  case Timeout:
    debugSerial.println("Timeout. starting 20 second delay");
    delay(20000);
    break;
  case PayloadSizeError:
    debugSerial.println("Payload too large!");
    break;
  case InternalError:   
    debugSerial.println("Internal Error; resetting module");
    setupLoRa();
    break;
  case Busy:
    debugSerial.println("LoRa module active");
    delay(10000);
    break;
  case NetworkFatalError:
    debugSerial.println("Network connection error; resetting module");
    setupLoRa();
    break;
  case NotConnected:
    debugSerial.println("Not connected; resetting module");
    setupLoRa();
    break;
  case NoAcknowledgment:
    debugSerial.println("No acknowledgement received");
    break;
  default:
    break;
  }
  sleep_LoRa();  
}

// *****************************************************************************************
// Temperature Sensor functions
// output: 2-byte int centi-celcius (divide by 100 to type float for correct value).

void getTemperature()
{
  int int_temp;
  uint8_t negativeFlag;

  float mVolts = (float)analogRead(TEMP_SENSOR) * 3300.0 / 1024.0;
  float temp = (mVolts - 500.0) / 10.0;

  temp *= 100;

  if (temp < 0) negativeFlag = 0x80;
  else negativeFlag = 0x00;

  int_temp = abs((int) temp);

  message[list_iter*2]     = (int_temp >> 8) | negativeFlag;
  message[list_iter*2 + 1] = int_temp & 0xFF;
}



// *****************************************************************************************
// RTC functions

// Initializes the RTC
void initRtc()
{
    rtc.begin();

    // Schedule the wakeup interrupt for every minute
    // Alarm is triggered 1 cycle after match
    rtc.setAlarmSeconds(59);
    rtc.enableAlarm(RTCZero::MATCH_SS); // alarm every minute

    // Attach handler
    rtc.attachInterrupt(rtcAlarmHandler);

    // This sets it to 2000-01-01
    rtc.setEpoch(0);
}

// Runs every minute by the rtc alarm.
void rtcAlarmHandler()
{
    minuteFlag = true;
}

// Initializes the RTC Timer
void initRtcTimer()
{
  debugSerial.println("init rtc timer");
    timer.setNowCallback(getNow); // set how to get the current time
    timer.allowMultipleEvents();

    resetRtcTimerEvents();
}


void resetRtcTimerEvents()
{
    // Schedule the default fix event (if applicable)
    timer.every(record_every_x_minutes * 60, measureTemperature);
    debugSerial.println("event set");
} 

// Returns current datetime in seconds since epoch
uint32_t getNow()
{
    return rtc.getEpoch();
}

// Default event parameter
void measureTemperature(uint32_t now)
{
  getTemperature();
  
  list_iter++;
  
  if (!(list_iter < records_to_send)) {
    send_message((uint8_t*) &message, sizeof(message));
    list_iter = 0;
  }
}
