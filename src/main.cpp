/*
 * Project Name:    ScreenSpeeder
 * Course:          TPJ655
 * Description:     Firmware for the ScreenSpeeder: a wrist-mounted, BlueTooth and IoT enabled 
 *                  controller to aide in website navigation for the visually impaired.
* Version:         1.4
 * 
 * Features:
 * - BLE keyboard for browser navigation and speech-to-text control
 * - ADXL345 accelerometer for gesture-based commands
 * - IoT integration via MQTT for remote control and monitoring
 * - Power management with deep sleep mode and wake-on-button
 * - Integrated sound cues and haptic feedback to user input
 * - RGBLED for colour association 
 * - Hall effect, rotary encoder, and push-button inputs with various keyboard controls
 * - Voice-to-text input for Windows-based machines
 * 
 * License: GNU General Public License V3.0
 *
 * Notes:
 * - Ensure proper pull-up/pull-down resistor configurations for buttons.
 * - Device must be flat during accelerometer calibration on startup.
 */

#include <Arduino.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <FastLED.h>
#include <ADXL345_WE.h>
#include <ArduinoJson.h>
#include <AS5600.h>
#include <esp_heap_caps.h>      
#include <esp32/spiram.h>
#include <BleKeyboard.h>
#include <Adafruit_DRV2605.h>
#include <Encoder.h>
#include <EncoderButton.h>
#include <DFPlayerMini_Fast.h>
#include "driver/rtc_io.h"

// Definitions for ESP32's Deep Sleep functionality
// Initial example: 
// https://randomnerdtutorials.com/esp32-external-wake-up-deep-sleep/
#define BUTTON_PIN_BITMASK(GPIO)    (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex
#define USE_EXT0_WAKEUP             1               // 1 = EXT0 wakeup, 0 = EXT1 wakeup
#define WAKEUP_GPIO                 GPIO_NUM_32  // Use an RTC GPIO pin
#define BUTTON2_GPIO                GPIO_NUM_33

// RGBLED - WS2812B definitions
#define LED_PIN      5
#define NUM_LEDS     1
#define BRIGHTNESS   255
#define LED_TYPE     WS2812B
#define COLOR_ORDER  GRB
CRGB leds[NUM_LEDS];
// Index 0: Black
// Index 1: Green - Default/Tab Navigation
// Index 2: Teal - Header Navigation
// Index 3: Blue - Link Navigation
// Index 4: Orange - Form Navigation
// Index 5: Red - Recording Indicator
const CRGB MODE_COLOURS[6] =
{
    CRGB::Black,     
    CRGB::Green,     
    CRGB::Teal,      
    CRGB::Blue,      
    CRGB::Orange,     
    CRGB::Red       
};

// Wi-Fi and MQTT Setup
const char* ssid = "";      // change according to target network
const char* password = "";  // change according to target network
// Passed to Node-Red to broker connection between MCU and IoT Dashboard
const char* mqtt_server = "broker.emqx.io";
const String mqtt_clientID = "ScreenSpeeder_IoT";
const char* mqtt_username = "";     // change to desired MQTT username
const char* mqtt_password = "";    // change to desired MQTT password
const int16_t mqtt_port = 1883;

// MQTT topic variables
// For direct communication with individual components of the IoT dashboard
const char* topic_haptic = "screenspeeder/haptic";
const char* topic_encoderSet = "screenspeeder/encoder_set";
const char* topic_encoderPos = "screenspeeder/encoder_pos";
const char* topic_dfplayerVol = "screenspeeder/df_vol";
const char* topic_accel = "screenspeeder/accel";
bool from_mqtt = false;
bool haptic_enable = true; 
bool wifi_skip = false;

// Wifi and pubsubclient objects for MQTT communication
WiFiClient espClient;
PubSubClient client(espClient);

// DFPlayer Mini Definitions and Function
static const uint8_t DFPLAYER_RX = 19; // DFP RX -> ESP32 GPIO19
static const uint8_t DFPLAYER_TX = 18; // DFP TX -> ESP32 GPIO18
uint8_t dfp_volume = 27;
#define dfSerial Serial1    // Virtual Serial for the DFP
DFPlayerMini_Fast player;
const unsigned long INACTIVITY_TIMEOUT = 60000.0;       // after 1 min of inactivity: sleep
uint64_t bitmask = BUTTON_PIN_BITMASK(WAKEUP_GPIO);
RTC_DATA_ATTR int bootCount = 0;
unsigned long button_last_press = 0.0;
unsigned long last_active = 0.0;                    // activity monitor
static bool activity = false;
static bool b1_first_press = false;
const unsigned long BUTTON_TAB_WINDOW = 2000;
const unsigned long B_DEBOUNCE = 150;
int p32_state, p33_state = 0;
static int eb_currentPos = 1;              // The MOST important variable in the entire file regarding rotary encoding

bool isRecording = false;

// Bluetooth instances (bluetooth audio, keyboard)
BleKeyboard bleKeyboard("ScreenSpeeder K", "ESP32", 100);

// AS5600 Hall Effect Definitions and Functions
AS5600 as5600;
const int STEP_DEGREES = 30;          // Degrees per step
const int STEP_THRESHOLD = 5;         // Debounce threshold in raw units
const float RAW_PER_DEGREE = 4096.0 / 360.0;
const int RAW_STEP = STEP_DEGREES * RAW_PER_DEGREE;  // ≈341 raw units per step
unsigned long lastStepTime = 0;
int16_t prevRawAngle = 0;
int32_t cumulativePosition = 0;
float stepRate = 0;

// Motor Driver DRV2605 Definitions
// Pins: 
//      I2C:    SDA and SCL
//      Power:  3V3 and GND
Adafruit_DRV2605 mtr_drv;

// Global Waveform Array: 
int wave[] = {18, 22, 28, 32, 106, 8, 55, 45, 111, 99};

// Rotary Encoder Definitions and Functions
#define ENC_A 25  // RE CLK -> ESP32 GPIO 25
#define ENC_B 26  // RE DT  -> ESP32 GPIO 26
#define ENC_SW 27 // RE SW  -> ESP32 GPIO 27

// Rotary Encoder Object
EncoderButton eb(ENC_A, ENC_B, ENC_SW);

//ADXL345 Accelerometer Definitions
#define ADXL345_I2CADDR 0x53 // I2C Address
ADXL345_WE  mainAcc = ADXL345_WE(ADXL345_I2CADDR); // Accelerometer object
unsigned long lastAccelUpdate = 0;
unsigned long lastAccelGesture = 0;
const long accelInterval = 250; // The accelerometer will update at a frequency of  4Hz
const long GESTURE_CD = 1500; // Cooldown between Gestures
// While wrist mounted, rotating at the shoulder to 70 degrees (up or down) will be considered a Gesture
const float ROLL_THRESH_X = 70.0;
// Rotating at the wrist (towards or away) at 30 degrees is considered a Gesture 
const float TILT_THRESH_Y = 30.0;
xyzFloat g, angles, raw, corrAngles;
static bool gesture_active = false;

void activity_detected(){
    last_active = millis();
    Serial.printf("Activity Detected");
}
// Left and Right Directional Button Inputs Functions
void left_button_fn(){
    activity_detected();
    // If Encoder set to Mode 3, and the interval since the 
    // last input of this type has been exceeded
    if (eb_currentPos == 3 && (millis() - button_last_press >= BUTTON_TAB_WINDOW)){
        // CTRL + Shift + Tab = Move one Browser Tab to the Left
        bleKeyboard.press(KEY_LEFT_CTRL);
        delay(50);
        bleKeyboard.press(KEY_LEFT_SHIFT);
        delay(50);
        bleKeyboard.press(KEY_TAB);
        delay(50);
        bleKeyboard.releaseAll();
    }
    else if (eb_currentPos == 4){
        delay(B_DEBOUNCE);
        bleKeyboard.press(KEY_LEFT_ARROW);
        delay(50);
        bleKeyboard.releaseAll();
    }
    // In all other cases
    else{
        // Press the UP Arrow Key
        delay(B_DEBOUNCE);
        bleKeyboard.press(KEY_UP_ARROW);
        delay(50);
        bleKeyboard.releaseAll();
    }
    button_last_press = millis();
}
void right_button_fn(){
    activity_detected();
    // If Encoder set to Mode 3, and the interval since the 
    // last input of this type has been exceeded
    if (eb_currentPos == 3 && (millis() - button_last_press >= BUTTON_TAB_WINDOW)){
        // CTRL + Tab = Move one Browser Tab to the Right
        bleKeyboard.press(KEY_LEFT_CTRL);
        delay(50);
        bleKeyboard.press(KEY_TAB);
        delay(50);
        bleKeyboard.releaseAll();
    }
    else if (eb_currentPos == 4){
        delay(B_DEBOUNCE);
        bleKeyboard.press(KEY_RIGHT_ARROW);
        delay(50);
        bleKeyboard.releaseAll();
    }
    // Otherwise
    else{
        // Press the DOWN Arrow Key
        delay(B_DEBOUNCE);
        bleKeyboard.press(KEY_DOWN_ARROW);
        delay(50);
        bleKeyboard.releaseAll();
    }
    button_last_press = millis();
}

// Function for updating the RGBLED's colour
void updateLEDColour()
{
    if (isRecording)
    {
        leds[0] = MODE_COLOURS[5];  //red
    }
    else
    {
        leds[0] = MODE_COLOURS[eb_currentPos];
    }
    FastLED.show();
}

// Function for when the ScreenSpeeder is booted or awoken to play Startup Chime
void dfp_fn(){
    if (player.begin(dfSerial, true, true)) {
        Serial.println("DFPlayer connected!");
        Serial.println("Playing startup sound");
        
        player.volume(dfp_volume); // 0-30 scale
        delay(50);
        player.playFromMP3Folder(1);
    }
    else
    {
        Serial.println("DFPlayer connection error.");
    }
    delay(500);
}

// Hall Effect Clockwise Turn Function
void cw_step(int position){
    activity_detected();
    // Switch Cases defined by Rotary Encoder position
    switch(position) {
        case 1: // Standard Tab
        bleKeyboard.press(KEY_TAB);
        break;

        case 2: // Headers (NVDA)
        bleKeyboard.press('h');
        break;

        case 3: // Links (NVDA)
        bleKeyboard.press('k');     
        break;

        case 4: // Forms (NVDA)
        bleKeyboard.press('f');
        break;
    }
    delay(50);
    bleKeyboard.releaseAll();
}
// Hall Effect Counter-Clockwise Turn Function
void ccw_step(int position){
    activity_detected();
    switch(position) {
        case 1: // Standard Tab
        bleKeyboard.press(KEY_LEFT_SHIFT);
        bleKeyboard.press(KEY_TAB);
        break;

        case 2: // Headers (NVDA)
        bleKeyboard.press(KEY_LEFT_SHIFT);
        bleKeyboard.press('h');
        break;

        case 3: // Links (NVDA)
        bleKeyboard.press(KEY_LEFT_SHIFT);
        bleKeyboard.press('k');
        break;

        case 4: // Forms (NVDA)
        bleKeyboard.press(KEY_LEFT_SHIFT);
        bleKeyboard.press('f');
        break;
    }

    delay(50);
    bleKeyboard.releaseAll();
}

// 0 = Default - 18, Strong Click 2
// 1 = Header - 22, Medium Click 2
// 2 = Link - 28, Short Double Click Strong 2
// 3 = Form - 32, Short Double Click Medium 2
// 4 = Url Select - 106, Transition Ramp Up Long Smooth 1
// 5 = Voice Input - 8, Soft Bump 60%
// 6 = Power Off/On - 55, Pulsing Medium 2
// 7 = Aux Mode - 45, Long Double Sharp Tick 2
// 8 = Browser Forward - 111, Transition Ramp Up Short Smooth 2 0-50%
// 9 = Browser Backward - 99, Transition Ramp Down Short Smooth 2 50-0%
void haptic_fn(int action){
    /* Waveforms: 
        https://cdn-shop.adafruit.com/datasheets/DRV2605.pdf
    */
    if (haptic_enable == false)
        return;
    if (action > 123) {
        action = 0;
        return;
    }
    mtr_drv.setWaveform(0, (wave[action]));
    mtr_drv.go();
}

// ISR for encoder to allow ESP32 compatibility
void IRAM_ATTR readEncoderISR() {
    eb.EncoderISR(); 
}

// One of the most important functions: changes the Encoder's mode
// depending on physically turning it OR from IoT control
void eb_handleEncoder(EncoderButton& eb){
    activity_detected();
    int increment = -eb.increment();
    if (!from_mqtt)
        eb_currentPos += increment;
    from_mqtt = false;
    
    // Ensures the modes form a "circle" and return to a valid mode if out of bounds
    if(eb_currentPos < 1) eb_currentPos = 4;
    if(eb_currentPos > 4) eb_currentPos = 1;
    
    updateLEDColour();
    
    switch(eb_currentPos){
        case 1:
        // Debug terminal print:
        Serial.println("Default Mode");
        player.playFromMP3Folder(5);
        haptic_fn(0);
        break;
        
        case 2:
        Serial.println("Header Mode");
        player.playFromMP3Folder(3);
        haptic_fn(1);
        break;
        
        case 3:
        Serial.println("Link Mode");
        player.playFromMP3Folder(4);
        haptic_fn(2);
        break;
        
        case 4:
        Serial.println("Form Mode");
        player.playFromMP3Folder(2);
        haptic_fn(3);
        break;
    }
    // Publishing the current mode to Node-Red:
    client.publish(topic_encoderPos, String(eb_currentPos).c_str());
    // Short delays are used to give the ESP's CPU some breathing room 
    delay(50);
}

// Function for sending the keyboard command to enable Windows Speech-to-Text
void sendWinH() {
    if (bleKeyboard.isConnected()) {
        bleKeyboard.press(KEY_LEFT_GUI);
        delay(50);
        bleKeyboard.press('h');
        delay(100);
        // Release both keys simultaneously
        bleKeyboard.releaseAll();
        Serial.println("Win+H sent");
    }
}
void enter_deep_sleep(){
    Serial.println("Now entering sleep mode..");
        player.playFromMP3Folder(9);
        haptic_fn(6);
        // Flash LED Orange -> Red -> Black (off)
        for (int i = 4; i <= 6; i++){
            if (i == 6){
                leds[0] = MODE_COLOURS[0];
            }
            else{
                leds[0] = MODE_COLOURS[i+1];
            }
            FastLED.show();
            delay(300);
        }
        // Remain in Deep Sleep mode until Left Button is pressed
        esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO, 1);
        esp_deep_sleep_start();
}
// The Rotary Encoder library we used requires individual functions for each click of the encoder
// Singular Click:
void eb_handleButton1(EncoderButton& eb){
    activity_detected();
    Serial.println("Single Click");
    if (isRecording)
    {
        isRecording = false;
        sendWinH();
        updateLEDColour();
        
        Serial.println("Speech-to-Text Recording Stopped");
    }
    // If in Mode 4: send "ENTER" to keyboard
    else if(eb_currentPos == 4)
    {
        bleKeyboard.press(KEY_RETURN);
        delay(50);
        bleKeyboard.releaseAll();
    }
    // Otherwise, send "SPACEBAR"
    else
    {
        bleKeyboard.press(' ');
        delay(50);
        bleKeyboard.releaseAll();
    }
}
// Double Click Function
void eb_handleButton2(EncoderButton& eb){
    activity_detected();
    Serial.println("Double Click"); // rereads selected element
    bleKeyboard.press(KEY_INSERT);
    bleKeyboard.press('l');
    delay(50);
    bleKeyboard.releaseAll();
}
// Triple Click Function
void eb_handleButton3(EncoderButton& eb){
    activity_detected();
    // In Modes 1, 2, and 4: Focus the browser's URL bar
    if (eb_currentPos != 3){
        haptic_fn(5);
        Serial.println("Triple Click"); // Focus URL bar
        bleKeyboard.press(KEY_LEFT_CTRL);
        bleKeyboard.press('l');
        delay(50);
        bleKeyboard.releaseAll();
    }
    else{ // If encoder is triple clicked in Link Mode, DEEP SLEEP MODE 
        // Deep Sleep is the ESP32's lowest power state (aside from being off altogether)
        enter_deep_sleep();
    }
}
// Encoder Long Press Function
void eb_handleLongPress(EncoderButton& eb)
{
    activity_detected();
    Serial.println("Long Press - Toggle Speech-to-Text");
    isRecording = !isRecording;
    sendWinH();
    updateLEDColour();
    FastLED.show();
    Serial.println("Speech-to-Text Started");
}

// Accelerometer Gesture Function
void accel_gesture_fn(float roll, float tilt, float yeehaw)
{
    // Checks if currently performing a gesture already, and time since last gesture
    if (!gesture_active && (millis() - lastAccelGesture > GESTURE_CD)){
        // Checks if past TILT threshold, and in which direction 
        if (tilt < -TILT_THRESH_Y){
            activity_detected();
            haptic_fn(8);
            player.playFromMP3Folder(8);
            lastAccelGesture = millis();
            gesture_active = true;
            Serial.println("Right tilt Detected: Browser Forward");
            bleKeyboard.press(KEY_LEFT_ALT);
            delay(50);
            bleKeyboard.press(KEY_RIGHT_ARROW);
            delay(50);
            
            bleKeyboard.releaseAll();
        }
        else if(tilt > TILT_THRESH_Y){
            activity_detected();
            haptic_fn(9);
            player.playFromMP3Folder(7);
            lastAccelGesture = millis();
            gesture_active = true;
            Serial.println("Left tilt Detected: Browser Back");
            bleKeyboard.press(KEY_LEFT_ALT);
            delay(50);
            bleKeyboard.press(KEY_LEFT_ARROW);
            delay(50);
            
            bleKeyboard.releaseAll();
        }
        // Checks if past ROLL threshold, and in which direction
        if (roll > ROLL_THRESH_X){
            activity_detected();
            player.playFromMP3Folder(6);
            lastAccelGesture = millis();
            gesture_active = true;
            Serial.println("Upward Roll: Browser Refresh");
            bleKeyboard.press(KEY_F5);
            delay(50);
            
            bleKeyboard.releaseAll();
        }
        else if (roll < -ROLL_THRESH_X){
            activity_detected();
            lastAccelGesture = millis();
            gesture_active = true;
            // Encoder Mode 4
            if (eb_currentPos == 4){
                Serial.println("Downward Roll: Clear field");
                // Shift + CTRL + Up Arrow
                bleKeyboard.press(KEY_LEFT_SHIFT);
                delay(50);
                bleKeyboard.press(KEY_LEFT_CTRL);
                delay(50);
                bleKeyboard.press(KEY_UP_ARROW);
                delay(50);
                bleKeyboard.releaseAll();
                delay(150);
                // Backspace
                bleKeyboard.press(KEY_BACKSPACE);
                delay(50);
                bleKeyboard.releaseAll();
            }
            // Encoder Mode 2
            else if (eb_currentPos == 2){
                activity_detected();
                Serial.println("Downward Roll: New Tab");
                // CTRL + T
                bleKeyboard.press(KEY_LEFT_CTRL);
                delay(50);
                bleKeyboard.press('t');
                delay(50);
                bleKeyboard.releaseAll();
            }
        }
    }
    // Resets currently-performing-gesture check 
    else{
        gesture_active = false;
    }
}

// Function to gather the current orientation of the accelerometer and 
// and publish the data to IoT Dashboard
void publishAccel()
{    
    mainAcc.getGValues(&g);   
    // write to json for IoT Node-RED
    JsonDocument doc;
    doc["x"] = g.x;
    doc["y"] = g.y;
    doc["z"] = g.z; 
    char jsonBuffer[200];
    serializeJson(doc, jsonBuffer);
    client.publish(topic_accel, jsonBuffer);
}

// Initialization of I2C Function (performed upon boot-up and awaken from Deep Sleep)
void scanI2C() {
    Serial.println("Scanning I2C...");
    byte count = 0;
    for (byte i = 8; i < 120; i++) {
        // Uses Wire library to scan SCL and SDA lines for I2C devices 
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
            Serial.printf("Found device at 0x%02X\n", i);
            count++;
        }
    }
    Serial.printf("Found %d I2C devices\n", count);
}

// IoT Function to bridge connection between ESP32 and Dashboard
void reconnect() {
    while (!client.connected()) {
        // Upon Connection, listen for User Input from IoT
        if (client.connect(mqtt_clientID.c_str())) {
            Serial.println("MQTT connected!");
            client.subscribe(topic_encoderSet);
            client.subscribe(topic_haptic);
            client.subscribe(topic_dfplayerVol);
        }
        else
        {
            // Retry connection until validated
            Serial.println(client.state());
            delay(1500);
        }
    }
}

// IoT Function for handling data received from Dashboard (user input)
void callback(char* topic, byte* payload, unsigned int length) {
    // Add NULL BYTE to end of payload
    payload[length] = '\0';
    
    // From Buttons to change Encoder Mode
    if (String(topic) == topic_encoderSet) {
        // Convert payload to integer
        int new_position = atoi((char*)payload);
        // Ensure Encoder Mode is Valid
        if ((new_position >= 1 && new_position <= 4) && (new_position != eb_currentPos)) {
            eb_currentPos = new_position;
            from_mqtt = true;
            eb_handleEncoder(eb);
        }
    }
    
    // From Toggle Switch to Enable/Disable all Haptic feedback 
    else if (String(topic) == topic_haptic){
        for (int i = 0; i < length; i++) {
            Serial.print((char)payload[i]);
        }
        haptic_enable = (strcmp((char*)payload, "true")== 0);
        Serial.printf(": Haptic %s\n", haptic_enable ? "enabled":"disabled");
    }
    
    // From Slider to define Speaker's volume
    else if (String(topic) == topic_dfplayerVol) {
        int volume = 0;
        // Checks if the payload is a valid integer for the DFPlayer to accept
        if (sscanf((char*)payload, "%d", &volume) == 1) {
            player.volume(volume);
            delay(50);
            Serial.printf("Volume set to %d\n", volume);
        }
        else{
            // THIS IS NO PLACE FOR A COURT FOOL 
            Serial.println("TOO LOUD. THIS IS NOT A JEST.");
        }
    }
}

/*
 * This is where the build ACTUALLY begins. After compilation of 
 * the libraries and this file, this is the point where feedback
 * from the ESP can be generated and viewed in the Serial Terminal
 * (for debugging).
*/
void setup()
{
    // Setting the ESP's baud rate for communications 
    Serial.begin(115200);
    
    // Setting the virtual Serial for the DFP's onboard micro-micro control unit
    dfSerial.begin(9600, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);
    player.begin(dfSerial, true);
    delay(1000);
    
    // Informing the ESP to expect buttons on predefined pins, 
    // connected via pull down resistors 
    pinMode(WAKEUP_GPIO, INPUT_PULLDOWN);
    pinMode(BUTTON2_GPIO, INPUT_PULLDOWN);
    
    // Every time the ESP wakes from Deep Sleep, this number increments and
    // is printed to the terminal
    ++bootCount;
    Serial.println("Boot number: " + String(bootCount));
    
    // Infrom the ESP what to expect while in Deep Sleep as a Wake Up call
    esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO, 1);
    rtc_gpio_pulldown_en(WAKEUP_GPIO);
    rtc_gpio_pullup_dis(WAKEUP_GPIO);
    
    // Initializes the expanded PSRAM onboard the ESP, allowing for increased
    // memory while in operation
    if (psramInit()) {
        Serial.printf("PSRAM size: %d bytes\n", ESP.getPsramSize());
        Serial.printf("PSRAM free: %d bytes\n", ESP.getFreePsram());
    } 
    else {
        Serial.println("PSRAM not available or not initialized!");
    }
    
    // I2C services initialized
    Wire.begin();
    
    // Motor Driver Setup
    mtr_drv.begin();
    mtr_drv.selectLibrary(1);
    mtr_drv.setMode(DRV2605_MODE_INTTRIG);
    
    // AS5600 Hall Effect Initialization
    if (!as5600.begin()) {
        Serial.println("AS5600 not found!");
        while(1);
    }
    
    // Encoder Setup and Initialization
    eb.Encoderbegin();
    eb.Encodersetup(readEncoderISR);
    
    // Defining each Function the encoder is to use when it registers each input
    eb.setClickHandler(eb_handleButton1); 
    eb.setDoubleClickHandler(eb_handleButton2);
    eb.setTripleClickHandler(eb_handleButton3);
    eb.setEncoderHandler(eb_handleEncoder);
    // Time between "turns" (200ms)
    eb.setRateLimit(200);
    // Window of time between clicks to allow for multi-click inputs (500ms)
    eb.setMultiClickInterval(500);   
    eb.setLongPressHandler(eb_handleLongPress);
    // Time window for long press
    eb.setLongClickDuration(2000);       
    
    // BlueTooth Keyboard services Iniitialize
    bleKeyboard.begin();
    
    // Scan the I2C pins for connected devices
    scanI2C();
    
    // Initializing the RGBLED
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS/6);
    
    // AS5600 Hall Effect Initialization
    as5600.begin(); 
    as5600.setDirection(AS5600_CLOCK_WISE);
    
    // ADXL345 Accelerometer Calibration
    if(!mainAcc.init())
    {
        Serial.println("ADXL345 not connected!");
        while(1){
            Serial.println("Awaiting response...");
            delay(1000);
            if(mainAcc.init()) continue;
        };
    }
    else
    {
        // Once the accelerometer has been connected, it must gather its initial
        // orientation for proper operation
        Serial.println("Accelerometer Calibrating, please hold comfortably.");
        delay(2000);
        mainAcc.measureAngleOffsets();
        Serial.println("Calibration complete.");
    }
    
    //Initializing the Accelerometer
    mainAcc.setRange(ADXL345_RANGE_4G);
    mainAcc.setDataRate(ADXL345_DATA_RATE_25);  // 25Hz
    mainAcc.setMeasureMode(true);  // Enable measurement
    
    // Initializing the DFPlayer 
    dfp_fn();
    
    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    bool purple = true;
    while (WiFi.status() != WL_CONNECTED) {
        if (purple){
            leds[0] = CRGB::Purple; // Set colour to Purple
        }
        else{
            leds[0] = MODE_COLOURS[0]; // Set colour to Black
        }
        purple = !purple;
        FastLED.show();
    delay(500);
        Serial.print(".");
        if (digitalRead(BUTTON2_GPIO) == HIGH){
            wifi_skip = true;
            break;
        }
    }
    if (wifi_skip){
        Serial.println("\nContinuing with no Wi-Fi");
    }
    else{
        Serial.println("\nWiFi connected");
    }
    delay(500);
    leds[0] = MODE_COLOURS[1]; // Set colour to Green
    FastLED.show();

    // IoT MQTT Setup
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    last_active = millis();
}

/*
 * This is the heart of the device's operation. After all the checks
 * and initializations have been made, the loop() function is the 
 * constantly running thread of the ESP.
 */
void loop()
{
    // Ensuring a stable connetion to the IoT services
    // and constantly checking for input from the Dashboard
    if (!client.connected() && wifi_skip == false) reconnect();
    client.loop();
    
    // BlueTooth Connection
    if (!bleKeyboard.isConnected()) {
        Serial.println("BLE keyboard not connected!");
        delay(500);
        return;
    }
    
    // At a set interval, the data of the accelerometer is published  to the Dashboard 
    if (millis() - lastAccelUpdate >= accelInterval){
        publishAccel();
        lastAccelUpdate = millis();
    }
    
    // Checks if last input was more than 1 minute ago
    // if so, enter DEEP SLEEP
    if (millis() - last_active >= INACTIVITY_TIMEOUT){
        Serial.println("Inactivity Timeout..");
        enter_deep_sleep();
    }
    // Every loop iteration, the button GPIOs are checked for a HIGH value.
    // HIGH = the button is being pressed, call the associated function 
    p32_state = digitalRead(WAKEUP_GPIO);
    p33_state = digitalRead(BUTTON2_GPIO);
    if (p32_state == HIGH){
        left_button_fn();
    }
    else if(p33_state == HIGH){
        right_button_fn();
    }
    
    // Each iteration, the accelerometer gathers its positional information
    // and some is sent to a function associated with Gestures 
    mainAcc.getGValues(&g);
    mainAcc.getAngles(&angles);
    if (bleKeyboard.isConnected()){accel_gesture_fn(angles.x, angles.y, angles.z);}
    
    // The AS5600 Hall Effect Sensor can detect very minute changes in its 
    // surrounding magnetic field, which are very useful for determining user input
    int16_t currentRaw = as5600.rawAngle();
    int16_t delta = currentRaw - prevRawAngle;
    // Handle overflow (0 ↔ 4095 transition)
    if (delta < -2000) delta += 4096;  // Large positive movement
    if (delta > 2000) delta -= 4096;   // Large negative movement
    // Update cumulative position
    cumulativePosition += delta;
    prevRawAngle = currentRaw;
    // Check if movement exceeds step threshold
    if (abs(cumulativePosition) >= RAW_STEP) {
        // Calculating Step speed (step/sec)
        unsigned long currentTime = millis();
        unsigned long timeDelta = currentTime - lastStepTime;
        if (lastStepTime > 0){
            stepRate = 1000.0 / timeDelta;
        }
        lastStepTime = currentTime;
        int steps = cumulativePosition / RAW_STEP;
        cumulativePosition %= RAW_STEP;  // Carry over remainder

        // Send keypresses based on direction
        if (steps < 0 ) { // Clockwise  
            cw_step(eb_currentPos);
        } 
        else {
            ccw_step(eb_currentPos);
        }
    }

    // Small movements debounce
    if (abs(cumulativePosition) < STEP_THRESHOLD) {
        cumulativePosition = 0;
    }
    // For one of the most important parts of the entire device, 
    // all that is required in the loop() is one line:
    eb.update();  
    
    // Each loop iteration has a 0.001 second delay to ensure
    // the I2C devices don't have their signals conflict
    delay(10);
}