


  #include <ArduinoBLE.h>
  #include <FastLED.h>

  // ----- LED Configuration -----
  // Change the number of LEDs to 29
  #define LED_PIN     6         // The pin connected to the LED strip data line
  #define NUM_LEDS    29        // Number of LEDs in your strip (updated from 60 to 29)
  #define LED_TYPE    WS2812B   // Type of LED strip (WS2812B in this case)
  #define COLOR_ORDER GRB       // Color order
  #define BRIGHTNESS  50       // Global brightness

  #define FLOWER_IO 2

  CRGB leds[NUM_LEDS];

  const int LED = LED_BUILTIN;

  // Define 7 colors (e.g., chakra colors)
  const CRGB chakraColors[7] = {
    CRGB::Red,            // Level 1
    CRGB::Orange,         // Level 2
    CRGB::Yellow,         // Level 3
    CRGB::Green,          // Level 4
    CRGB::Blue,           // Level 5
    CRGB(75, 0, 130),     // Level 6 (Indigo)
    CRGB::Purple         // Level 7
  };

  // ----- BLE Setup -----
  // Same service & BPM characteristic as before
  BLEService pulseService("0000FFE0-0000-1000-8000-00805F9B34FB");
  BLEByteCharacteristic pulseChar("0000FFE1-0000-1000-8000-00805F9B34FB", BLERead | BLENotify);

  // --- NEW: RX Characteristic for incoming messages from the phone ---
  // Using up to 20 bytes. We allow BLEWrite or BLEWriteWithoutResponse so the phone can send data.
  BLECharacteristic rxChar("0000FFE2-0000-1000-8000-00805F9B34FB", BLEWrite | BLEWriteWithoutResponse, 20);

  // ----- Pin Configuration -----
  // Pulse Sensor beat detection (adapted from PulseSensorPlayground logic) 
  const int PULSE_PIN = A0;
  int Threshold = 520;                    // Dynamic threshold (initialized to midpoint)
  static int Signal, BPM;
  static int IBI = 600;                   // Inter-beat interval (ms) initialized to 600ms (100 BPM)
  static bool Pulse = false;
  static bool firstBeat = true, secondBeat = false;
  static int P = 520, T = 520;            // Peak and trough, start at mid-range
  static int amp = 100;                   // Amplitude of pulse wave
  static unsigned long sampleCounter = 0;
  static unsigned long lastBeatTime = 0;
  static int rate[10] = {0};              // Last 10 IBI values
  static bool beatDetected = false;       // Flag set when a new beat is found

  unsigned long calmStartTime = 0;
  bool inCalmState = false;

  void checkPulseSensor() {
    // Call this function frequently (every loop iteration). It will read the sensor at ~2ms intervals.
    static unsigned long lastSampleMicros = 0;
    unsigned long now = micros();
    if (now - lastSampleMicros >= 2000) {        // 2ms elapsed?
      lastSampleMicros += 2000;                  // schedule next sample time
      Signal = analogRead(PULSE_PIN);            // read the pulse sensor analog value
      sampleCounter += 2;                       // track time in ms since start (2 ms per sample)
      int N = sampleCounter - lastBeatTime;     // time since last beat in ms

      // 1. Peak and trough tracking (to adjust threshold dynamically):
      if (Signal < Threshold && N > (IBI/5)*3) {  
        // Tracking trough: update T to lowest seen value after 60% of last IBI
        if (Signal < T) {
          T = Signal;
        }
      }
      if (Signal > Threshold && Signal > P) {    
        // Tracking peak: update P to highest seen value (only above threshold to avoid noise)
        P = Signal;
      }

      // 2. Check for a new beat (when the signal rises above threshold after the refractory period):
      if (N > 250) {  
        // Avoid high-frequency noise – require at least 250ms between beats
        if ((Signal > Threshold) && (Pulse == false) && (N > (IBI/5)*3)) {
          // Signal crossed threshold upward, and it's been >60% of last IBI since last beat
          Pulse = true;                           // we think a beat is starting
          IBI = sampleCounter - lastBeatTime;     // measure time between beats in ms
          lastBeatTime = sampleCounter;

          if (firstBeat) {
            // Discard the first beat detection because IBI is not yet valid
            firstBeat = false;
            secondBeat = true;
            return;                               // jump out, no BPM calculation on first beat
          }
          if (secondBeat) {
            // Second beat – initialize the running average
            secondBeat = false;
            for (int i = 0; i < 10; ++i) {
              rate[i] = IBI;
            }
          }
          // Calculate BPM as the average of the last 10 IBI values:
          long runningTotal = 0;
          for (int i = 0; i < 9; ++i) {
            rate[i] = rate[i+1];                  // shift data in the rate array
            runningTotal += rate[i];
          }
          rate[9] = IBI;
          runningTotal += rate[9];
          long averageIBI = runningTotal / 10;
          BPM = 60000 / averageIBI;               // BPM (beats per minute)
          beatDetected = true;                    // set the flag to indicate a new beat
        }
      }

      // 3. When the signal falls below the threshold, conclude the beat:
      if (Signal < Threshold && Pulse == true) {
        // The wave is now below threshold, so the beat “pulse” is over
        Pulse = false;                           
        amp = P - T;                              // calculate pulse wave amplitude
        Threshold = T + amp/2;                    // set new threshold at 50% of the amplitude
        // Reset peak and trough for next cycle, centered at the new threshold:
        P = Threshold;
        T = Threshold;
      }
    }
  }

  bool sawStartOfBeat() {
    // Checks if a new beat was detected since last call
    if (beatDetected) {
      beatDetected = false;    // reset the flag
      return true;
    }
    return false;
  }

  int getBeatsPerMinute() {
    // Returns the most recent BPM value
    return BPM;
  }

  // -------------------- 花瓣开合（阻塞控制） --------------------
  void triggerFlowerOpen() {
    //Serial.println("💡 启动花瓣开合器");
    digitalWrite(FLOWER_IO, HIGH);
    delay(15000);  // 模块执行时间
    digitalWrite(FLOWER_IO, LOW);
    //Serial.println("✅ 花瓣动作完成");
    inCalmState = false;
  }

  // ----- Setup -----
  void setup() {
    //Serial.begin(115200);
    //while (!Serial); // Wait for Serial Monitor

    if (!BLE.begin()) {
      //Serial.println("ERROR: Could not start BLE!");
      while (1);
    }
    
    BLE.setLocalName("PulseSensor");
    BLE.setAdvertisedService(pulseService);

    // Add BOTH characteristics to the service
    pulseService.addCharacteristic(pulseChar); // BPM out
    pulseService.addCharacteristic(rxChar);    // incoming messages

    BLE.addService(pulseService);

    // Initialize BPM characteristic
    pulseChar.writeValue((uint8_t)0);

    // Start advertising
    BLE.advertise();
    //Serial.println("BLE advertising...");

    // Initialize the rate array
    for (int i = 0; i < 10; i++) {
      rate[i] = 600; 
    }

    pinMode(LED, OUTPUT);
    pinMode(FLOWER_IO, OUTPUT);
    digitalWrite(FLOWER_IO, LOW);
    
    // Initialize LED strip
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.clear();
    FastLED.show();
  }

  // ----- Main Loop -----
  void loop() {
    // Wait for a BLE central to connect
    BLEDevice central = BLE.central();
    if (central) {
      //Serial.print("Connected to central: ");
      //Serial.println(central.address());
      
      while (central.connected()) {
        BLE.poll(); // Process BLE events

        // 1) Sample the pulse sensor
        checkPulseSensor();

        // 2) Check if we just detected a new beat
        if (sawStartOfBeat()) {
          int bpm = getBeatsPerMinute() - 30;
          pulseChar.writeValue((uint8_t)bpm);
          //Serial.print("BPM: ");
          //Serial.println(bpm);
        }

        // 3) Check if the phone wrote new data to rxChar
        if (rxChar.written()) {
          int length = rxChar.valueLength();
          if (length > 0) {
            char buffer[21];  // up to 20 bytes + null terminator
            rxChar.readValue(buffer, length);
            buffer[length] = '\0';
            //Serial.print("Received message: ");
            //Serial.println(buffer);

            // Convert the message to an integer (assume values 1 to 7)
            int level = atoi(buffer);
            if (level >= 0 && level <= 7) {
              // Clear all LEDs before updating
              fill_solid(leds, NUM_LEDS, CRGB::Black);

              // Calculate the base segment size and any extra LEDs to distribute
              int baseSegmentSize = NUM_LEDS / 7; 
              int extraLEDs = NUM_LEDS % 7;

              int startIndex = 0;
              // Loop through each level up to the current 'level'
              for (int i = 0; i < level; i++) {
                int segmentSize = baseSegmentSize;
                // Distribute one extra LED to the first 'extraLEDs' segments
                if (i < extraLEDs) {
                  segmentSize++;
                }
                for (int j = 0; j < segmentSize; j++) {
                  leds[startIndex + j] = chakraColors[i];
                }
                startIndex += segmentSize;
              }
            
              // Update the LED strip to display changes
              FastLED.show();
            }
            
            if (level == 7){
              if (!inCalmState){
                inCalmState = true;
                calmStartTime = millis();
              }
              else if (millis() - calmStartTime >= 5000){
                //Serial.println("🌸 ---------莲花开了！------------");
                triggerFlowerOpen();
              }
            }
          }
        }

        delay(2); // keep ~500 Hz sampling rate
      }
      
      //Serial.print("Disconnected from central: ");
      //Serial.println(central.address());
    }
  }
