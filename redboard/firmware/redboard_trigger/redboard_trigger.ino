const int ANALOG_PIN = A0;
const int DIGITAL_PIN = 7;

// Operational parameters (default fallbacks)
unsigned long delayTime = 100;       // ms
unsigned long opTime = 400;          // ms
unsigned long lockoutDuration = 10000; // ms (10 seconds default)
float voltageThreshold = 2.5;

// State tracking variables
bool triggerLocked = false; 
unsigned long lockStartTime = 0;       

// Returns the current A0 voltage in volts (0.0 - 5.0 for a 10-bit ADC at 5V ref)
float readA0Voltage() {
  int rawValue = analogRead(ANALOG_PIN);
  return (rawValue * 5.0) / 1023.0;
}

void setup() {
  Serial.begin(9600);
  pinMode(DIGITAL_PIN, OUTPUT);
  digitalWrite(DIGITAL_PIN, LOW);
}

void loop() {
  // 1. Python configuration input (Parses 4 arguments separated by commas)
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // On-demand A0 query -- lets the host poll the live voltage at any time,
    // including while the trigger is locked out.
    if (input == "?") {
      Serial.print("A0: ");
      Serial.print(readA0Voltage());
      Serial.print("V, locked=");
      Serial.println(triggerLocked ? 1 : 0);
    }
    else if (input.length() > 0) {
      int firstComma = input.indexOf(',');
      int secondComma = input.indexOf(',', firstComma + 1);
      int thirdComma = input.indexOf(',', secondComma + 1);
      
      if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
        String threshStr = input.substring(0, firstComma);
        String delayStr = input.substring(firstComma + 1, secondComma);
        String opStr = input.substring(secondComma + 1, thirdComma);
        String lockStr = input.substring(thirdComma + 1);
        
        voltageThreshold = threshStr.toFloat();
        delayTime = delayStr.toFloat() * 1000; 
        opTime = opStr.toFloat() * 1000;
        lockoutDuration = lockStr.toFloat() * 1000; // Convert seconds to ms
        
        // Return verification status to Python
        Serial.print("UPDATED: Threshold=");
        Serial.print(voltageThreshold);
        Serial.print("V, Delay=");
        Serial.print(delayTime / 1000.0);
        Serial.print("s, OpTime=");
        Serial.print(opTime / 1000.0);
        Serial.print("s, Lockout=");
        Serial.print(lockoutDuration / 1000.0);
        Serial.println("s");
      }
    }
  }

  // 2. Read the voltage
  float currentVoltage = readA0Voltage();

  // 3. Trigger conditions
  if (currentVoltage > voltageThreshold && !triggerLocked) {
    triggerLocked = true;

    Serial.print("TRIGGER: ");
    Serial.print(currentVoltage);
    Serial.println("V");

    delay(delayTime);
    digitalWrite(DIGITAL_PIN, HIGH);
    delay(opTime);
    digitalWrite(DIGITAL_PIN, LOW);

    // Start the lockout clock only after the pulse has finished, so the full
    // lockoutDuration elapses before A0 is acted on again.
    lockStartTime = millis();

    Serial.print("LOCKOUT: ");
    Serial.print(lockoutDuration / 1000.0);
    Serial.println("s");
  }

  // 4. Timer-Based Reset Condition using the dynamic user input value
  if (triggerLocked && (millis() - lockStartTime >= lockoutDuration)) {
    triggerLocked = false;
    Serial.println("READY: monitoring A0");
  }
}