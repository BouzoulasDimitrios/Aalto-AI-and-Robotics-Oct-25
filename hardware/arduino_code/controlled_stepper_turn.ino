#include <Stepper.h>

const long stepsPerRevolution = 2048;                // your working value
Stepper stepper(stepsPerRevolution, 8, 10, 9, 11);   // IN1,3,2,4 order

double accum = 0.0;  // keep fractional steps so angles stay accurate

void stepDegrees(double deg) {
  accum += deg * (double)stepsPerRevolution / 360.0;
  long whole = lround(accum);
  accum -= whole;
  stepper.step((int)whole);   // blocking move; simple and reliable
}

void setup() {
  Serial.begin(115200);
  stepper.setSpeed(10);       // rpm
  delay(1000);                // Uno auto-resets when the port opens
  Serial.println("READY");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');  // waits for newline
    cmd.trim();

    if (cmd == "+")                  { stepDegrees(10);  Serial.println("OK"); }
    else if (cmd == "-")             { stepDegrees(-10); Serial.println("OK"); }
    else if (cmd.startsWith("MOVE ")) {
      double deg = cmd.substring(5).toFloat();
      stepDegrees(deg);              Serial.println("OK");
    }
    else if (cmd.startsWith("RPM ")) {
      int rpm = cmd.substring(4).toInt();
      rpm = constrain(rpm, 2, 20);
      stepper.setSpeed(rpm);         Serial.println("OK");
    }
    else { Serial.println("ERR"); }
  }
}
