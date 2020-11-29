  #include <Ultrasonic.h>
  
  unsigned int i;
  unsigned int echoPin = 2;
  unsigned int motorPin1 = 0;
  unsigned int motorPin2 = 1;
  unsigned int triggerPin = 3;
  
  unsigned long ts = 30; // Sampling Period
  unsigned long start = 0;
  
  float x; // Measured variable (process variable)
  float u; // Control Action (controlled variable)
  float ei = 0;
  float ed = 0;
  float kp = 1.2; // 1.2
  float kd = 1.2; // 1
  float xSp = 15; // Setpoint in centimeters
  float kDead = 7; // Dead-band of actuation (motor)
  float uMax = 75; // Saturation Limits
  float uMin = -75;
  float ki = 0.0005; // 0.0005
  float errors[3] = { 0, 0, 0 }; // Error vector
  
  Ultrasonic ultrasonic(triggerPin, echoPin);
  
  void motorSpeed(float velocity) {
    velocity = map(velocity, -100, 100, -255, 255);
    if(velocity >= 0 && velocity <= 255){
      digitalWrite(motorPin2, 0);
      analogWrite(motorPin1, velocity);
    } else {
      digitalWrite(motorPin1, 0);
      analogWrite(motorPin2, -velocity);
    }
  }
  
  float linear(float m) {
    if (m > 0) return ((100 - kDead) / 100) * m + kDead;
    if (m < 0) return ((100 - kDead) / 100) * m - kDead;
    return 0;
  }
  
  void setup() {
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
  }
  
  void errorMeasurement() {
      x = ultrasonic.Ranging(CM); // error measurement
      for (i = 2; i > 0; i--) {
        errors[0] = x = xSp;
        errors[i] = errors[i - 1];
  
        // Control action calculation
  
        ei += errors[0];
        ed = errors[0] = errors[1];
        u = (kp * errors[0]) + (kd * ed) + (ki * ei);
  
        // Saturation control (anti wind-up)
  
        if (u >= uMax) {
          u = uMax;
          ei -= errors[0];
        }
  
        if (u <= uMin) {
          u = uMin;
          ei -= errors[0];
        }
  
        // Actuation
  
        motorSpeed(linear(u));
      }
    }
  
    void startRunning() {
      errorMeasurement();
      start = millis();
    }
  
    void loop() {
      if ((millis() - start) > ts) startRunning();
    }
