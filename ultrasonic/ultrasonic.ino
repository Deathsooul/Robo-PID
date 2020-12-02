  #include<AFMotor.h>
  #include <Ultrasonic.h>
  
  #define ECHO_PIN A4
  #define TRIGGER_PIN A5

  #define MOTOR_VELOCITY_0 0
  #define MOTOR_VELOCITY_1 20
  #define MOTOR_VELOCITY_2 130
  #define MOTOR_VELOCITY_3 170
  #define MOTOR_VELOCITY_4 200

  #define PIN_ENABLE 7
  
  unsigned int i;

  unsigned long ts = 30; // Sampling Period
  unsigned long start = 0;
  
  float x; // Measured variable (process variable)
  float u; // Control Action (controlled variable)
  float ei = 0;
  float ed = 0;
  float kp = 1.2;
  float kd = 1.2;
  float xSp = 15; // Setpoint in centimeters
  float kDead = 7; // Dead-band of actuation (motor)
  float uMax = 75; // Saturation Limits
  float uMin = -75;
  float ki = 0.0005;
  float errors[3] = { 0, 0, 0 }; // Error vector

  float maxVelocity = MOTOR_VELOCITY_3;

  AF_DCMotor motor(4);
  
  Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);

  void setup() {
    Serial.begin(9600);
    
    pinMode(PIN_ENABLE, OUTPUT);

    digitalWrite(PIN_ENABLE, LOW);

    motor.setSpeed(MOTOR_VELOCITY_3);
    
    motor.run(RELEASE);
  }
  
  void motorSpeed(float velocity) {    
    velocity = map(velocity, -100, 100, -255, 255);

    float velocityCorrection = velocity > 0 ? velocity : -velocity;

    if (velocityCorrection > maxVelocity) velocityCorrection = maxVelocity;

    motor.setSpeed(velocityCorrection);

    if (velocity >= 0 && velocity <= 255) {
      motor.run(FORWARD);
    } else {
      motor.run(BACKWARD);
    }
  }
  
  float linear(float m) {
    if (m > 0) return ((100 - kDead) / 100) * m + kDead;
    if (m < 0) return ((100 - kDead) / 100) * m - kDead;
    return 0;
  }
  
  void errorMeasurement() {
    x = ultrasonic.Ranging(CM); // error measurement
    for (i = 2; i > 0; i--) {
      errors[0] = x - xSp;
      errors[i] = errors[i - 1];
  
      // Control action calculation
  
      ei += errors[0];
      ed = errors[0] - errors[1];
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
  
  void startRunningMotor() {
    errorMeasurement();
    start = millis();
  }
  
  void loop() {
    if ((millis() - start) > ts) startRunningMotor();
  }
