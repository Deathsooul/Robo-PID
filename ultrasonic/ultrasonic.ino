  #include <Ultrasonic.h>
  
  #define ECHO_PIN A4
  #define MOTOR_PIN_1 0
  #define MOTOR_PIN_2 1
  #define TRIGGER_PIN A5

  #define PIN_ENABLE 7

  #define BIT_MOTOR_1_A 2
  #define BIT_MOTOR_1_B 3
  #define BIT_MOTOR_2_A 1
  #define BIT_MOTOR_2_B 4
  #define BIT_MOTOR_3_A 5
  #define BIT_MOTOR_3_B 7
  #define BIT_MOTOR_4_A 0
  #define BIT_MOTOR_4_B 6
  
  #define PIN_MOTOR_3_PWM 5
  #define PIN_MOTOR_2_PWM 3
  #define PIN_MOTOR_4_PWM 6
  #define PIN_MOTOR_1_PWM 11

  #define MOTOR_VELOCITY_0 0
  #define MOTOR_VELOCITY_1 30
  #define MOTOR_VELOCITY_2 130
  #define MOTOR_VELOCITY_3 190
  #define MOTOR_VELOCITY_4 255
  
  unsigned int i;
  
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
  
  Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);

  void setup() {
    Serial.begin(9600);
    
    pinMode(PIN_ENABLE, OUTPUT);

    pinMode(PIN_MOTOR_1_PWM, OUTPUT);
    pinMode(PIN_MOTOR_2_PWM, OUTPUT);
    pinMode(PIN_MOTOR_3_PWM, OUTPUT);
    pinMode(PIN_MOTOR_4_PWM, OUTPUT);

    digitalWrite(PIN_ENABLE, LOW);

    digitalWrite(BIT_MOTOR_1_B, LOW);
    digitalWrite(BIT_MOTOR_1_A, HIGH);
  
    digitalWrite(BIT_MOTOR_2_B, LOW);
    digitalWrite(BIT_MOTOR_2_A, HIGH);
  
    digitalWrite(BIT_MOTOR_3_B, LOW);
    digitalWrite(BIT_MOTOR_3_A, HIGH);
  
    digitalWrite(BIT_MOTOR_4_B, LOW);
    digitalWrite(BIT_MOTOR_4_A, HIGH);
  
    setMotorSpeed(PIN_MOTOR_1_PWM, MOTOR_VELOCITY_3);
    setMotorSpeed(PIN_MOTOR_2_PWM, MOTOR_VELOCITY_3);
    setMotorSpeed(PIN_MOTOR_2_PWM, MOTOR_VELOCITY_3);
    setMotorSpeed(PIN_MOTOR_4_PWM, MOTOR_VELOCITY_3);
  }

  void setMotorSpeed(unsigned int pinPwmToUpdate, int speedValue) {
    analogWrite(pinPwmToUpdate, speedValue);
  }

  
  void motorSpeed(float velocity) {
    velocity = map(velocity, -100, 100, -255, 255);
    if (velocity >= 0 && velocity <= 255) {
      setMotorSpeed(PIN_MOTOR_2_PWM, MOTOR_VELOCITY_3);
      setMotorSpeed(PIN_MOTOR_1_PWM, MOTOR_VELOCITY_3);
    } else {
      setMotorSpeed(PIN_MOTOR_1_PWM, 0);
      setMotorSpeed(PIN_MOTOR_2_PWM, 0);
    }
  }
  
  float linear(float m) {
    Serial.println(m);
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
