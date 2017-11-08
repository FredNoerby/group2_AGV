
#include <limits.h>
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_TB6612.h>
#include <SparkFunLSM9DS1.h>
//#include <MahonyAHRS.h>

const float DEG2RAD = M_PI/180.0f;
const float OFFSET = 4.8f * DEG2RAD;
const float TICKS2RAD = (2 * M_PI)/900.0f;
const unsigned long BAUD_RATE = 57600u;
const uint8_t AIN1 = 32u;
const uint8_t AIN2 = 30u;
const uint8_t PWMA = 11u;
const uint8_t BIN1 = 38u;
const uint8_t BIN2 = 36u;
const uint8_t PWMB = 12u;
const uint8_t STBY = 34u;
const uint8_t ENCODER0A = 3u;
const uint8_t ENCODER0B = 2u;
const uint8_t ENCODER1A = 18u;
const uint8_t ENCODER1B = 19u;
const uint8_t LSM9DS1_M = 0x1E;
const uint8_t LSM9DS1_AG = 0x6B;
const int OFFSET_A = 1;
const int OFFSET_B = -1;
const float LOOP_FREQUENCY_HZ = 119.0f;
const unsigned long LOOP_TIME_US = (unsigned long)(1.0f/LOOP_FREQUENCY_HZ * 1e6);
const float LOOP_TIME = (float)(LOOP_TIME_US * 1e-6);
const int FEEDFORWARD_PWM = 8; //8

class Rate {
public:
  Rate() { }
  ~Rate() { }

  void begin(unsigned long loop_time_us) {
    loop_time_us_ = loop_time_us;
    current_time_us_ = micros();
    previous_time_us_ = current_time_us_;
  }

  void sleep(void) {
    //if (!waiting())
      //Serial.println("loop time exceeded!");

    while (waiting())
      ;

    previous_time_us_ = current_time_us_;
  }

private:
  unsigned long loop_time_us_;
  unsigned long current_time_us_;
  unsigned long previous_time_us_;
  bool waiting_;

  bool waiting(void) {
    current_time_us_ = micros();
    return current_time_us_ < previous_time_us_ + loop_time_us_
      || current_time_us_ - previous_time_us_ > ULONG_MAX - loop_time_us_;
  }
};

class LowPassFilter {
public:
  LowPassFilter() {
    a_ = 0.0f;
    x_ = 0.0f;
  }
  ~LowPassFilter() { }

  void begin(float filter_constant) {
    a_ = filter_constant;
    x_ = 0.0f;
  }

  void update(float input) {
    x_ = a_ * x_ + (1.0f - a_) * input;
  }

  float getValue(void) {return x_;}

private:
  float a_;
  float x_;
};

class Differentiator {
public:
  Differentiator() {
    loop_time_ = 1.0f;
    u_ = 0.0f;
  }
  ~Differentiator() { }

  void begin(float loop_time, float filter_constant) {
    loop_time_ = loop_time;
    lpf_.begin(filter_constant);
    u_ = 0.0f;
  }

  void update(float input) {
    lpf_.update((input - u_)/loop_time_);
    u_ = input;
  }

  float getValue(void) {return lpf_.getValue();}

private:
  float loop_time_;
  LowPassFilter lpf_;
  float u_;
};

class Encoder {
public:
  Encoder() { }
  ~Encoder() { }

  void begin(uint8_t pin_a, uint8_t pin_b, float ticks2rad) {
    pin_a_ = pin_a;
    pin_b_ = pin_b;
    ticks2rad_ = ticks2rad;
    position_ = 0u;
    pinMode(pin_a_, INPUT);
    pinMode(pin_b_, INPUT);
  }

  uint8_t getPinA(void) {return pin_a_;}
  uint8_t getPinB(void) {return pin_b_;}
  float getPosition(void) {return (float)(position_) * ticks2rad_;}

  void onBChange(void) {
    digitalRead(pin_a_)^digitalRead(pin_b_) ? position_++ : position_--;
  }

private:
  uint8_t pin_a_;
  uint8_t pin_b_;
  int16_t position_;
  float ticks2rad_;
};

class DualEncoders {
public:
  DualEncoders() { }
  ~DualEncoders() { }

  // Callbacks
  static void encoder0BCallback(void);
  static void encoder1BCallback(void);

  void begin(uint8_t encoder0a, uint8_t encoder0b,
             uint8_t encoder1a, uint8_t encoder1b,
             float ticks2rad) {
    e0_.begin(encoder0a, encoder0b, ticks2rad);
    e1_.begin(encoder1a, encoder1b, ticks2rad);
    attachInterrupt(digitalPinToInterrupt(e0_.getPinB()), encoder0BCallback, CHANGE);
    attachInterrupt(digitalPinToInterrupt(e1_.getPinB()), encoder1BCallback, CHANGE);
  }

  float getEncoder0(void) {return -e0_.getPosition();}
  float getEncoder1(void) {return e1_.getPosition();}

private:
  static Encoder e0_;
  static Encoder e1_;
};

Encoder DualEncoders::e0_;
Encoder DualEncoders::e1_;
void DualEncoders::encoder0BCallback(void) {e0_.onBChange();}
void DualEncoders::encoder1BCallback(void) {e1_.onBChange();}

class Imu {
public:
  Imu() { }
  ~Imu() { }

  float cx, cy, cz, gx, gy, gz, ax, ay, az;

  void begin(void) {
    lsm9ds1_.settings.device.commInterface = IMU_MODE_I2C;
    lsm9ds1_.settings.device.mAddress = LSM9DS1_M;
    lsm9ds1_.settings.device.agAddress = LSM9DS1_AG;
    lsm9ds1_.settings.gyro.sampleRate = 3;

    cx = 1.0f;
    cy = 1.0f;
    cz = 1.0f;

    if (!lsm9ds1_.begin())
    {
      Serial.println("IMU initialization failed!");
      while (1)
        ;
    }
  }

  void update(void) {
    if (lsm9ds1_.gyroAvailable())
      lsm9ds1_.readGyro();
    if (lsm9ds1_.accelAvailable())
      lsm9ds1_.readAccel();
    if (lsm9ds1_.magAvailable())
      lsm9ds1_.readMag();
    gx = cx * lsm9ds1_.calcGyro(lsm9ds1_.gx);
    gy = -cy * lsm9ds1_.calcGyro(lsm9ds1_.gy);
    gz = cz * lsm9ds1_.calcGyro(lsm9ds1_.gz);
    ax = lsm9ds1_.calcAccel(lsm9ds1_.ax);
    ay = -lsm9ds1_.calcAccel(lsm9ds1_.ay);
    az = lsm9ds1_.calcAccel(lsm9ds1_.az);
  }

private:
  LSM9DS1 lsm9ds1_;
};

void feedforwardDrive(Motor* motor, int value) {
  value = constrain(value, -150.0f, 150.0f);
  if (value > 0)
    motor->drive(value + FEEDFORWARD_PWM);
  else if (value < 0)
    motor->drive(value - FEEDFORWARD_PWM);
  else
    motor->drive(value);
}

void fixMotorNoise(void) {
  TCCR0B &= ~0x07;
  TCCR0B |= 0x01;
  TCCR2B &= ~0x07;
  TCCR2B |= 0x01;
}

float lqr_state_feedback(float x1, float x2, float x3, float x4) {
  return 759.7324*x1 + 89.5371*x2 - 3.1623*x3 - 15.1751*x4;
}


// The PID controller where we define k_p, k_d. Here we calculate our PWM
// Takes in a position, velocity, and a goal
float pid_controller(float velocity, float posit, float goal) {
  float k_p = 100;
  float k_d = 0;

// The way the formula works it that the potortunal gain is multiplyed by the difrents in the desired position of the wheel minus the current possition of the wheel.
// This is then substrated by the differentiated gain multiplyed by the angula vilocity 
// The motors already have a lot of internal dampening comming from the gears and so on.
  return k_p*(goal-posit)-k_d*velocity;
}

// Objects
Rate rate;
Motor motor0 = Motor(BIN1, BIN2, PWMB, OFFSET_B, STBY);
Motor motor1 = Motor(AIN1, AIN2, PWMA, OFFSET_A, STBY);
DualEncoders encoders;
Differentiator tachometer0;
Differentiator tachometer1;
LowPassFilter output_lpf;

void setup() {
  //fixMotorNoise();
  Serial.begin(BAUD_RATE);
  encoders.begin(ENCODER0A, ENCODER0B, ENCODER1A, ENCODER1B, TICKS2RAD);
  rate.begin(LOOP_TIME_US);
  tachometer0.begin(LOOP_TIME, 0.7f);
  tachometer1.begin(LOOP_TIME, 0.7f);
  output_lpf.begin(0.95f);
}

// Initializes the pwm variable for the right wheel. PWM stands for puls with modulation and it has an interval of 0 to 255
int pwm = 0;

// Initializes the pwm for the left wheel
int pwm2 = 0;

// We initialize an integer used for counting how long to drive the motors 
int counter = 0;

// We intialize a variable for the goal
float goal = 0.0;

void loop() {
  // Estimation
  tachometer0.update(encoders.getEncoder0());
  tachometer1.update(encoders.getEncoder1());
  // Control
  output_lpf.update(0.0);

  // Increments the counter if it is below 100 else it sets it to zero
  if (counter < 100) {
    counter ++;  
  } else {
    counter = 0;
  }

  // Changes the goal from positive to negative depending on the counter
  // This makes the motors spin one way when the counter is below 50 and the other way when it is above
  if (counter < 50) {
    goal = -1;
  } else {
    goal = 1;
  }

  // The pwm's are calculated using the PID controller function
  // It takes in the motors rotations per second in as velocity and the encoder values as positions
  pwm = pid_controller(tachometer1.getValue(), encoders.getEncoder1(), goal);
  pwm2 = pid_controller(tachometer0.getValue(), encoders.getEncoder0(), goal);

  // The feed forward drive method is used to drive the two motors with the calculated PWM
  feedforwardDrive(&motor0, pwm);
  feedforwardDrive(&motor1, pwm2);

  // We set it to print some of the readings so we know can check if it is correct. 
  Serial.print("Left position: ");
  Serial.print(encoders.getEncoder0());
  Serial.print(" Left angular velocity: ");
  Serial.print(tachometer0.getValue());
  Serial.print(' ');
  Serial.print("PWM: ");
  Serial.print(pwm2);
  Serial.print("  Right position: ");
  Serial.print(encoders.getEncoder1());
  Serial.print(" Right angular velocity: ");
  Serial.print(tachometer1.getValue());
  Serial.print(' ');
  Serial.print("PWM: ");
  Serial.print(pwm);
  Serial.println();

  // The loop sleeps for the rate amount of time (around 1/10th of a second)
  rate.sleep();
}

