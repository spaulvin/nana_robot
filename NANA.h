#include "PID_v1.h"
#include "user_interface.h"
int left_ticks = 0;
int prev_left_ticks = 0;
int right_ticks = 0;
int prev_right_ticks = 0;

int left_pwm = 0;
int right_pwm = 0;
//Всегда стремимся к 0 градусов
double thSet = 0;
double thOut = 0;

//Угол отклонения от цели [-pi,pi]
double theta = 0;

//Скорость [-4095,4095]
int speed = 0;

int ti = 0;           // tic timer counter

double kp = 0.5, ki = 10, kd = 0.0;
//PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
PID thPID(&theta, &thOut, &thSet, kp, ki, kd, DIRECT);

static os_timer_t myTimer;
int period = 50;

#define min(X, Y) (((X)<(Y))?(X):(Y))
#define max(X, Y) (((X)>(Y))?(X):(Y))

void right_tick() {
  if (right_pwm >= 0) {
    right_ticks++;
  } else {
    right_ticks--;
  }

}

void left_tick() {
  if (left_pwm >= 0) {
    left_ticks++;
  } else {
    left_ticks--;
  }
}


void tic(void *pArg) {
  thPID.Compute();

  left_pwm = speed;
  right_pwm = speed;

  left_pwm -= thOut;
  right_pwm += thOut;

  ti += 1;                // tic counter
}

class NANA {
  public:
    bool active = false;

    const int wheelDistance = 270;
    const float distancePerCount = 0.22;

    //Общая пройденая дистанция, независимо от направления
    float distance = 0;
    float last_distance = distance;

    int next_distance = -1;
    bool update_on_angle = false;

    //Бампер
    const int bumper_pin_right = 16;
    const int bumper_pin_left = 14;

    const int left_tick_pin = 13;

    const int right_tick_pin = 12;

    int bumberCounter = 0;

    bool prevBumperPressed = false;

    void loop() {

      updateOdometry();

      sendDebug();

      bumper_read();

      //только после нажатия на бампер
      if (active) {
        echo("active");

        //Включить щетки
        pwm.setPWM(6, 4096, 0);

        if ((!update_on_angle && next_distance > 0 && next_distance <= distance) || (update_on_angle && theta < 0.1 && theta > -0.1)) {
          mower();
        }

        drive(left_pwm,right_pwm);

      } else {
        pwm.setPWM(6, 0, 4096);
      }

    }

    void mower(boolean bumper = false) {
      if (bumper) {
        speed = -4096;
        next_distance = distance + wheelDistance / 4;
        update_on_angle = false;
      } else {
        if (update_on_angle) {
          speed = 4096;
          update_on_angle = false;
          next_distance = distance + 20 * wheelDistance;
        } else {
          theta = random(M_PI * 1000) / 1000.0;
          theta = random(10) <= 3 ? -theta : theta;
          speed = 0;
          update_on_angle = true;
          next_distance = -1;
        }


        last_distance = distance;
      }
    }

    void sendDebug() {
      int voltage = analogRead(A0);

      String debug = "{\n";
      debug += "\"type\": \"debug\",\n";
      debug += "\"t\": " + String(millis()) + ",\n";
      debug += "\"distance\": " + String(distance) + ",\n";
      debug += "\"left_pwm\": " + String(left_pwm) + ",\n";
      debug += "\"right_pwm\": " + String(right_pwm) + ",\n";
      debug += "\"theta\": " + String(theta) + ",\n";
      debug += "\"voltage\": " + String(voltage) + "\n";
      debug += "}";

      webSocket.broadcastTXT(debug);
    }

    void consoleLog(String message) {
      String debug = "{\n";
      debug += "\"type\": \"consoleLog\" ,\n";
      debug += "\"message\": \"" + String(message) + "\"\n";
      debug += "}";

      webSocket.broadcastTXT(debug);
    }

    bool avoidObstacle() {
      consoleLog("avoidObstacle" + String(bumberCounter));
      mower(true);
    }

    void updateOdometry() {
      float SR = distancePerCount * (right_ticks - prev_right_ticks);
      float SL = distancePerCount * (left_ticks - prev_left_ticks);

      prev_right_ticks = right_ticks;
      prev_left_ticks = left_ticks;

      float meanDistance = (SL + SR) / 2;
      distance += abs(meanDistance);

      theta += (SR - SL) / wheelDistance;

      theta = normalizeTheta(theta);
    }

    float normalizeTheta(float th) {
      //      theta between -PI and PI (https://youtu.be/ky-sIvvQFXM?t=174)
      while (th > M_PI)
        th -= 2 * M_PI;
      while (th < -M_PI)
        th += 2 * M_PI;

      return th;
    }


    void setup() {
      pinMode(bumper_pin_right, INPUT);
      pinMode(bumper_pin_left, INPUT);

      pinMode(left_tick_pin, INPUT);
      pinMode(right_tick_pin, INPUT);

      attachInterrupt(right_tick_pin, right_tick, FALLING);
      attachInterrupt(left_tick_pin, left_tick, FALLING);

      sei();                                  // Enable interrupts
      os_timer_setfn(&myTimer, tic, NULL);
      os_timer_arm(&myTimer, period, true);   // timer in ms

      thPID.SetSampleTime(period);
      thPID.SetOutputLimits(-4096, 4096);
      thPID.SetMode(AUTOMATIC);

      drive(0, 0);
    }

    bool bumper_read() {
      boolean bumber_right = digitalRead(bumper_pin_right) == HIGH;
      boolean bumber_left = digitalRead(bumper_pin_left) == HIGH;

      if (bumber_right || bumber_left) {
        //Начинает работать только после нажатия на бампер
        if (!prevBumperPressed) {
          active = true;
          avoidObstacle();
        }

        prevBumperPressed = true;
      } else {
        prevBumperPressed = false;
      }

      return bumber_right || bumber_left;
    }

    void drive(int left, int right) {

      //Левое
      setMotorRotation(4, 3, left);

      //Правое
      setMotorRotation(2, 1, right);

      uint16_t l = min(4095, abs(left));
      uint16_t r = min(4095, abs(right));

      if (l > 0 && l < 3000) {
        l = 3000;
      }

      if (r > 0 && r < 3000) {
        r = 3000;
      }

      //Скорость
      pwm.setPWM(0, 0, r);

      pwm.setPWM(5, 0, l);
    }

    void setMotorRotation(uint8_t pinA, uint8_t pinB, int spd) {
      if (spd > 0) {
        pwm.setPWM(pinA, 4096, 0);
        pwm.setPWM(pinB, 0, 4096);
        return;
      }

      if (spd < 0) {
        pwm.setPWM(pinA, 0, 4096);
        pwm.setPWM(pinB, 4096, 0);
        return;
      }

      pwm.setPWM(pinA, 4096, 0);
      pwm.setPWM(pinB, 4096, 0);
    }

};
