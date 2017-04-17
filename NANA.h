int left_ticks = 0;
int prev_left_ticks = 0;
int right_ticks = 0;
int prev_right_ticks = 0;

int left_pwm = 0;
int right_pwm = 0;

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

class NANA {
  public:
    bool active = false;

    const int MPU_addr = 0x68;

    const int wheelDistance = 270;
    const float distancePerCount = 0.22;

    static const int map_size = 50;
    //mm
    int map_res = wheelDistance;

    //-1 - obstacle
    //0 - wasnt here
    //1 - was here
    //x,y
    short room_map[map_size][map_size];

    float theta = 0;
    float theta_d = 0;

    int x_on_map = map_size / 2;
    int y_on_map = map_size / 2;

    float x = x_on_map * map_res;
    float y = y_on_map * map_res;

    int x_goal = false;
    int y_goal = false;

    int x_pred = x_on_map;
    int y_pred = y_on_map;

    //Бампер
    const int bumper_pin_right = 16;
    const int bumper_pin_left = 14;

    const int left_tick_pin = 13;

    const int right_tick_pin = 12;

    void loop() {

      updateOdometry();

      sendDebug();

      bumper_read();

      //только после нажатия на бампер
      if (active) {
        echo("active");

        if ((x_on_map == x_goal && y_on_map == y_goal)) {
          selectNewGoal();
        }

        makeControl();
      }

    }

    void sendDebug() {
      int voltage = analogRead(A0);

      String debug = "{\n";
      debug += "\"t\": " + String(millis()) + ",\n";
      debug += "\"right_ticks\": " + String(right_ticks) + ",\n";
      debug += "\"left_ticks\": " + String(left_ticks) + ",\n";
      debug += "\"left_pwm\": " + String(left_pwm) + ",\n";
      debug += "\"right_pwm\": " + String(right_pwm) + ",\n";
      debug += "\"theta\": " + String(theta) + ",\n";
      debug += "\"theta_d\": " + String(theta_d) + ",\n";
      debug += "\"x\": " + String(x_on_map) + ",\n";
      debug += "\"y\": " + String(y_on_map) + ",\n";
      debug += "\"x_goal\": " + String(x_goal) + ",\n";
      debug += "\"y_goal\": " + String(y_goal) + ",\n";
      debug += "\"voltage\": " + String(voltage) + "\n";
      debug += "}";

      webSocket.broadcastTXT(debug);
    }

    void makeControl() {
      theta_d = atan2(y_goal * map_res - y, x_goal * map_res - x);

      float theta_error = theta_d - theta;

      theta_error = max(-2 * M_PI, theta_error);
      theta_error = min(2 * M_PI, theta_error);

      left_pwm = 4095;
      right_pwm = 4095;

      int k = 8190 / M_PI * theta_error;

      if (k > 0) {
        left_pwm -= k;
      } else if (k < 0) {
        right_pwm += k;
      }

      drive(left_pwm, right_pwm);

      //     TODO: left spinnint full back, right not spinnin why?
      //      drive(-64, 8254);
    }

    bool avoidObstacle() {
      float target_theta = theta + M_PI / 2;
      x_goal = round(x_on_map + cos(target_theta));
      y_goal = round(y_on_map + sin(target_theta));
    }

    void selectNewGoal() {
      x_goal = round(x_on_map + cos(theta));
      y_goal = round(y_on_map + sin(theta));
    }

    void updateOdometry() {
      float SR = distancePerCount * (right_ticks - prev_right_ticks);
      float SL = distancePerCount * (left_ticks - prev_left_ticks);

      prev_right_ticks = right_ticks;
      prev_left_ticks = left_ticks;

      float meanDistance = (SL + SR) / 2;

      x += meanDistance * cos(theta);
      y += meanDistance * sin(theta);
      theta += (SR - SL) / wheelDistance;

      theta = normalizeTheta(theta);

      x_on_map = (int) (x / map_res);
      y_on_map = (int) (y / map_res);

      //только, если не отмечено как препятствие
      if (room_map[x_on_map][y_on_map] >= 0)
        room_map[x_on_map][y_on_map] = 1;

      x_pred = ceil(x_on_map + cos(theta));
      y_pred = ceil(y_on_map + sin(theta));

      if (x_pred < 0 || y_pred < 0 || x_pred > map_size || y_pred > map_size) {
        bumper();
      }
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

      drive(0, 0);
    }

    bool bumper_read() {
      boolean bumber_right = digitalRead(bumper_pin_right) == HIGH;
      boolean bumber_left = digitalRead(bumper_pin_left) == HIGH;

      if (bumber_right || bumber_left) {
        //Начинает работать только после нажатия на бампер
        active = true;
        bumper();
      }

      return bumber_right || bumber_left;
    }

    void bumper() {
      if (!(x_pred < 0 || y_pred < 0 || x_pred > map_size || y_pred > map_size)) {
        room_map[x_pred][y_pred] = -1;
      }
      avoidObstacle();
    }

    void drive(int left, int right) {
      //Левое
      setMotorRotation(4, 3, left);

      //Правое
      setMotorRotation(2, 1, right);

      //Скорость
      pwm.setPWM(0, 0, max(4095, abs(left)));

      pwm.setPWM(5, 0, max(4095, abs(right)));

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
