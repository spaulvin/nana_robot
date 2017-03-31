int left_ticks = 0;
int prev_left_ticks = 0;
int right_ticks = 0;
int prev_right_ticks = 0;

int left_pwm = 0;
int right_pwm = 0;

#define min(X, Y) (((X)<(Y))?(X):(Y))
#define max(X, Y) (((X)>(Y))?(X):(Y))

void right_tick() {
  if (right_pwm >= 0 ) {
    right_ticks++;
  } else {
    right_ticks--;
  }

}

void left_tick() {
  if (left_pwm >= 0 ) {
    left_ticks++;
  } else {
    left_ticks--;
  }
}

class NANA
{
  public:
    bool active = false;

    const int MPU_addr = 0x68;

    const int wheelDistance = 270;
    const float distancePerCount = 0.22;

    static const  int map_size = 50;
    //mm
    int map_res = wheelDistance;

    //-1 - obstacle
    //0 - wasnt here
    //1 - was here
    //x,y
    short room_map[map_size][map_size];

    bool reach_map[map_size][map_size];

    float theta = 0;

    int x_on_map = map_size / 2;
    int y_on_map = map_size / 2;

    float x = x_on_map * map_res;
    float y = y_on_map * map_res;

    int x_goal = false;
    int y_goal = false;

    bool goal_selected = false;

    //Бампер
    const int bumper_pin_right = 16;
    const int bumper_pin_left = 14;

    const int left_tick_pin = 13;

    const int right_tick_pin = 12;

    void loop() {

      updateOdometry();

      x_on_map = (int)x / map_res;
      y_on_map = (int)y / map_res;

      //только, если не отмечено как препятствие
      if (room_map[x_on_map][y_on_map] >= 0)
        room_map[x_on_map][y_on_map] = 1;

      int voltage =   analogRead(A0);

      String debug = "{\n";
      debug += "\"t\": " + String(millis()) + ",\n";
      debug += "\"right_ticks\": " + String(right_ticks) + ",\n";
      debug += "\"left_ticks\": " + String(left_ticks) + ",\n";
      debug += "\"left_pwm\": " + String(left_pwm) + ",\n";
      debug += "\"right_pwm\": " + String(right_pwm) + ",\n";
      debug += "\"theta\": " + String(theta) + ",\n";
      debug += "\"x\": " + String(x_on_map) + ",\n";
      debug += "\"y\": " + String(y_on_map) + ",\n";
      debug += "\"x_goal\": " + String(x_goal) + ",\n";
      debug += "\"y_goal\": " + String(y_goal) + ",\n";
      debug += "\"voltage\": " + String(voltage) + "\n";
      debug += "}";

      webSocket.broadcastTXT(debug);

      bumper_read();

      //только после нажатия на бампер
      if (active) {
        echo("active");

        if (x_on_map == x_goal && y_on_map == y_goal) {
          traverse();
        }

        makeControl();
      }

    }

    void makeControl() {

      float theta_desired = atan2(y_goal - y_on_map, x_goal - x_on_map);

      float Distance = sqrt(pow(x_goal - x_on_map, 2) + pow(y_goal - y_on_map, 2));

      float theta_error = theta - theta_desired;

      theta_error = normalizeTheta(theta_error);

      left_pwm = (int)Distance * 2048;
      right_pwm = (int)Distance * 2048;

      left_pwm = min(left_pwm, 4096);
      left_pwm = max(left_pwm, 0);

      right_pwm = min(right_pwm, 4096);
      right_pwm = max(right_pwm, 0);

      if (left_pwm && right_pwm) {
        int k = 4096 / M_PI * theta_error;
        left_pwm += k;
        right_pwm -= k;
      }

      drive(left_pwm, right_pwm);
    }

    bool avoidObstacle() {
      float target_theta = theta + M_PI / 2;
      target_theta = normalizeTheta(target_theta);
      x_goal = floor(x_on_map + cos(target_theta));
      y_goal = floor(y_on_map + sin(target_theta));
    }

    void traverse() {
      float target_theta = theta;
      target_theta = normalizeTheta(target_theta);
      x_goal = floor(x_on_map + cos(target_theta));
      y_goal = floor(y_on_map + sin(target_theta));
    }

    void updateOdometry()
    {
      float SR = distancePerCount * (right_ticks - prev_right_ticks);
      float SL = distancePerCount * (left_ticks - prev_left_ticks);

      prev_right_ticks = right_ticks;
      prev_left_ticks = left_ticks;

      float meanDistance = (SL + SR) / 2;

      x += meanDistance * cos(theta);
      y += meanDistance * sin(theta);
      theta += (SR - SL) / wheelDistance;

      theta = normalizeTheta(theta);

      float x_pred = (x + map_res * cos(theta)) / map_res;
      float y_pred = (y + map_res * sin(theta)) / map_res;

      if ( x_pred < 0 || y_pred < 0 || x_pred > map_size || y_pred > map_size ) {
        bumper();
      }
    }

    float normalizeTheta(float th) {
      //      theta between 0 and 2PI
      while (th >= 2 * M_PI)
        th -= 2 * M_PI;
      while (th < 0)
        th +=  2 * M_PI;

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

    boolean bumper_read() {
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
      float x_pred = x_on_map + cos(theta);
      float y_pred = y_on_map + sin(theta);

      if ( !(x_pred < 0 || y_pred < 0 || x_pred > map_size || y_pred > map_size) ) {
        room_map[round(x_pred)][round(y_pred)] = -1;
        avoidObstacle();
      }

    }

    void drive(int left, int right) {
      left_pwm = left;
      right_pwm = right;
      //Левое
      if (left > 0) {
        pwm.setPWM(4, 4096, 0);
        pwm.setPWM(3, 0, 4096);
      } else if (left < 0) {
        pwm.setPWM(4, 0, 4096);
        pwm.setPWM(3, 4096, 0);
      } else {
        pwm.setPWM(4, 4096, 0);
        pwm.setPWM(3, 4096, 0);
      }
      //Правое
      if (right > 0) {
        pwm.setPWM(2, 4096, 0);
        pwm.setPWM(1, 0, 4096);
      } else if (right < 0) {
        pwm.setPWM(2, 0, 4096);
        pwm.setPWM(1, 4096, 0);
      } else {
        pwm.setPWM(2, 4096, 0);
        pwm.setPWM(1, 4096, 0);
      }

      left = abs(left);
      right = abs(right);
      if (left > 4096) {
        left = 4096;
      }
      if (right > 4096) {
        right = 4096;
      }
      if (left > 0 && left < 3000) {
        left = 3000;
      }

      if (right > 0 && right < 3000) {
        right = 3000;
      }

      //Скорость
      if (left == 4096) {
        pwm.setPWM(0, left, 0);
      } else {
        pwm.setPWM(0, 0, left);
      }

      if (right == 4096) {
        pwm.setPWM(5, right, 0);
      } else {
        pwm.setPWM(5, 0, right);
      }

    }

};
