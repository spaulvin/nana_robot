int left_ticks = 0;
int prev_left_ticks = 0;
int right_ticks = 0;
int prev_right_ticks = 0;


int left_pwm_current = 0;
int right_pwm_current = 0;

void right_tick() {
  if (right_pwm_current >= 0 ) {
    right_ticks++;
  } else {
    right_ticks--;
  }

}

void left_tick() {
  if (left_pwm_current >= 0 ) {
    left_ticks++;
  } else {
    left_ticks--;
  }
}

class NANA
{
  public:
    const int MPU_addr = 0x68;

    int moving_mode = 0;
    int switch_moving_mode_at = 0;

    const int WAITING = 0;
    const int SPIRAL = 1;
    const int MOWER = 2;
    const int WALL = 3;

    const int wheelDistance = 270;
    const float distancePerCount = 0.22;

    float theta = 0;

    float x = 0;
    float y = 0;

    //Время, за которое робот поворачивается на PI (180градусов)
    const int PI_TIME = 1700;
    //Время движения назад после срабатывания бампера
    const int BACK_TIME = 300;

    //Запланированое изменение скорости(направления) двигателей
    int update_pwm_at = 0;
    int left_pwm = 0;
    int right_pwm = 0;

    //Бампер
    const int bumper_pin_right = 16;
    const int bumper_pin_left = 14;

    const int left_tick_pin = 13;

    const int right_tick_pin = 12;

    int turn_counter = 0;

    int speed = 0;

    void loop() {

      updateOdometry();

      int voltage =   analogRead(A0);

      String debug = "{\n";
      debug += "\"t\": " + String(millis()) + ",\n";
      debug += "\"right_ticks\": " + String(right_ticks) + ",\n";
      debug += "\"left_ticks\": " + String(left_ticks) + ",\n";
      debug += "\"theta\": " + String(theta) + ",\n";
      debug += "\"x\": " + String(x) + ",\n";
      debug += "\"y\": " + String(y) + ",\n";
      debug += "\"voltage\": " + String(voltage) + ",\n";
      debug += "\"speed\": " + String(speed) + "\n";
      debug += "}";


      webSocket.broadcastTXT(debug);

      //Дальше только автоматические режимы
      if (!moving_mode) {
        return;
      }

      bumper_read();

      //Плановое изменение режима
      if (switch_moving_mode_at && switch_moving_mode_at < millis()) {
        setDriveMode(moving_mode++ % 3);
      }

      if (update_pwm_at && update_pwm_at < millis() ) {
        drive(left_pwm, right_pwm);
        //Запланировать следующее
        switch (moving_mode) {
          case 1:
            if (turn_counter % 2) {
              left_pwm = -4096;
              right_pwm = 4096;
              update_pwm_at = millis() + floor(turn_counter / 4) * 1000;
            } else {
              left_pwm = 4096;
              right_pwm = 4096;
              update_pwm_at = millis() + PI_TIME / 2;
            }
            turn_counter++;

            break;
          case 2:
            mower();

            break;
          case 3:
            if (left_pwm != right_pwm) {
              //              Выполняем поворот
              //  Планируем ехать прямо до препятствия
              left_pwm = 4096;
              right_pwm = 4096;
              update_pwm_at = millis() + BACK_TIME;
            }
            break;
        }
      }
    }

    void updateOdometry()
    {
      float SR = distancePerCount * (right_ticks - prev_right_ticks);
      float SL = distancePerCount * (left_ticks - prev_left_ticks);

      prev_right_ticks = right_ticks;
      prev_left_ticks = left_ticks;

      float meanDistance = (SL + SR)/2;

      x += meanDistance * cos(theta);
      y += meanDistance * sin(theta);
      theta += (SR - SL) / wheelDistance;

      if (theta > 2 * M_PI)
        theta -= 2 * M_PI;
      else if (theta < -2 * M_PI)
        theta += 2 * M_PI;
    }

    void mower(boolean bumper = false) {
      //Выполняется попорот. Запланировать ехать прямо
      if ((left_pwm > 0 || right_pwm > 0) && left_pwm != right_pwm) {
        left_pwm = 4096;
        right_pwm = 4096;
        update_pwm_at = millis() + PI_TIME / 2;
      } else {
        //Едем прямо, запланировать поворот
        switch (turn_counter % 4) {
          case 0:
            //Вправо на 90
            left_pwm = 4096;
            right_pwm = -4096;
            update_pwm_at = millis() + 60000;
            break;
          case 1:
            //Вправо на 90
            left_pwm = 4096;
            right_pwm = -4096;
            update_pwm_at = millis() + PI_TIME;
            break;
          case 2:
            //Влево на 90
            left_pwm = -4096;
            right_pwm = 4096;
            update_pwm_at = millis() + 60000;
            break;
          case 3:
            //Влево на 90
            left_pwm = -4096;
            right_pwm = 4096;
            update_pwm_at = millis() + PI_TIME;
            break;
        }
        if (bumper) {
          update_pwm_at = millis() + BACK_TIME;
        } else {
          turn_counter++;
        }
      }
    }

    void setDriveMode(int m) {
      turn_counter = 0;
      switch (m) {
        case 1:
          left_pwm = 4096;
          right_pwm = 4096;

          drive(left_pwm, right_pwm);

          update_pwm_at = millis() + 1000;

          moving_mode = SPIRAL;

          switch_moving_mode_at = millis() + 60000;

          break;
        case 2:
          left_pwm = -4096;
          right_pwm = -4096;
          drive(left_pwm, right_pwm);
          moving_mode = MOWER;

          //Через секунд делаем левый поворот - начало
          left_pwm = -4096;
          right_pwm = 4096;
          update_pwm_at = millis() + BACK_TIME;

          switch_moving_mode_at = millis() + 300000;
          break;
        case 3:
          left_pwm = 4096;
          right_pwm = 4096;
          drive(left_pwm, right_pwm);
          moving_mode = WALL;

          switch_moving_mode_at = millis() + 10000;
          break;
        default:
          drive(0, 0);
          moving_mode = WAITING;

          switch_moving_mode_at = 0;
      }
    }

    void setup() {
      pinMode(bumper_pin_right, INPUT);
      pinMode(bumper_pin_left, INPUT);

      pinMode(left_tick_pin, INPUT);
      pinMode(right_tick_pin, INPUT);

      attachInterrupt(right_tick_pin, right_tick, FALLING);
      attachInterrupt(left_tick_pin, left_tick, FALLING);
    }

    boolean bumper_read() {
      boolean bumber_right = digitalRead(bumper_pin_right) == HIGH;
      boolean bumber_left = digitalRead(bumper_pin_left) == HIGH;

      if (bumber_right || bumber_left) {
        switch (moving_mode) {
          case 0:
            break;
          case 1:
            setDriveMode(MOWER);
            break;
          case 2:
            left_pwm = -4096;
            right_pwm = -4096;
            drive(left_pwm, right_pwm);
            mower(true);
            break;
          case 3:
            //          Едем назад
            left_pwm = -4096;
            right_pwm = -4096;
            drive(left_pwm, right_pwm);

            if (bumber_right) {
              left_pwm = -4096;
              right_pwm = 4096;
            } else {
              left_pwm = 4096;
              right_pwm = -4096;
            }

            update_pwm_at = millis() + BACK_TIME;
            break;
        }
      }


      return bumber_right || bumber_left;
    }

    void drive(int left, int right) {
      left_pwm_current = left;
      right_pwm_current = right;
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
