#include <Arduino.h>
#include <GyverMotor/GyverMotor.h>
#include <Arduino-MPU6050-master/MPU6050.h>

#define Mot1 7
#define Mot2 8
#define Mot3 9

#define Mot4 4
#define Mot5 5
#define Mot6 6

#define MinSpeed 80
#define MaxSpeed 255

#define k_p 0.3

struct Sonar
{
private:
  byte trig;
  byte echo;

public:
  Sonar(byte trig_pin, byte echo_pin) : trig(trig_pin), echo(echo_pin)
  {
    pinMode(trig_pin, OUTPUT);
    pinMode(echo_pin, INPUT);
  };

  float getDistance()
  {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    int duration = pulseIn(echo, HIGH);
    return float(duration) * 0.034 / 2.0;
  }
};

class Robot
{
private:
  float volatile yaw = 0;

  unsigned long timer = 0;
  const float timeStep = 0.1;

  const float coef[2] = {1.0, 0.8};

public:
  MPU6050 &mpu;
  GMotor &motor1;
  GMotor &motor2;
  Sonar &sonar;

  Robot(GMotor &motor1, GMotor &motor2, MPU6050 &mpu, Sonar &sonar) : motor1(motor1), motor2(motor2), mpu(mpu), sonar(sonar)
  {
    this->mpu = mpu;
    this->motor1.setSpeed(0);
    this->motor2.setSpeed(0);
    this->mpu.begin();
    this->mpu.calibrateGyro();
    this->mpu.setThreshold(3);
  };

  void turn(int degrees)
  {
    int point = yaw + degrees;
    int err = point - yaw;
    int speed = 0;

    while (abs(err) > 5.0)
    {
      err = point - yaw;
      speed = constrain(k_p * abs(err), 0, MaxSpeed - MinSpeed);
      motor1.setSpeed((MinSpeed + speed) * (err / abs(err)) * coef[1]);
      motor2.setSpeed(-(MinSpeed + speed) * (err / abs(err)) * coef[0]);

      Serial.print(" Speed = ");
      Serial.print(speed);

      Serial.println();
      Vector norm = mpu.readNormalizeGyro();
      yaw = yaw + norm.ZAxis * (millis() - timer) / 1000;
      // delay((timeStep*1000) - (millis() - timer));
      timer = millis();
    }

    motor1.setSpeed(0);
    motor2.setSpeed(0);
  };

  void dash()
  {
    motor1.setSpeed((MinSpeed + 50) * coef[1]);
    motor2.setSpeed(-(MinSpeed + 50) * coef[0]);
    delay(20);
    motor1.setSpeed(-(MinSpeed + 50) * coef[1]);
    motor2.setSpeed((MinSpeed + 50) * coef[0]);
    delay(20);
    motor1.setSpeed(0);
    motor2.setSpeed(0);
  };

  void keepPosition()
  {
    int point = yaw;
    int err = 0;
    int speed = 0;
    while (true)
    {
      err = point - yaw;
      speed = constrain(k_p * abs(err), 0, MaxSpeed - MinSpeed);
      if (abs(err) > 5.0)
      {
        motor1.setSpeed((MinSpeed + speed) * (err / abs(err)) * coef[1]);
        motor2.setSpeed(-(MinSpeed + speed) * (err / abs(err)) * coef[0]);
      }
      else
      {
        motor1.setSpeed(0);
        motor2.setSpeed(0);
      }

      Vector norm = mpu.readNormalizeGyro();
      yaw = yaw + norm.ZAxis * (millis() - timer) / 1000;
      timer = millis();
    }
  };

  void keepDistance(int distance)
  {
    int point = distance;
    int err = 0;
    int speed = 0;
    while (true)
    {
      err = sonar.getDistance() - point;
      speed = constrain(k_p * abs(err), 0, MaxSpeed - MinSpeed);
      if (abs(err) > 2)
      {
        // dash();
        motor1.setSpeed((MinSpeed + speed) * (err / abs(err)) * coef[1]);
        motor2.setSpeed((MinSpeed + speed) * (err / abs(err)) * coef[0]);
      }
      else
      {
        motor1.setSpeed(0);
        motor2.setSpeed(0);
      }
      delay(50);
    }
  }
};

MPU6050 mpu_;
GMotor mot1(DRIVER3WIRE, Mot1, Mot2, Mot3, LOW);
GMotor mot2(DRIVER3WIRE, Mot4, Mot5, Mot6, LOW);
Sonar son(10, 11);
Robot robot(mot1, mot2, mpu_, son);

void setup()
{
  Serial.begin(115200);
}

void task1()
{
  delay(1000);
  robot.turn(-90);
  delay(1000);
  robot.turn(90);
  delay(1000);
  robot.turn(180);
  delay(1000);
  robot.turn(180);
  delay(1000);
  robot.turn(45);
  delay(1000);
  robot.turn(-45);
};

void task2()
{
  robot.keepPosition();
};

void task3(){
  robot.keepDistance(20);
};

void loop()
{
}