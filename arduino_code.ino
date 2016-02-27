#include <Servo.h>

#define ECHO_PIN 2
#define TRIG_PIN 3

#define RANGE_MIN 20
#define RANGE_MAX 30	

#define SERVO_CNT 5

Servo sm[5];
int rotate_angle;

// hand : 8
// wrist : 9
// left : 10
// right : 11
// rotate : 12
const int servoPin[5] = {8, 9, 10, 11, 12};
const int servoInit[5] = {120, 170, 0, 180, 55};

void setup() {
  Serial.begin(9600);
  Serial.flush();
  
  for (int i=0; i<SERVO_CNT; ++i)
    sm[i].attach(servoPin[i]);
  
  for (int i=0; i<SERVO_CNT; ++i)
    sm[i].write(servoInit[i]);

  rotate_angle = servoInit[4];
}

void arm_init()
{
  for (int i=0; i<SERVO_CNT-1; ++i)
    sm[i].write(servoInit[i]);
}

long ping(int iter)
{
  long sum=0;

  for (int i=0; i<iter; ++i)
  {
    long duration, cm;
    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    pinMode(ECHO_PIN, INPUT);
    duration = pulseIn(ECHO_PIN, HIGH);

    cm = msToCm(duration);
    sum += cm;
  }
  
  return sum/iter;
}

long msToCm(long duration)
{
  return duration / 29 / 2;
}

double getDistance(long cm)
{
  if (cm < RANGE_MIN || cm > RANGE_MAX)
    return 0;
  else
    return cm;
}

void rotateCenter(){
  // 1. distance 값이 나오면, 그 때의 rotate_angle을 저장(start)
  // 2. 3번 회전하면서 얻은 distance의 평균이 일정 값보다 작으면 그 때의 rotate_angle을 저장(end)
  // 3. (start+end)/2로 회전
  long temp[3] = {RANGE_MIN, 0, 0};
  long cm, distance;
  int sum, start=0, end=0;
  
  start = rotate_angle;
  for (int i=1; ;++i)
  {
      sum = 0;
      if (i > 2)
        i = 0;

      cm = ping(3);
      distance = getDistance(cm);
      Serial.println(distance);
      temp[i] = distance;      

      for (int j=0; j<3; ++j)
        sum+=temp[j]; 
      
      if (sum < RANGE_MIN*1.9)
      {
        end = rotate_angle;
        break;
      }

      ++rotate_angle;
      sm[4].write(rotate_angle);
      delay(300);      
  }
  
  sm[4].write((start+end)/2);
}

void shoulder(int angle1, int angle2)
{
  int i = angle1;
  if (angle1 <= angle2)
  {
    for ( ; i<=angle2; ++i)
    {
      sm[2].write(i);
      sm[3].write(180-i);
      delay(150);
    }
    }
  else
  {
    for ( ; i>=angle2; --i)
    {
      sm[2].write(i);
      sm[3].write(180-i);
      delay(150);
    }
  }
}

void arm(int angle1, int angle2)
{
  int i = angle1;
  if (angle1 <= angle2)
  {
    for ( ; i<=angle2; ++i)
    {
      sm[1].write(i);
      delay(150);
    }
    }
  else
  {
    for ( ; i>=angle2; --i)
    {
      sm[1].write(i);
      delay(150);
    }
  }
}

void grap()
{
  for (int i=120; i<=180; ++i)
  {
    sm[0].write(i);
    delay(150);
  }
}

void release()
{
  for (int i=180; i>=120; --i)
  {
    sm[0].write(i);
    delay(150);
  }
}


void pickObject()
{
  shoulder(0, 35);
  delay(250);
  arm(170, 120);
  shoulder(35, 65);
  delay(250);
  arm(120, 100);
  grap();
}

void moveToGoal()
{
  shoulder(65, 0);
  for (int i=0; rotate_angle<140; ++i)
  {
    sm[4].write(rotate_angle+i);
    delay(150);
  }
  shoulder(0, 65);
  release();
  shoulder(65, 0);
  arm(100, 170);
  for (int i=0; i<140-rotate_angle; ++i)
  {
    sm[4].write(rotate_angle-i);
  }
  sm[4].write(rotate_angle);
}

void loop() {
  long cm, distance;
  cm = ping(3);
  distance = getDistance(cm);

  Serial.println(distance); 
  // pick object
  if (distance)
  {
    rotateCenter();
    Serial.println(rotate_angle);
    // 1. 팔을 내리고, 뻗고, 물건을 집는다.
    pickObject();
    // 2. 팔을 올린다.
    // 3. 목표 지점으로 rotate 한다.
    // 4. 팔을 내리고 물건을 놓는다.
    // 5. 기본 상태로 팔을 세팅한 뒤 물건을 집은 위치로 회전한다.
    moveToGoal();
    delay(10000);
  }
  // rotate
  else  
    ++rotate_angle;   
  
  if (rotate_angle > 140)
  {
    rotate_angle = servoInit[4];
    delay(10000);
  }

  sm[4].write(rotate_angle);
  delay(300);
}

/*
void InverseKinematics(double L1, double L2, double distance , double height) 
{
  double thetaT, theta1, theta2;
  double val = PI / 180;

  thetaT = acos( distance / sqrt(distance * distance + height * height));
  
  theta1 = acos((L1 * L1 + distance * distance + height * height - L2 * L2) / 
                (2 * L1 * sqrt(distance * distance + height * height))) + thetaT;
                                
  theta2 = (180 * val ) - acos((L1 * L1 + L2 * L2 - (distance * distance + height * height)) / 
                  ( 2 * L1 * L2));  
  Serial.println("Inverse Kinematics :");
  Serial.print("Distance :");
  Serial.print(distance);
  Serial.print(" Height :");
  Serial.println(height);
  Serial.print("Angle Theta1 :");
  Serial.println(theta1/val);
  Serial.print("Angle Theta2 :");
  Serial.println(theta2/val);
  
  moveServo(3, 240 - theta1/val, 15);
  moveServo(2, theta2/val + 90, 15);
  moveServo(1, theta1/val + 30, 15);

  moveServo(4, 0, 15);
  //moveServo(4, 70, 15);

  delay(1000);
  
  
  moveServo(0, 90, 15);
  moveServo(0, 0, 15);

}
*/
