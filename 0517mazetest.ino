/////////////////////sonar///////////////////////

#include <NewPing.h> // NewPing 라이브러리 포함

#define SONAR_NUM 3      // 센서의 수
#define MAX_DISTANCE 150 // 최대 거리 (cm 단위)
#define WALL_GAP_DISTANCE 320 // 벽 사이 거리 (mm 단위)
#define WALL_GAP_DISTANCE_HALF 200 // 벽 사이 거리의 절반 (mm 단위)
#define MOTOR_PWM_OFFSET 10 // 모터 PWM 오프셋

#define Front 0 // 전방 초음파 센서 인덱스
#define Left  1 // 좌측 초음파 센서 인덱스
#define Right 2 // 우측 초음파 센서 인덱스

#define TRIG1 31 // 초음파 센서 1번 Trig 핀 번호
#define ECHO1 30 // 초음파 센서 1번 Echo 핀 번호

#define TRIG2 27 // 초음파 센서 2번 Trig 핀 번호
#define ECHO2 26 // 초음파 센서 2번 Echo 핀 번호

#define TRIG3 34 // 초음파 센서 3번 Trig 핀 번호
#define ECHO3 35 // 초음파 센서 3번 Echo 핀 번호

NewPing sonar[SONAR_NUM] = 
{
    // 초음파 센서 배열 초기화
    NewPing(TRIG1, ECHO1, MAX_DISTANCE), // 첫 번째 센서
    NewPing(TRIG2, ECHO2, MAX_DISTANCE), // 두 번째 센서
    NewPing(TRIG3, ECHO3, MAX_DISTANCE)  // 세 번째 센서
};

/////////////////////L298//////////////////////////
#define ENA 7  // 모터 A의 Enable 핀
#define IN1 11 // 모터 A의 입력 핀 1
#define IN2 10 // 모터 A의 입력 핀 2
#define IN3 9  // 모터 B의 입력 핀 1
#define IN4 8  // 모터 B의 입력 핀 2
#define ENB 6  // 모터 B의 Enable 핀

float front_sonar = 0.0; // 전방 초음파 센서 거리
float left_sonar  = 0.0; // 좌측 초음파 센서 거리
float right_sonar = 0.0; // 우측 초음파 센서 거리

/////////////////////Maze_Status//////////////////////////
int maze_status = 0; // 미로 상태 변수

void setup() 
{
  Serial.begin(115200); // 시리얼 통신 속도 설정
  
  // 초음파 센서 핀 설정
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);
  
  // 모터 핀 설정
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void motor_A_control(int direction_a, int motor_speed_a) // 모터 A의 방향과 속도를 제어
{
  if(direction_a == HIGH)
  {
     digitalWrite(IN1, LOW); // 모터 A 방향 설정
     digitalWrite(IN2, HIGH);
     analogWrite(ENA, motor_speed_a); // 모터 A 속도 설정
  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, motor_speed_a);
  }
}

void motor_B_control(int direction_b, int motor_speed_b) // 모터 B의 방향과 속도를 제어
{
  if(direction_b == HIGH)
  {
     digitalWrite(IN3, LOW); // 모터 B 방향 설정
     digitalWrite(IN4, HIGH);
     analogWrite(ENB, motor_speed_b); // 모터 B 속도 설정
  }
  else
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, motor_speed_b);
  }
}

void check_maze_status(void)
{
  // 미로 상태를 확인하여 maze_status 값을 설정
  if((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE) && 
     (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE) && 
     (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF))
  {
    maze_status = 4; // 3면이 막힌 경우
    Serial.println("maze_status = 4");
  }
  else if((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE) && 
          (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE) && 
          (front_sonar >= WALL_GAP_DISTANCE_HALF))  
  {
    maze_status = 1; // 전방이 열린 경우
    Serial.println("maze_status = 1");
  }
  else if((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE) && 
          (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF))
  {
    maze_status = 2; // 좌측이 막힌 경우
    Serial.println("maze_status = 2");
  }
  else if((right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE) && 
          (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF))
  {
    maze_status = 3; // 우측이 막힌 경우
    Serial.println("maze_status = 3");
  }
  else
  {
    maze_status = 0; // 모든 조건에 해당하지 않는 경우
    Serial.println("maze_status = 0");
  }
}

void wall_collision_avoid(int base_speed) // 벽 충돌 방지 함수
{
  float error = 0.0;
  float Kp = 0.7; // 비례 상수, 나중에 조정 가능
  int pwm_control = 0;
  int right_pwm = 0;
  int left_pwm = 0;

  error = (right_sonar - left_sonar); // 좌우 초음파 센서의 차이로 에러 계산
  error = Kp * error;
  
  if(error >= 50) error = 50;
  if(error <= -50) error = -50; 

  right_pwm = base_speed - error; // 우측 모터 속도 조정
  left_pwm = base_speed + error;  // 좌측 모터 속도 조정
  
  if(right_pwm <= 0) right_pwm = 0;
  if(left_pwm <= 0) left_pwm = 0;
  if(right_pwm >= 230) right_pwm = 230;
  if(left_pwm >= 220) left_pwm = 220;
  
  motor_A_control(HIGH, left_pwm); // 오른쪽 전진
  motor_B_control(HIGH, right_pwm); // 왼쪽 전진
}

void loop() 
{
  // 초음파 센서로 거리 측정
  front_sonar = sonar[Front].ping_cm() * 10; // 전방 센서 거리(mm)
  left_sonar = sonar[Left].ping_cm() * 10;  // 좌측 센서 거리(mm)
  right_sonar = sonar[Right].ping_cm() * 10; // 우측 센서 거리(mm)
  
  if(front_sonar == 0.0) front_sonar = MAX_DISTANCE * 10; // 측정 실패 시 최대 거리로 설정
  if(left_sonar == 0.0) left_sonar = MAX_DISTANCE * 10;
  if(right_sonar == 0.0) right_sonar = MAX_DISTANCE * 10;

  // 센서 측정값 출력
  Serial.print("L: "); Serial.print(left_sonar); Serial.print(" ");
  Serial.print("F: "); Serial.print(front_sonar); Serial.print(" ");
  Serial.print("R: "); Serial.println(right_sonar);

  // 미로 상태 확인
  check_maze_status();

  // 미로 상태에 따른 동작
  if(maze_status == 4)
  {
    // 정지
    Serial.println("Rotate CCW");
    motor_A_control(HIGH, 0);
    motor_B_control(LOW, 0);
    delay(100);
    
    // 180도 회전
    motor_A_control(HIGH, 170); // 왼쪽 전진
    motor_B_control(LOW, 160);  // 오른쪽 후진
    delay(700);
    
    // 정지
    motor_A_control(HIGH, 0);
    motor_B_control(LOW, 0);
    delay(100);
  }
  else if(maze_status == 1)
  {
    // 직진
    Serial.println("run straight");
    wall_collision_avoid(230);
  } 
  else if(maze_status == 3)
  {
    // 정지
    Serial.println("Rotate CCW");
    motor_A_control(HIGH, 0);
    motor_B_control(LOW, 0);
    delay(100);

    // 좌회전
    motor_A_control(LOW, 160); // 왼쪽 후진
    motor_B_control(HIGH, 170); // 오른쪽 전진
    delay(350);
  }
  else if(maze_status == 2)
  {
    // 정지
    Serial.println("Rotate CW");
    motor_A_control(HIGH, 0);
    motor_B_control(LOW, 0);
    delay(100);

    // 우회전
    motor_A_control(HIGH, 170); // 왼쪽 전진
    motor_B_control(LOW, 160);  // 오른쪽 후진
    delay(350);
  }
  else
  {
    // 기본 동작
    Serial.println("run default");
    wall_collision_avoid(230);
  }
}
