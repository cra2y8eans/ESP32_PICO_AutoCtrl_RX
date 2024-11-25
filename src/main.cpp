/***********************************************************************************************************************************************************

ESP32_PICO 手抛飞机自稳接收机

- 版本：1.0.0    1、明确需要接收和回传的数据
                   * 接收：自稳开启flag，摇杆中值，油门、升降舵、副翼的ADC值，X轴、Y轴的PID参数，转向系数等。其中PID及转向系数可在调参完成后写死。
                   * 发送：电池电量ADC值，X、Y轴角度和角速度，舵机执行的角度等。为减轻MCU负担，可在调参完毕后只回传电量这一项数据。
                 2、采用switch case方法，用自稳flag作为判断条件，以便在手动控制和自稳混控中切换。
                 3、舵机所需执行角度计算方法：
                   * 计算出摇杆实际拨动的值：摇杆当前读数 - 摇杆中值 = 实际拨动的值。差值有正负，用以区分拨动的方向。此中值为开机时读取到的值。
                   * 确定角度系数为120 ÷ 256 = 0.46875，然后将上面求得的实际拨动的值 * 角度系数 = 预期平衡角度
                   * 使用预期平衡角度 - 当前飞机角度 = 角度差，带入PID计算公示为：比例项P*角度差 + 积分项I*（角度差+=角度差）+ 微分项D*角速度 = 控制所需的PWM值。
                   * 因为舵机最大角度设定为120°，所以PWM值换算成角度的范围只能在-60°到60°之间。
                   * 将摇杆ADC中值换算成角度，并以这个角度为舵机角度中值，在角度中值的基础上加减前面计算出来的角度，得到舵机最终需要执行的角度。
                 4、PID调试
                   * 因为计算公式的问题，需要将Y轴的PID值都调到负数才能匹配飞机姿态和舵面纠正：P = -4.0, I = -0.01, D = -0.2;
                 5、接收襟翼开关数据，只有手动模式下才能打开襟翼。
                 6、使用git管理代码

************************************************************************************************************************************************************/

#include <Arduino.h>
#include <ESP32Servo.h>
#include <MPU6050_tockn.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
// #include <BluetoothSerial.h>

/*------------------------------------------------- ESP NOW -------------------------------------------------*/

// 创建ESP NOW通讯实例
esp_now_peer_info_t peerInfo;

uint8_t padAddress[] = { 0x08, 0xa6, 0xf7, 0x17, 0x6d, 0x84 }; // ESP32_薄
// uint8_t padAddress[] = { 0x2c, 0xbc, 0xbb, 0x00, 0x52, 0xd4 }; // ESP32_厚

// 存储接收到的数据
struct Pad {
  int   button_flag[2]      = {}; // 0、自稳开关         1、襟翼开关
  int   joystick_mid_val[2] = {}; // 0、副翼中值         1、升降舵中值
  int   joystick_ADC[3]     = {}; // 0、油门             1、副翼；       2、升降舵
  float x_pid_data[3]       = {}; // 0、X轴比例          1、X轴积分；    2、X轴微分
  float y_pid_data[3]       = {}; // 0、Y轴比例          1、Y轴积分；    2、Y轴微分
  float coe[1]              = {}; // 0、舵机角度换算系数
};
Pad pad;

// 发回遥控器数据
struct Aircraft {
  int   batteryValue[1] = {}; // 0、电池电量ADC值
  int   servo_angle[2]  = {}; // 0、升降舵机角度   1、副翼舵机角度
  float x_data[2]       = {}; // 0、X轴角度        1、X轴角速度
  float y_data[2]       = {}; // 0、Y轴角度        1、Y轴角速度
};
Aircraft aircraft;

// 连接成功标志位
bool esp_connected;

/*------------------------------------------------- 陀螺仪 -------------------------------------------------*/

#define SDA_PIN 18
#define SCL_PIN 23
float
    angle_X, // X轴角度
    angle_Y, // Y轴角度
    gyro_X,  // X轴角速度
    gyro_Y;  // Y轴角速度

MPU6050 mpu6050(Wire);

/*------------------------------------------------- 舵机 -------------------------------------------------*/

#define SERVO_AILERON_L 25    // 左副翼引脚
#define SERVO_AILERON_R 13    // 右副翼引脚
#define SERVO_ELEVATOR 14     // 升降舵引脚
#define SERVO_FREQ_MIN 500    // 舵机最小频率
#define SERVO_FREQ_MAX 2500   // 舵机最大频率
#define SERVO_ANGLE_RANGE 120 // 舵机角度范围

// int SERVO_ANGLE_RANGE = 120; // 舵机角度范围
int pitch_servo_angle, roll_servo_angle;

// ESP32Servo库
Servo Aileron_L;
Servo Aileron_R;
Servo Elevator;

/*------------------------------------------------- 电机 -------------------------------------------------*/

#define MOTOR_PIN_L 26  // 左电机引脚
#define MOTOR_PIN_R 27  // 右电机引脚
#define MOTOR_CHANNEL 4 // 电机PWM通道
#define RESOLUTION 8    // 电机PWM精度
#define FREQUENCY 30000 // 电机频率

/*------------------------------------------------- P.I.D -------------------------------------------------*/

#define DEVIATION_RANGE_NEG -300
#define DEVIATION_RANGE_POS 300

float
    MaitainedAngle_X, // 俯仰控制角度
    MaitainedAngle_Y, // 滚转控制角度
    integral_X,       // 俯仰偏差积分
    integral_Y,       // 滚转偏差积分
    deviation_X,      // 俯仰偏差角度
    deviation_Y;      // 滚转偏差角度

/*------------------------------------------------- 滤波 -------------------------------------------------*/

#define LIMIT_FILTER 10    // 限幅滤波阈值，建议取值范围3~10，值越小，操控越需要柔和
#define AVERAGE_FILTER 100 // 均值滤波，N次取样平均，建议取值范围20~80
#define ADC_MIN 0
#define ADC_MAX 255

/*------------------------------------------------- 电量读取 -------------------------------------------------*/

#define BATTERY_PIN 35            // 电量读取引脚
#define BATTERY_INTERVAL 2000     // 电量检测间隔时间，单位毫秒
unsigned long previousMillis = 0; // 当前检测时间

/*------------------------------------------------- 自定义函数 -------------------------------------------------*/

// 数据发出去之后的回调函数
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    esp_connected = true;
  } else {
    esp_connected = false;
  }
}

// 收到消息后的回调
void OnDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  memcpy(&pad, data, sizeof(pad)); // 将收到的消息进行存储
}

// 限幅滤波，防止尖端突变
int limit_filter(int pin) {
  static int last_val = analogRead(pin); // 静态变量，只初始化一次，全程序中保存在内存
  int        val      = analogRead(pin);
  if (abs(val - last_val) > LIMIT_FILTER) {
    val = last_val;
  }
  last_val = analogRead(pin);
  return val;
}

// 均值滤波，抑制噪声
int avg_filter(int pin) {
  int val, sum = 0;
  for (int count = 0; count < AVERAGE_FILTER; count++) {
    sum += analogRead(pin);
  }
  val = sum / AVERAGE_FILTER;
  return val;
}

// 限幅滤波+均值滤波
int limit_avg_filter(int pin) {
  int val, sum = 0;
  for (int count = 0; count < AVERAGE_FILTER; count++) {
    sum += limit_filter(pin);
  }
  val = sum / AVERAGE_FILTER;
  return val;
}

// 获取飞机姿态数据
void GetAttitudeData() {
  mpu6050.update();
  angle_X = mpu6050.getAngleX();
  angle_Y = mpu6050.getAngleY();
  gyro_X  = mpu6050.getGyroX();
  gyro_Y  = mpu6050.getGyroY();
}

// X轴升降舵保持平飞舵机角度计算
int pitch_balance() {
  GetAttitudeData(); // 获取姿态信息
  int pitch_servo_pwm;
  int angle_pitch;
  MaitainedAngle_X = (pad.joystick_ADC[2] - pad.joystick_mid_val[0]) * pad.coe[0];                                 // 动态维持的角度 = （升降舵ADC - 摇杆中值） * 角度系数
  deviation_X      = angle_X - MaitainedAngle_X;                                                                   // 偏差角度 = 当前角度 - 动态维持的角度（遥控器发送过来的角度）
  integral_X += deviation_X;                                                                                       // 积分累计
  integral_X      = constrain(integral_X, DEVIATION_RANGE_NEG, DEVIATION_RANGE_POS);                               // 限制积分上限
  pitch_servo_pwm = pad.x_pid_data[0] * deviation_X + pad.x_pid_data[1] * integral_X + pad.x_pid_data[2] * gyro_X; // 引入PID参数，计算所需的pmw值
  pitch_servo_pwm = constrain(pitch_servo_pwm, -128, 127);                                                         // 限制pwm值的范围
  angle_pitch     = map(pitch_servo_pwm, -128, 127, SERVO_ANGLE_RANGE / -2, SERVO_ANGLE_RANGE / 2);                // 将PWM换算成角度

  return angle_pitch;
}

// Y轴副翼保持平飞舵机角度计算
int roll_balance() {
  GetAttitudeData();
  int roll_servo_pwm;
  int angle_roll;
  MaitainedAngle_Y = (pad.joystick_ADC[1] - pad.joystick_mid_val[1]) * pad.coe[0];                                // 动态维持的角度 = （副翼ADC - 摇杆中值） * 角度系数
  deviation_Y      = angle_Y - MaitainedAngle_Y;                                                                  // 偏差角度 = 当前角度 - 动态维持的角度（遥控器发送过来的角度）
  integral_Y += deviation_Y;                                                                                      // 积分累计
  integral_Y     = constrain(integral_Y, DEVIATION_RANGE_NEG, DEVIATION_RANGE_POS);                               // 限制积分上限
  roll_servo_pwm = pad.y_pid_data[0] * deviation_Y + pad.y_pid_data[1] * integral_Y + pad.y_pid_data[2] * gyro_Y; // 计算所需的pmw值
  roll_servo_pwm = constrain(roll_servo_pwm, -128, 127);                                                          // 限制pwm值的范围
  angle_roll     = map(roll_servo_pwm, -128, 127, SERVO_ANGLE_RANGE / -2, SERVO_ANGLE_RANGE / 2);                 // 将PWM换算成角度

  return angle_roll;
}

// 操控
void airCraftControl() {
  if (esp_connected == true) {
    int ail_mid_angle, ele_mid_angle;
    switch (pad.button_flag[0]) {
    case 0:
      // 手动模式
      roll_servo_angle  = map(pad.joystick_ADC[1], ADC_MIN, ADC_MAX, ADC_MIN, SERVO_ANGLE_RANGE);
      pitch_servo_angle = map(pad.joystick_ADC[2], ADC_MIN, ADC_MAX, ADC_MIN, SERVO_ANGLE_RANGE);
      /*
      // 襟翼必须在自稳模式关闭的前提下才能开启
      if (pad.button_flag[0] == 0 && pad.button_flag[1] == 1) {
        Aileron_L.write(SERVO_ANGLE_RANGE - roll_servo_angle);
        Aileron_R.write(roll_servo_angle);
      } else {
        Aileron_L.write(SERVO_ANGLE_RANGE - roll_servo_angle);
        Aileron_R.write(SERVO_ANGLE_RANGE - roll_servo_angle);
      }
      ledcWrite(MOTOR_CHANNEL, pad.joystick_ADC[0]);
      Elevator.write(SERVO_ANGLE_RANGE - pitch_servo_angle);
      */
      ledcWrite(MOTOR_CHANNEL, pad.joystick_ADC[0]);
      Aileron_L.write(SERVO_ANGLE_RANGE - roll_servo_angle);
      Aileron_R.write(SERVO_ANGLE_RANGE - roll_servo_angle);
      Elevator.write(SERVO_ANGLE_RANGE - pitch_servo_angle);
      break;
    case 1:
      // 自稳模式
      ail_mid_angle     = map(pad.joystick_mid_val[1], ADC_MIN, ADC_MAX, ADC_MIN, SERVO_ANGLE_RANGE);
      ele_mid_angle     = map(pad.joystick_mid_val[0], ADC_MIN, ADC_MAX, ADC_MIN, SERVO_ANGLE_RANGE);
      roll_servo_angle  = ail_mid_angle + roll_balance();
      pitch_servo_angle = ele_mid_angle + pitch_balance();
      ledcWrite(MOTOR_CHANNEL, pad.joystick_ADC[0]);
      Aileron_L.write(SERVO_ANGLE_RANGE - roll_servo_angle);
      Aileron_R.write(SERVO_ANGLE_RANGE - roll_servo_angle);
      Elevator.write(SERVO_ANGLE_RANGE - pitch_servo_angle);
      break;
    default:
      break;
    }
  } else {
    // 电机停转、飞机盘旋
    ledcWrite(MOTOR_CHANNEL, 0);
    Elevator.write(120);
    Aileron_L.write(30);
    Aileron_R.write(120);
  }
}

//  数据回传
void dataSendBack() {
  GetAttitudeData();
  aircraft.x_data[0]       = angle_X;
  aircraft.x_data[1]       = gyro_X;
  aircraft.y_data[0]       = angle_Y;
  aircraft.y_data[1]       = gyro_Y;
  aircraft.servo_angle[0]  = pitch_servo_angle;
  aircraft.servo_angle[1]  = roll_servo_angle;
  aircraft.batteryValue[0] = limit_avg_filter(BATTERY_PIN);

  // 发送
  esp_now_send(padAddress, (uint8_t*)&aircraft, sizeof(aircraft));
}

// 串口输出
void SerialDataPrint() {
  Serial.printf("flag: %d\n", pad.button_flag[0]);
  Serial.printf("油门: %d\n", pad.joystick_ADC[0]);

  Serial.printf("副翼ADC: %d", pad.joystick_ADC[1]);
  Serial.printf("   角度: %d\n", roll_servo_angle);

  Serial.printf("X轴中值: %d\n", pad.joystick_mid_val[0]);
  Serial.printf("Y轴中值: %d\n", pad.joystick_mid_val[1]);

  Serial.printf("升降舵ADC: %d", pad.joystick_ADC[2]);
  Serial.printf("   角度: %d\n", pitch_servo_angle);
  delay(500);
}

/*------------------------------------------------------------------------------------------------------------*/

void setup() {
  Serial.begin(115200);

  // 陀螺仪
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // wifi及ESP NOW初始化
  WiFi.mode(WIFI_STA);                  // 设置wifi为STA模式
  esp_now_init();                       // 初始化ESP NOW
  esp_now_register_send_cb(OnDataSent); // 注册发送成功的回调函数
  esp_now_register_recv_cb(OnDataRecv); // 注册接受数据后的回调函数
  // 注册通信频道
  memcpy(peerInfo.peer_addr, padAddress, 6); // 设置配对设备的MAC地址并储存，参数为拷贝地址、拷贝对象、数据长度
  peerInfo.channel = 1;                      // 设置通信频道
  esp_now_add_peer(&peerInfo);               // 添加通信对象

  // 舵机定时器
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  // 设定舵机频率
  Aileron_L.setPeriodHertz(50);
  Aileron_R.setPeriodHertz(50);
  Elevator.setPeriodHertz(50);
  // 绑定引脚和最大、最小频率
  Aileron_L.attach(SERVO_AILERON_L, SERVO_FREQ_MIN, SERVO_FREQ_MAX);
  Aileron_R.attach(SERVO_AILERON_R, SERVO_FREQ_MIN, SERVO_FREQ_MAX);
  Elevator.attach(SERVO_ELEVATOR, SERVO_FREQ_MIN, SERVO_FREQ_MAX);

  // 电机
  ledcSetup(MOTOR_CHANNEL, FREQUENCY, RESOLUTION); // 通道、频率、精度
  ledcAttachPin(MOTOR_PIN_L, MOTOR_CHANNEL);       // 引脚号、通道
  ledcAttachPin(MOTOR_PIN_R, MOTOR_CHANNEL);       // 引脚号、通道
  ledcWrite(MOTOR_PIN_L, 0);                       // 引脚号、PWM值
  ledcWrite(MOTOR_PIN_R, 0);

  //  电池检测初始化
  analogReadResolution(8);
  aircraft.batteryValue[0] = analogRead(BATTERY_PIN);
}

void loop() {
  airCraftControl();
  dataSendBack();
}