  #include <Wire.h>
  #include <math.h>
  #include <Kalman.h>
  
  const int MPU = 0x68; // MPU6050位置
  Kalman kalmanX, kalmanY, kalmanZ;
  float AccX, AccY, AccZ;
  float GyroX, GyroY, GyroZ;
  float hx, hy, hz, wx, wy, wz, bx, by, bz;
  float ix, iy, iz, jx, jy, jz, kx, ky, kz, s, s1;
  float AngleX;
  float roll, pitch, yaw;
  float roll_new, pitch_new, yaw_new;
  float GyroX_new, GyroY_new, GyroZ_new, accangleX, accangleY, accangleZ;
  float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
  float elapsedTime, currentTime, previousTime;
  int c = 0;
  
  #define FILTER_NUM  8 // 滑动平均滤波数值个数
  float filterx[FILTER_NUM] = {0}, filtery[FILTER_NUM] = {0}, filterz[FILTER_NUM] = {0};
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  // 定义滤波系数
  const float alpha = 0.2; // 低通滤波系数，取值范围为0到1
  // 定义变量
  float filteredAccX = 0.0; // 经过滤波后的X方向加速度
  float filteredAccY = 0.0; // 经过滤波后的Y方向加速度
  float filteredAccZ = 0.0; // 经过滤波后的Z方向加速度
  
  // 定义互补滤波PI控制器系数与半周期
  #define mahony_Kp 0.8f
  float mahony_Ki = 0.2;
  float halfT;
  // 一阶龙格库塔积分初值
  float exint = 0, eyint = 0, ezint = 0;
  float norm;
  float q0=1, q1=0, q2=0, q3=0;
  float c11, c12, c13, c21, c22, c23, c31, c32, c33;
  float vx, vy, vz, ex, ey, ez;
  
  // 定义pid控制结构
  struct _pid{
   float SetSpeed; // 定义设定值
   float ActualSpeed; //定义实际值
   float err; //定义偏差值
   float err_1; //定义上一个偏差值
   float err_2;
   float Kp, Ki, Kd; // 定义比例、积分、微分系数
   float integral; //定义积分值
  } pid, pidangle;

  // 定义pid内外环积分分离限制
  float angle_err_max = 200, anglespeed_err_max = 200;
  int index;
  // 定义pid内外环积分饱和量
  float anglespeed_max, angle_max;

  // 初始化变量
  void PID_init(){
   pid.SetSpeed = 0;
   pid.ActualSpeed = 0;
   pid.err = 0;
   pid.err_1 = 0;
   pid.err_2 = 0;
   pid.integral = 0;
   pid.Kp = 0.2;
   pid.Ki = 0.015;
   pid.Kd = 0.2;
   pidangle.SetSpeed = 0;
   pidangle.ActualSpeed = 0;
   pidangle.err = 0;
   pidangle.err_1 = 0;
   pidangle.err_2 = 0;
   pidangle.integral = 0;
   pidangle.Kp = 0.2;
   pidangle.Ki = 0.015;
   pidangle.Kd = 0.2;
  }

  void setup() {
    Serial.begin(9600);               // 与计算机通信速率9600
    Wire.beginTransmission(MPU);       
    Wire.write(0x6B);                  
    Wire.write(0x00);                  
    Wire.endTransmission(true);        
    
    // 选择量程
    Wire.beginTransmission(MPU);
    Wire.write(0x1C);                  
    Wire.write(0x00);                  // +-2g量程
    Wire.endTransmission(true);
    // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
    Wire.beginTransmission(MPU);
    Wire.write(0x1B);                   
    Wire.write(0x00);                   // 250deg/s 量程
    Wire.endTransmission(true);
    delay(20);
    
    delay(20);
  }

  void loop() {
    // put your main code here, to run repeatedly:
    // === 加载加速度计数据 === //
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); 
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 - 0.05; // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 + 0.02; // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ; // Z-axis value
    Accfilter(); // 加速度数据进行低通滤波
    
    // === 加载陀螺仪数据 === //
    previousTime = currentTime;        
    currentTime = millis();            
    elapsedTime = (currentTime - previousTime) / 1000; 
    float elapsedTime2 = elapsedTime;
    halfT = elapsedTime/2;
    Wire.beginTransmission(MPU);
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); 
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0 + 0.85; 
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0 + 0.1;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0 - 1.27;
    Gyrofilter(); // 输出结果角度值，进行滑动平均滤波
    
    mahony(); // mahony姿态解算
    // 对三个欧拉角进行卡尔曼滤波
    //roll = kalmanX.getAngle(roll, GyroX, elapsedTime);
    //pitch = kalmanY.getAngle(pitch, GyroY, elapsedTime);
 
    //更新三轴转动角速度，输出为角度制
    GyroX = GyroX*180/PI;
    GyroY = GyroY*180/PI;
    GyroZ = GyroZ*180/PI;

    // 对三轴角度进行内环PID
    PID_init();
    GyroX_new = PID_anglespeed(0,GyroX);
    GyroY_new = PID_anglespeed(0,GyroY);
    GyroZ_new = PID_anglespeed(0,GyroZ);

    // === 加载陀螺仪数据 === //
    previousTime = currentTime;        
    currentTime = millis();            
    elapsedTime = (currentTime - previousTime) / 1000; 
    elapsedTime2 += elapsedTime;
    halfT = elapsedTime/2;
    Wire.beginTransmission(MPU);
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); 
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0 + 0.85; 
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0 + 0.1;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0 - 1.27;
    Gyrofilter(); // 输出结果角度值，进行滑动平均滤波
    
    mahony(); // mahony姿态解算
 
    //更新三轴转动角速度，输出为角度制
    GyroX = GyroX*180/PI;
    GyroY = GyroY*180/PI;
    GyroZ = GyroZ*180/PI;

    // 对三轴角速度进行外环PID
    PID_init();
    roll_new = PID_angle(0,roll);
    pitch_new = PID_angle(0,pitch);
    yaw_new = PID_angle(0,yaw);
    GyroX = (roll_new-roll)/elapsedTime2;
    GyroY = (pitch_new-pitch)/elapsedTime2;
    GyroZ = (yaw_new-yaw)/elapsedTime2;
    
    // 对三轴角度进行内环PID
    PID_init();
    GyroX_new = PID_anglespeed(0,GyroX);
    GyroY_new = PID_anglespeed(0,GyroY);
    GyroZ_new = PID_anglespeed(0,GyroZ);
  }

  void Gyrofilter(){
    // 用滑动加法平均滤波处理陀螺仪数据
    sum_gx = sum_gx-filterx[0];
    sum_gy = sum_gy-filtery[0];
    sum_gz = sum_gz-filterz[0];
    while (c < FILTER_NUM-1) {
      filterx[c] = filterx[c+1];
      filtery[c] = filtery[c+1];
      filterz[c] = filterz[c+1];
      c++;
    }// 将前7个数据移位
    c = 0;
    filterx[FILTER_NUM-1] = GyroX;
    sum_gx = sum_gx+GyroX;
    filtery[FILTER_NUM-1] = GyroY;
    sum_gy = sum_gy+GyroY;
    filterz[FILTER_NUM-1] = GyroZ;
    sum_gz = sum_gz+GyroZ;
    // 更新滑动数组
    
    GyroX = sum_gx / FILTER_NUM;
    GyroY = sum_gy / FILTER_NUM;
    GyroZ = sum_gz / FILTER_NUM;
  }

  void Accfilter(){
    // 用一阶低通滤波处理加速度计数据
    filteredAccX = (alpha * AccX) + ((1 - alpha) * filteredAccX);
    AccX = filteredAccX;
    filteredAccY = (alpha * AccY) + ((1 - alpha) * filteredAccY);
    AccY = filteredAccY;
    filteredAccZ = (alpha * AccZ) + ((1 - alpha) * filteredAccZ);
    AccZ = filteredAccZ;
  }

  void mahony () {
    GyroX = GyroX*PI/180;
    GyroY = GyroY*PI/180;
    GyroZ = GyroZ*PI/180;   
    
    // 第一步 初始化四元数
    /*q0 = cos(pitch/2)*cos(roll/2)*cos(yaw/2)+sin(pitch/2)*sin(roll/2)*sin(yaw/2);
    q1 = sin(pitch/2)*cos(roll/2)*cos(yaw/2)-cos(pitch/2)*sin(roll/2)*sin(yaw/2);
    q2 = cos(pitch/2)*sin(roll/2)*cos(yaw/2)+sin(pitch/2)*cos(roll/2)*sin(yaw/2);
    q3 = cos(pitch/2)*cos(roll/2)*sin(yaw/2)+sin(pitch/2)*sin(roll/2)*cos(yaw/2);*/
    
    // 第二步 标准化速度加速度
    norm = sqrt(AccX*AccX+AccY*AccY+AccZ*AccZ);
    AccX = AccX/norm;
    AccY = AccY/norm;
    AccZ = AccZ/norm;
    //Serial.println(AccX);
    //Serial.println(AccY);
    //Serial.println(AccZ);

    // 第三步 误差
    vx = 2 * ( q1*q3 - q0*q2 );
    vy = 2 * ( q2*q3 + q0*q1 );
    vz = pow(q0,2) - pow(q1,2) - pow(q2,2) + pow(q3,2);

    ex = AccY*vz - AccZ*vy;
    ey = AccZ*vx - AccX*vz;
    ez = AccX*vy - AccY*vx;
    exint = exint + ex*mahony_Ki*halfT;
    eyint = eyint + ey*mahony_Ki*halfT;
    ezint = ezint + ez*mahony_Ki*halfT;
    GyroX = GyroX + mahony_Kp*ex + exint; 
    GyroY = GyroY + mahony_Kp*ey + eyint; 
    GyroZ = GyroZ + mahony_Kp*ez + ezint; 

    // 第五步 更新四元数
    q0 = q0 + (-q1*GyroX-q2*GyroY-q3*GyroZ)*halfT;
    q1 = q1 + (q0*GyroX+q2*GyroZ-q3*GyroY)*halfT;
    q2 = q2 + (q0*GyroY-q1*GyroZ+q3*GyroX)*halfT;
    q3 = q3 + (q0*GyroZ+q1*GyroY-q2*GyroX)*halfT;
    
    // 第六步 规范化
    norm = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
    q0 = q0/norm;
    q1 = q1/norm;
    q2 = q2/norm;
    q3 = q3/norm;
    
    // 第七步 更新欧拉角
    roll = atan(2*(q3*q2+q0*q1)/(1-2*(q1*q1+q2*q2)))*180/PI;
    pitch = -asin(2*(q1*q3-q0*q2))*180/PI;

  }

  float PID_anglespeed(float speed, float actualspeed){  // 由角速度计算角加速度再到力矩，内环控制
   pid.SetSpeed = speed;
   pid.ActualSpeed = actualspeed;
   pid.err = pid.SetSpeed - pid.ActualSpeed;
   if (pid.ActualSpeed > anglespeed_max) {
     if (pid.err > 0) {index = 0;}
     else {index = 1;}
   }
   else {index = 1;}
   float incrementSpeed = pid.Kp*(pid.err-pid.err_1)+index*pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_1+pid.err_2);
   pid.ActualSpeed += incrementSpeed;
   pid.err_2 = pid.err_1;
   pid.err_1 = pid.err;
   return pid.ActualSpeed;
  }

  float PID_angle(float angle, float actualangle){   // 由角度计算理想角速度，外环控制
   pidangle.SetSpeed = angle;
   pidangle.ActualSpeed = actualangle;
   pidangle.err = pidangle.SetSpeed - pidangle.ActualSpeed;
   if (pid.ActualSpeed > angle_max) {
     if (pid.err > 0) {index = 0;}
     else {index = 1;}
   }
   else {index = 1;}
   float incrementSpeed = pidangle.Kp*(pidangle.err-pidangle.err_1)+index*pidangle.Ki*pidangle.err;
   pidangle.ActualSpeed += incrementSpeed;
   pidangle.err_1 = pidangle.err;
   return pidangle.ActualSpeed;
  }
   
