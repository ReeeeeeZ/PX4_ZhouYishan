
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
  #define Kp 0.8f
  float Ki = 0.2;
  float halfT;
  // 一阶龙格库塔积分初值
  float exint = 0, eyint = 0, ezint = 0;
  float norm;
  float q0=1, q1=0, q2=0, q3=0;
  float c11, c12, c13, c21, c22, c23, c31, c32, c33;
  float vx, vy, vz, ex, ey, ez;
  
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
    halfT = elapsedTime/2;
    Wire.beginTransmission(MPU);
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); 
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0 + 0.85; 
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0 + 0.1;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0 - 1.27;
    Gyrofilter(); // 输出结果角度值，进行滑动平均滤波
    /*
    pitch = atan(AccX/sqrt(AccY*AccY+AccZ*AccZ));
    yaw =  (yaw + GyroZ * elapsedTime)*PI/180;
    roll = atan(AccY/AccZ);*/
    
    mahony(); // mahony姿态解算
    // 对三个欧拉角进行卡尔曼滤波
    //roll = kalmanX.getAngle(roll, GyroX, elapsedTime);
    //pitch = kalmanY.getAngle(pitch, GyroY, elapsedTime);
    
    // 输出三个欧拉角Roll,Pitch,Yaw
    Serial.print(roll);Serial.print(",");
    Serial.println(pitch);
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
    exint = exint + ex*Ki*halfT;
    eyint = eyint + ey*Ki*halfT;
    ezint = ezint + ez*Ki*halfT;
    GyroX = GyroX + Kp*ex + exint; 
    GyroY = GyroY + Kp*ey + eyint; 
    GyroZ = GyroZ + Kp*ez + ezint; 

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
    //Serial.println(q0);
    //Serial.println(q1);
    //Serial.println(q2);
    //Serial.println(q3);
    
    // 第七步 更新欧拉角
    roll = atan(2*(q3*q2+q0*q1)/(1-2*(q1*q1+q2*q2)))*180/PI;
    pitch = -asin(2*(q1*q3-q0*q2))*180/PI;

  }
