
  #include <Wire.h>
  #include <math.h>
  #include <Kalman.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_HMC5883_U.h>

  // 电子罗盘
  Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
  
  const int MPU = 0x68; // MPU6050位置
  const int HMC = 0x0D; // HMC5883L位置
  Kalman kalmanX, kalmanY, kalmanZ;
  float AccX, AccY, AccZ;
  float GyroX, GyroY, GyroZ;
  float MX, MY, MZ;
  float hx, hy, hz, wx, wy, wz, bx, by, bz;
  float ix, iy, iz, jx, jy, jz, kx, ky, kz, s, s1;
  float gyroAngleX, gyroAngleY, gyroAngleZ;
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
  float q0, q1, q2, q3;
  float c11, c12, c13, c21, c22, c23, c31, c32, c33;
  float vx, vy, vz, ex, ey, ez;
  
  void setup() {
    Serial.begin(9600);               // 与计算机通信速率9600
    if(!mag.begin())
    {
      Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
      while(1);
    }
    Wire.begin();                      // 初始化
    Wire.beginTransmission(HMC);
    Wire.write(0x09);                  //选择控制寄存器0X09
    Wire.write(0x1D);                  //设置0x09寄存器为：OSR:512Hz，RNG：+/-8Gauss，ODG：200Hz，MODE：连续工作模式
    Wire.endTransmission();
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
    
    calculate_IMU_error();
    delay(20);
  }

  void loop() {
    // put your main code here, to run repeatedly:
    // 读磁力计
    sensors_event_t event; 
    mag.getEvent(&event);
    MX = event.magnetic.x;
    MY = event.magnetic.y;
    MZ = event.magnetic.z;
    
    // === 加载加速度计数据 === //
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); 
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 - AccErrorX -0.05; // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 - AccErrorY +0.02; // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 - AccErrorZ -2.01; // Z-axis value
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
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0 - GyroErrorX; 
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0 - GyroErrorY;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0 - GyroErrorZ;
    Gyrofilter(); // 输出结果角度值，进行滑动平均滤波
    
    mahony(); // mahony姿态解算
    // 对三个欧拉角进行卡尔曼滤波
    roll = kalmanX.getAngle(roll, GyroX, elapsedTime);
    pitch = kalmanY.getAngle(pitch, GyroY, elapsedTime);
    yaw = kalmanZ.getAngle(yaw, GyroZ, elapsedTime);
    // 处理翻滚角大小
    if (roll<0) roll=360+roll;
    if (roll>360) roll=roll-360;
    
    // 输出三个欧拉角Roll,Pitch,Yaw
    Serial.print(roll);Serial.print(",");
    Serial.print(pitch);Serial.print(",");
    Serial.println(yaw);
  }

  void calculate_IMU_error() {
    // 对传感器进行静态校准，应该在正式工作前将传感器水平静止放置
    // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
    // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
    // Read accelerometer values 200 times
    
    while (c < 200) {
      Wire.beginTransmission(MPU);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 6, true);
      AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
      AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
      AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
      // Sum all readings
      AccErrorX = AccErrorX + AccX / 16384.0 ;
      AccErrorY = AccErrorY + AccY / 16384.0 ;
      AccErrorZ = AccErrorZ + AccZ / 16384.0 ;
      c++;
    }
    AccErrorX = AccErrorX / 200;
    AccErrorY = AccErrorY / 200;
    AccErrorZ = AccErrorZ / 200;
    c = 0;
    // Read gyro values 200 times
    while (c < 200) {
      Wire.beginTransmission(MPU);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 6, true);
      GyroX = Wire.read() << 8 | Wire.read();
      GyroY = Wire.read() << 8 | Wire.read();
      GyroZ = Wire.read() << 8 | Wire.read();
      // Sum all readings
      GyroErrorX = GyroErrorX + (GyroX / 131.0);
      GyroErrorY = GyroErrorY + (GyroY / 131.0);
      GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
      c++;
    }
    //Divide the sum by 200 to get the error value
    GyroErrorX = GyroErrorX / 200;
    GyroErrorY = GyroErrorY / 200;
    GyroErrorZ = GyroErrorZ / 200;
    c = 0;
  /*
    // Print the error values on the Serial Monitor
    Serial.print("AccErrorX: ");
    Serial.println(AccErrorX);
    Serial.print("AccErrorY: ");
    Serial.println(AccErrorY);
    Serial.print("GyroErrorX: ");
    Serial.println(GyroErrorX);
    Serial.print("GyroErrorY: ");
    Serial.println(GyroErrorY);
    Serial.print("GyroErrorZ: ");
    Serial.println(GyroErrorZ);*/
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
    kx = -AccX/sqrt(AccX*AccX+AccY*AccY+AccZ*AccZ);
    ky = -AccY/sqrt(AccX*AccX+AccY*AccY+AccZ*AccZ);
    kz = -AccZ/sqrt(AccX*AccX+AccY*AccY+AccZ*AccZ);  
    
    // 修正磁力方向，沿正北方向
    ix = MX - MX*kx*kx;
    iy = MY - MY*ky*ky;
    iz = MZ - MZ*kz*kz;
    norm = sqrt(ix*ix+iy*iy+iz*iz);
    ix = ix/norm;
    iy = iy/norm;
    iz = iz/norm;

    // 正东方向
    jx = ky*iz - kz*iy;
    jy = kz*ix - kx*iz;
    jz = kx*iy - ky*ix;
    norm = sqrt(jx*jx+jy*jy+jz*jz);
    jx = jx/norm;
    jy = jy/norm;
    jz = jz/norm;
      
    // 构建矩阵
    c11 = jx; c12 = jy; c13 = jz;
    c21 = ix; c22 = iy; c23 = iz;
    c31 = kx; c32 = ky; c33 = kz;
    // 第二步 初始化四元数
    s = sqrt(c11+c22+c33+1);
    q0 = 0.5*s;
    q1 = (c32-c23)/(4*q0);
    q2 = (c13-c31)/(4*q0);
    q3 = (c21-c12)/(4*q0);

    // 第三步 从四元数反推重力
    norm = sqrt(MX*MX+MY*MY+MZ*MZ);
    MX = MX/norm;
    MY = MY/norm;
    MZ = MZ/norm;
    norm = sqrt(AccX*AccX+AccY*AccY+AccZ*AccZ);
    AccX = AccX/norm;
    AccY = AccY/norm;
    AccZ = AccZ/norm;
    
    vx = 2 * ( q1*q3 - q0*q2 );
    vy = 2 * ( q2*q3 + q0*q1 );
    vz = pow(q0,2) - pow(q1,2) - pow(q2,2) + pow(q3,2);
    
    hx = MX*(1-2*(q2*q2+q3*q3))+MY*2*(q1*q2-q0*q3)+MZ*2*(q1*q3+q0*q2);
    hy = MX*2*(q1*q2+q0*q3)+MY*(1-2*(q1*q1+q3*q3))+MZ*2*(q2*q3+q0*q1);
    hz = MX*2*(q1*q3-q0*q2)+MY*2*(q2*q3+q0*q1)+MZ*(1-2*(q2*q2+q1*q1));
    bx = sqrt(hx*hx+hy*hy);
    bz = hz;
    
    wx = bx*(1-2*(q2*q2+q3*q3))+bz*2*(q1*q3+q0*q2);
    wy = bx*2*(q1*q2+q0*q3)+bz*2*(q2*q3+q0*q1);
    wz = bx*2*(q1*q3-q0*q2)+bz*(1-2*(q2*q2+q1*q1));
    
    // 第四步 计算误差
    ex = AccY*vz - AccZ*vy + MY*wz - MZ*wy;
    ey = AccZ*vx - AccX*vz + MZ*wx - MX*wz;
    ez = AccX*vy - AccY*vx + MX*wy - MY*wx;
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
    
    // 第七步 更新欧拉角
    roll = atan(2*(q0*q3+q2*q1)/(1-2*(q2*q2+q3*q3)))*180/PI;
    pitch = -asin(2*(q1*q3-q0*q2))*180/PI;
    yaw = atan(2*(q2*q3+q0*q1)/(1-2*(q2*q2+q1*q1)))*180/PI;
  }
