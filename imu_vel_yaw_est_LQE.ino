#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// State space representation for Kalman filter
double dt = 0.01; // Time step

// State vector [angle, bias]
double xX[2] = {0, 0};
double xY[2] = {0, 0};

// State transition matrix
double A[2][2] = {{1, -dt}, {0, 1}};

// Control input matrix
double B[2] = {dt, 0};

// Measurement matrix
double H[2] = {1, 0};

// Process noise covariance
double Q[2][2] = {{0.001, 0}, {0, 0.003}};

// Measurement noise covariance
double R = 0.03;

// Error covariance matrix
double P[2][2] = {{1, 0}, {0, 1}};

// Storage for intermediate calculations
double S;
double K[2];
double y;
double P_temp[2][2];

double velocityX = 0;
double velocityY = 0;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  mpu.initialize();

  // Verify connection
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  
  Serial.println("MPU6050 connection successful");
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  // Read raw accelerometer and gyroscope data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  double accelX = ax / 16384.0; // Convert to g
  double accelY = ay / 16384.0;
  double gyroX = gx / 131.0;   // Convert to degrees/sec
  double gyroY = gy / 131.0;

  // Kalman filter for angle X
  kalmanFilter(xX, P, accelX, gyroX);

  // Kalman filter for angle Y
  kalmanFilter(xY, P, accelY, gyroY);

  // Integrate acceleration to get velocity (simple integration)
  velocityX += accelX * 9.81 * dt; // Convert g to m/s^2
  velocityY += accelY * 9.81 * dt;

  // Print results
  Serial.print("Angle X: "); Serial.print(xX[0]);
  Serial.print("\tAngle Y: "); Serial.print(xY[0]);
  Serial.print("\tVelocity X: "); Serial.print(velocityX);
  Serial.print("\tVelocity Y: "); Serial.println(velocityY);

  delay(10); // Delay for a short period
}

void kalmanFilter(double x[2], double P[2][2], double accel, double gyro) {
  // Predict
  double rate = gyro - x[1];
  x[0] += B[0] * rate;
  x[1] += B[1] * rate;

  P_temp[0][0] = A[0][0] * P[0][0] + A[0][1] * P[1][0];
  P_temp[0][1] = A[0][0] * P[0][1] + A[0][1] * P[1][1];
  P_temp[1][0] = A[1][0] * P[0][0] + A[1][1] * P[1][0];
  P_temp[1][1] = A[1][0] * P[0][1] + A[1][1] * P[1][1];

  P[0][0] = P_temp[0][0] + Q[0][0];
  P[0][1] = P_temp[0][1] + Q[0][1];
  P[1][0] = P_temp[1][0] + Q[1][0];
  P[1][1] = P_temp[1][1] + Q[1][1];

  // Update
  S = P[0][0] + R;
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  y = accel - x[0];
  x[0] += K[0] * y;
  x[1] += K[1] * y;

  P_temp[0][0] = P[0][0];
  P_temp[0][1] = P[0][1];
  P[0][0] -= K[0] * P_temp[0][0];
  P[0][1] -= K[0] * P_temp[0][1];
  P[1][0] -= K[1] * P_temp[0][0];
  P[1][1] -= K[1] * P_temp[0][1];
}
