#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "MPU9250_asukiaaa.h"
#include <math.h>

struct Vector {
    float x, y, z;

    // Constructor
    Vector(float _x = 0, float _y = 0, float _z = 0) : x(_x), y(_y), z(_z) {}

    // Addition operator
    Vector operator+(const Vector& v) const {
        return Vector(x + v.x, y + v.y, z + v.z);
    }

    // Subtraction operator
    Vector operator-(const Vector& v) const {
        return Vector(x - v.x, y - v.y, z - v.z);
    }

    // Scalar multiplication
    Vector operator*(float scalar) const {
        return Vector(x * scalar, y * scalar, z * scalar);
    }

    Vector operator/(const Vector& v) const { 
      return Vector(x / v.x, y / v.y, z / v.z); 
    } // Element-wise division

    // Dot product
    float dot(const Vector& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    // Cross product
    Vector cross(const Vector& v) const {
        return Vector(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }

    // Magnitude of the vector
    float magnitude() const {
        return sqrt(x * x + y * y + z * z);
    }

    // Normalize the vector
    Vector normalize() const {
        float mag = magnitude();
        return (mag > 0) ? (*this * (1.0 / mag)) : Vector(0, 0, 0);
    }

    // Print the vector
    void print() const {
        Serial.print("(");
        Serial.print(x);
        Serial.print(", ");
        Serial.print(y);
        Serial.print(", ");
        Serial.print(z);
        Serial.println(")");
    }
};

// Initializing sensors objects 
MPU9250_asukiaaa mpu1(0x68);  // MPU9250 #1 (AD0 tied to GND)
MPU9250_asukiaaa mpu2(0x69);  // MPU9250 #2 (AD0 tied to 3.3V)
Adafruit_BMP280 bmp;

//initialization variables
// 0 - Initialization
// 1 - Armed
// 2 - Powered Ascent
// 3 - Gliding
// 4 - Descent
// 5 - Airbrake
// 6 - Parachute Deployed
// 7 - Landed (All safe)
int status = 0;
float ParachuteHeight = 30.0;
float AirbrakeHeight = 80.0;

//Linear Movement Variables
Vector EulerPosition(0,0,0);
Vector EulerSpeed(0,0,0);
Vector RocketSpeed(0,0,0);
Vector EulerAcceleration(0,0,0);
Vector RocketAcceleration(0,0,0);

//angular terms
Vector GyroAcceleration(0,0,0);
Vector GyroSpeed(0,0,0);
Vector GyroOrientation(0,0,0);

//Navigation Terms
Vector InitialGPS(0,0,0);
unsigned long gps_ti;
unsigned long bmp_ti;

//guidance terms
Vector Correction(0,0,0);
int GuidanceMode = 0; //0 - Straight up/down; 1-Waypoint
int dt = 5;
static double theta_prev = 0.0;  // Retains value between calls
static double psi_prev = 0.0;    // Retains value between calls
float total_error_x = 0.0;
float total_error_y = 0.0;
float obs = 0.0;

// Waypoints for waypoint guidance mode
const int NUM_WAYPOINTS = 4;
Vector waypoints[NUM_WAYPOINTS] = {{0.0, 0.0, 100.0}, {10.0, 5.0, 200.0}, {0.0, 0.0, 300.0}, {0.0,0.0,80.0}};
int currentWaypoint = 0;
int DeployAirBrake = 0;

//other terms
float Apogee = 0;
int Error = 0;



void setup() {
    Serial.begin(115200);
}

void arm() {
  // put your setup code here, to run once:
  Wire.begin();  // Start I²C

  // Initialize MPU9250 #1
  mpu1.setWire(&Wire);
  mpu1.beginAccel();
  mpu1.beginGyro();
  Serial.println("MPU9250 #1 initialized");

  // Initialize MPU9250 #2
  mpu2.setWire(&Wire);
  mpu2.beginAccel();
  mpu2.beginGyro();
  Serial.println("MPU9250 #2 initialized");

  // Initialize BMP280
  if (!bmp.begin(0x76)) {  // Use 0x77 if SDO is tied to 3.3V
      Serial.println("BMP280 initialization failed! Check wiring.");
      while (1);
      Error += 1;
  }
  Serial.println("BMP280 initialized successfully!");
  //
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   // Continuous mode
                    Adafruit_BMP280::SAMPLING_X1,  // Temp oversampling x1
                    Adafruit_BMP280::SAMPLING_X1,  // Pressure oversampling x1
                    Adafruit_BMP280::FILTER_OFF,   // No IIR filter
                    Adafruit_BMP280::STANDBY_MS_1);  // Standby time 5ms (~100Hz)

  //Kalman Terms
  gps_ti = millis();
  bmp_ti = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  //Initialization
  while(status == 0){
    //communicate serially with the computer through GUI
    //write to major updates file with timestamp and position "INITIALIZED"
    if(){//arming sequence
      //write to major updates file with timestamp and position "ARMING SEQUENCE DETECTED"
      arm();
      if(){//do the self check of the rocket and if it returns true, ARM the rocket
        status = 1;
        //write to major updates file with timestamp and position "CHECKS DONE. ARMING SEQUENCE SUCCESSFUL."
      }
      else {
        //print out the possible error
      }
    }
  }
  //Armed
  while(status == 1) {
    //activate TVC
    KalmanUpdate(); //Updates positions
    ComputeGuidance();
    RollFins();
    TVCAdjust();
    WriteTelemetry();
    if(){//pin read high from launcher, or, detect certain acceleration for a certain time
      status = 2;
      //write to major updates file with timestamp and position "IGNITION DETECTED"
    }
    delay(dt); //5ms cycles 200Hz
    //write to Arduino
    //write to Flash to teensy
  }
  //Powered Launch
  while(status == 2) {
    //activate TVC
    KalmanUpdate(); //Updates positions
    ComputeGuidance();
    RollFins();
    TVCAdjust();
    WriteTelemetry();
    if(){// we detect that we are no longer accelerating
      status = 3;
      //write to major updates file with timestamp and position "BURNOUT DETECTED"
    }
    delay(dt);
  }
  //Gliding
  while(status == 3) {
    KalmanUpdate();
    ComputeGuidance();
    AdjustFins();
    WriteTelemetry();
    if(){//apogee detected (height max and current less) and velocity = 0
      status = 4;
      UpdateApogee();
      UpdateDescentWaypoint();
      //write to major updates file with timestamp and position "APOGEE DETECTED"
    }
    delay(dt);
  }
  //Descent
  while(status == 4) {
    KalmanUpdate();
    ComputeGuidance();
    AdjustFins();
    WriteTelemetry();
    if(DeployAirBrake == 1) {//at waypoint
      //Deploy Drag Brake
      status = 5;
      DeployAirbrake();
      UpdateParachuteHeight();
      //write to major updates file with timestamp and position "AIRBRAKE ACTIVATED"
    }
    delay(dt);
  }
  //Airbrake
  while(status == 5) {
    KalmanUpdate();
    WriteTelemetry();
    if(height == ParachuteHeight) {
      parachute_deploy();
      status = 6
      //write to major updates file with timestamp and position "PARACHUTE DEPLOYED"
    }
    delay(dt);
  }
}

Vector P[7] = { 
    Vector(1, 1, 1), Vector(1, 1, 1), Vector(1, 1, 1), 
    Vector(1, 1, 1), Vector(1, 1, 1), Vector(1, 1, 1), Vector(1, 1, 1)
};//covariance matrix for each state
Vector Q(0.01, 0.01, 0.01);  // Process noise
Vector R_GPS(5, 5, 5);       // Measurement noise for GPS
Vector R_IMU(0.1, 0.1, 0.1); // Measurement noise for IMU

void KalmanUpdate() {
  //Initialization
  
  //Prediction
  Vector EulerPositionPredict = EulerPosition + (EulerSpeed * dt) + (EulerAcceleration * (0.5 * dt * dt));
  Vector EulerSpeedPredict = EulerSpeed + (EulerAcceleration * dt);
  Vector RocketSpeedPredict = RocketSpeed + (RocketAcceleration *dt);

  Vector GyroOrientationPredict = GyroOrientation + (GyroSpeed * dt) + (GyroAcceleration * (0.5 * dt * dt));
  Vector GyroSpeedPredict = GyroSpeed + (GyroAcceleration * dt);


  //Getting sensor values
  // Averaging for 2xMPU
  // Euler(z) = MPU(X)
  // Euler(y) = MPU(Y)
  // Euler(x) = MPU(-Z)
  Vector mpu1acc(-(mpu1.accelZ()), mpu1.accelY(), mpu1.accelX());
  Vector mpu2acc(-(mpu2.accelZ()), mpu2.accelY(), mpu2.accelX());
  Vector mpuacc = (mpu1acc + mpu2acc) * 0.5;

  Vector mpu1gyro(-(mpu1.gyroZ()), mpu1.gyroY(), mpu1.gyroZ());
  Vector mpu2gyro(-(mpu2.gyroZ()), mpu2.gyroY(), mpu2.gyroZ());
  Vector mpugyro = (mpu1gyro + mpu2gyro) * 0.5;

  unsigned long gps_c = millis();
  unsigned long bmp_c = millis();
  if ((bmp_c - bmp_ti) >= 20) {
    float bmp_Altitude = bmp.readAltitude(1013.25);
    bmp_ti = bmp_c;
  }
  if ((gps_c - gps_ti) >= 100) {
    float gps_x;
    float gps_y;
    float gpsxdot = getGpsVelocities('x');
    float gpdydot = getGpsVelocities('y');
  }
  
  
  //Measure part:
  //Gyrospeed we will directly get from measurement (Apply complementary filter)
  //GyroOrientation we will get directly (Apply Complementary Filter)
  //Rocket Acceleration aajayegi (Complementary filter) - no need for sensor fusion
  //Rocket speeds we can get from that
  //Euler Acceleration we can convert from RocketAcceleration to Euler Frame
  //Euler Speeds and Positions from Acceleration
  //Measured part
  Vector GyroSpeedMeasured = mpugyro;
  Vector GyroOrientationMeasured; //complementary filter with IMU & Gyro

  Vector EulerPositionMeasured; // When GPS comes in (BMP every 50ms, Updates with BMP evry 2 cycles, Updates with GPS every 20 cycles)
  Vector EulerSpeedMeasured; // GPS comes in (BMP every 50ms, updates with BMP every 2 cycles, Updates with GPS every 20 cycles)
  Vector RocketSpeedMeasured; //Not measured. Only from IMU and reverse transformation of GPS speeds

  
}

void ComputeGuidance() {
    //initializing terms for guidance
    Vector Target;
    
    if (GuidanceMode == 0) {
        // Target is just the vertical trajectory
        Target.x = 0;
        Target.y = 0;
        Target.z = EulerPosition.z + 1; // Just ensure we always move upwards
    } 
    else if (GuidanceMode == 1) {
        // Target is the next waypoint
        /*x_target = waypoints[currentWaypoint].x;
        y_target = waypoints[currentWaypoint].y;
        z_target = waypoints[currentWaypoint].z;*/
        Vector Target = waypoints[currentWaypoint];

        // Check if we reached the waypoint
        float dist = (Target - EulerPosition).magnitude()
        if (dist < 5) { // 5m tolerance
            if (currentWaypoint >= NUM_WAYPOINTS - 1) {
                GuidanceMode = 0;
            }
            else {
              currentWaypoint++;
            }
            
        }
    }

    // Compute position error
    float error_x = EulerPosition.x - Target.x;
    float error_y = EulerPosition.y - Target.y;
    total_error_x += error_x;
    total_error_y += error_y;
    obs += 1.0;

    
    // Compute velocity direction to the target
    float theta_desired = atan2(EulerSpeed.x, EulerSpeed.z); // Pitch error
    float psi_desired = atan2(EulerSpeed.y, EulerSpeed.z);   // Yaw error

    // Compute rate of change of error
    float theta_error = theta_desired;
    float psi_error = psi_desired;
    
    float theta_dot = (theta_error - theta_prev) / (dt/1000);
    float psi_dot = (psi_error - psi_prev) / (dt/1000);

    theta_prev = theta_error;
    psi_prev = psi_error;

    // Proportional Navigation Law (Corrective acceleration)
    float N = 3.0; // Gain for guidance
    Correction.x = -N * theta_dot * EulerSpeed.z - error_x;
    Correction.y = -N * psi_dot * EulerSpeed.z - error_y;
    Correction.z = 0; // No correction in vertical
}

void UpdateDescentWaypoint() {
  float wind_x = (total_error_x/obs) * 0.6;//avg error per m in xe due to wind
  float wind_y = (total_error_y/obs) * 0.6;//avg error per m in ye due to wind
  float descent_distance = Apogee - AirbrakeHeight;
  //error racked up between that distance
  float x_compensation = -0.9 * (descent_distance * wind_x);
  float y_compensation = -0.9 * (descent_distance * wind_y);
  waypoints[3].x = x_compensation;
  waypoints[3].y = y_compensation;
  waypoints[3].z = AirbrakeHeight;
}

void DeployAirbrake() {
  //set all fins to 90
}