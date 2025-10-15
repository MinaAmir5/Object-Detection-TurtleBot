#include "Arduino.h"
#include "MPU9250.h"


#define INTERVAL 50


double X = 0;
double Y = 0;
float  Theta = 0;


MPU9250 mpu;

void resetMPU9250()
{
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x80); // reset command 
    Wire.endTransmission();
    delay(100); //wait for the reset to complete 
}

void MPU_Init() 
{
    Wire.begin();         // Start I2C communication

    // Try to initialize MPU9250 with its I2C address (default 0x68)
    if (!mpu.setup(0x68)) { 
        while (1) {
            Serial.println("MPU connection failed."); // Print error if connection fails
            delay(5000);                             // Wait 5 seconds before retrying
        }
    }
    Serial.println("MPU connection success."); 

    // Calibrate the sensors
    mpu.verbose(true);          // Enable verbose output for debugging
    delay(5000);                // Wait for 5 seconds
    mpu.calibrateAccelGyro();   // Calibrate accelerometer and gyroscope
    delay(5000);                // Wait for another 5 seconds
    mpu.calibrateMag();         // Calibrate the magnetometer
    mpu.verbose(false);         // Disable verbose output
}

// Function to print orientation (Yaw, Pitch, Roll)
void print_Orientation_3D() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
}


 
// Function to print magnetometer readings
void print_Mag_3D() 
{
    Serial.print("MAGX, MAGY, MAGZ: ");
    Serial.print(mpu.getMagX(), 2);
    Serial.print(", ");
    Serial.print(mpu.getMagY(), 2);
    Serial.print(", ");
    Serial.println(mpu.getMagZ(), 2);

}



// Function to print accelerometer readings
void print_Accel_3D() {
    Serial.print("AccX, AccY, AccZ: ");
    Serial.print(mpu.getAccX(), 2);
    Serial.print(", ");
    Serial.print(mpu.getAccY(), 2);
    Serial.print(", ");
    Serial.println(mpu.getAccZ(), 2);
}


// Function to print gyroscope readings
void print_Gyro_3D() {
    Serial.print("GyroX, GyroY, GyroZ: ");
    Serial.print(mpu.getGyroX(), 2);
    Serial.print(", ");
    Serial.print(mpu.getGyroY(), 2);
    Serial.print(", ");
    Serial.println(mpu.getGyroZ(), 2);
}



void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
}



void Get_Position (void)
{
    double ax=0, ay=0 , az=0;
    double vx=0, vy=0;
    //double current_ax=0;
    //double current_ay=0;
    //static double prev_ax=0;
    //static double prev_ay=0;


   
    

    //Get the linear acceleraation from the MPU
    ax = (mpu.getAccX() * 9.81 * 100);
 
    ay = (mpu.getAccY() * 9.81 * 100);
   
    //az = (mpu.getAccZ() * 9.81 * 100);

     
    

 // Apply noise threshold to acceleration
    if (abs(ax) < 50) ax = 0;
    if (abs(ay) < 13) ay = 0;



    //calculatin the linear velocities
    vx += (( (ax) * (INTERVAL/1000.0) ));
    vy += (( (ay) * (INTERVAL/1000.0) ));




    //Serial.print("ax, ay, az: ");
    //Serial.print(vx, 2);
    //Serial.print(", ");
    //Serial.println(vy, 2);
 

// Compensate for drift
    if (ax == 0 && ay == 0) 
    {
        vx = 0;
        vy = 0;
    }

    //Get the position
    X += ( vx * (INTERVAL/1000.0) );
    Y += ( vy * (INTERVAL/1000.0) );
   


    // Apply noise threshold to acceleration
    
    //Get the theta
    Theta = mpu.getYaw();
}




void print_Position (void)
{
    Serial.print("X, Y, theta: ");
    Serial.print(X, 2);
    Serial.print(", ");
    Serial.print(Y, 2);
    Serial.print(", ");
    Serial.println(Theta, 2);

}