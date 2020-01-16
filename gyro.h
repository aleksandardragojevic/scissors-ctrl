//
// Gyroscope support.
//
// author: aleksandar
//

#pragma once

#include <Wire.h>

template<int Mpu = 68>
class Gyro {
public:
    static Gyro &Instance() {
        return instance;
    }

    void Setup() {
        Wire.beginTransmission(Mpu);
        Wire.write(0x6B); 
        Wire.write(0);    
        Wire.endTransmission(true);
    }

    void Read() {
        Wire.beginTransmission(Mpu);
        Wire.write(0x3B);  
        Wire.endTransmission(false);
        Wire.requestFrom(Mpu, 14, true);

        //Acceleration data correction
        AcXcal = -950;
        AcYcal = -300;
        AcZcal = 0;

        //Temperature correction
        tcal = -1600;

        //Gyro correction
        GyXcal = 480;
        GyYcal = 170;
        GyZcal = 210;
  
        AcX=Wire.read()<<8|Wire.read();    
        AcY=Wire.read()<<8|Wire.read();  
        AcZ=Wire.read()<<8|Wire.read(); 

        Tmp=Wire.read()<<8|Wire.read();  
        GyX=Wire.read()<<8|Wire.read();  
        GyY=Wire.read()<<8|Wire.read();  
        GyZ=Wire.read()<<8|Wire.read();  
  
        tx = Tmp + tcal;
        t = tx/340 + 36.53; //equation for temperature in degrees C from datasheet
        tf = (t * 9/5) + 32; //fahrenheit

        //get pitch/roll
        getAngle(AcX, AcY, AcZ);
    }

    void Print() {
        Serial.print("Angle: ");
        Serial.print("Pitch = "); Serial.print(pitch);
        Serial.print(" Roll = "); Serial.println(roll);
    
        Serial.print("Accelerometer: ");
        Serial.print("X = "); Serial.print(AcX + AcXcal);
        Serial.print(" Y = "); Serial.print(AcY + AcYcal);
        Serial.print(" Z = "); Serial.println(AcZ + AcZcal); 

        Serial.print("Temperature in celsius = "); Serial.print(t);  
        Serial.print(" fahrenheit = "); Serial.println(tf);  
    
        Serial.print("Gyroscope: ");
        Serial.print("X = "); Serial.print(GyX + GyXcal);
        Serial.print(" Y = "); Serial.print(GyY + GyYcal);
        Serial.print(" Z = "); Serial.println(GyZ + GyZcal);
    }

private:
    void getAngle(int Ax,int Ay,int Az) 
    {
        double x = Ax;
        double y = Ay;
        double z = Az;

        pitch = atan(x/sqrt((y*y) + (z*z))); //pitch calculation
        roll = atan(y/sqrt((x*x) + (z*z))); //roll calculation
        yaw = atan(z/sqrt(x*x + z*z));

        //converting radians into degrees
        pitch = pitch * (180.0/3.14);
        roll = roll * (180.0/3.14) ;
        yaw = yaw * (180.0/3.14) ;
    }

    int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
    int AcXcal,AcYcal,AcZcal,GyXcal,GyYcal,GyZcal,tcal;
    double t,tx,tf,pitch,roll,yaw;

    static Gyro instance;
};

template<int Mpu>
Gyro<Mpu> Gyro<Mpu>::instance;
