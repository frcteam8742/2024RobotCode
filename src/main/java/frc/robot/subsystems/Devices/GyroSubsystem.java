// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Devices;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GyroSubsystem extends SubsystemBase {
    /** Creates a new Gyro. */
    private ADIS16470_IMU imu = new ADIS16470_IMU();
    private double Gyro_x = 0.0;
    private double Gyro_y = 0.0;
    private double Gyro_z = 0.0;

    public GyroSubsystem() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        
        Gyro_x = imu.getXComplementaryAngle();
        Gyro_y = imu.getYComplementaryAngle();



        // This is the axis that we will use, will have to have it reset every time it
        // does an action and every time it is switched to teleop because it goes over
        // past 360. Should be able to (in the DriveTrainAutoCommand) set it to where it
        // turns until this reaches 45, or whatever angle, reset the value, then do
        // whatever it needs to
        Gyro_z = imu.getAngle(imu.getRollAxis());

        // SmartDashboard.putNumber("X axis (Roll)", Gyro_x);
        // SmartDashboard.putNumber("Y axis (Pitch)", Gyro_y);
        // SmartDashboard.putNumber("Z axis (Yaw)", Gyro_z);
    }

    public double getX() {
        return Gyro_x;
    }

    public double getY() {
        return Gyro_y;
    }

    public double getZ() {
        if(Gyro_z < 0) {
        return 360 + (Gyro_z % 360);
        } else {
        return Gyro_z % 360;
        }
    }

    public void reset() {
        imu.reset();
    }
}
