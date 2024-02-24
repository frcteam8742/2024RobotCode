// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Devices;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

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
        Gyro_z = imu.getAngle(null);
    }
    public void reset() {
        imu.reset();
    }
}
