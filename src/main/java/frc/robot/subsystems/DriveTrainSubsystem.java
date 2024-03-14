// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.proto.System;
// import frc.robot.subsystems.Devices.DriveTrainEncoder;

import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
    /** Creates a new DriveTrainSubsystem. */

    // Right PID Initializers
    private double _RP = 0.0;
    private double _RI = 0.0;
    private double _RD = 0.0;

    // Left PID Initializers
    private double _LP = 0.0;
    private double _LI = 0.0;
    private double _LD = 0.0;

    private PWMSparkMax _leftDrive1 = new PWMSparkMax(Constants.DriveTrain.LeftMotor1);
    private PWMSparkMax _leftDrive2 = new PWMSparkMax(Constants.DriveTrain.LeftMotor2);
    private PWMSparkMax _rightDrive1 = new PWMSparkMax(Constants.DriveTrain.RightMotor1);
    private PWMSparkMax _rightDrive2 = new PWMSparkMax(Constants.DriveTrain.RightMotor2);
    private PIDController _rightPid = new PIDController(_RP, _RI, _RD);
    private PIDController _leftPid = new PIDController(_LP, _LI, _LD);
    // private DriveTrainEncoder _leftDriveEncoder = new DriveTrainEncoder(Constants.Sensors.LeftDriveEncoder);
    // private DriveTrainEncoder _rightDriveEncoder = new DriveTrainEncoder(Constants.Sensors.RightDriveEncoder);

    private double _rightPower = 0;
    private double _leftPower = 0;
    private double _rightSpeed = 0;
    private double _leftSpeed = 0;

    private boolean _DrivePIDControlled = false;

    public DriveTrainSubsystem() {
        // sets the motors on each side

        // Right PID Values
        _RP = 0.005;
        _RI = 0.0;
        _RD = 0.0;
        // Left PID Values
        _LP = 0.005;
        _LI = 0.0;
        _LD = 0.0;

        _leftDrive1.addFollower(_leftDrive2);
        _rightDrive1.addFollower(_rightDrive2);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (!_DrivePIDControlled) {
            // Teleop just dumps power
            _leftDrive1.set(-_leftPower);
            _rightDrive1.set(_rightPower);
        } else {
            // Auto uses PID
            // _rightDrive1.set(_rightPid.calculate(_rightDriveEncoder.getSpeed(),
            //         _rightSpeed));
            // _leftDrive1.set(_leftPid.calculate(_leftDriveEncoder.getSpeed(),
            //         _leftSpeed));
        }
    }

    public void setRightPower(double rightPower) {
        _DrivePIDControlled = false;
        _rightPower = rightPower;
    }

    public void setLeftPower(double leftPower) {
        _DrivePIDControlled = false;
        _leftPower = leftPower;
    }

    public void setRightSpeed(double rightSpeed) {
        _DrivePIDControlled = true;
        _rightSpeed = rightSpeed;
    }

    public void setLeftSpeed(double leftSpeed) {
        _DrivePIDControlled = true;
        _leftSpeed = leftSpeed;
    }


}
