// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
    /** Creates a new DriveTrainSubsystem. */

    private PWMSparkMax _leftDrive1 = new PWMSparkMax(Constants.DriveTrain.LeftMotor1);
    private PWMSparkMax _leftDrive2 = new PWMSparkMax(Constants.DriveTrain.LeftMotor1);
    private PWMSparkMax _rightDrive1 = new PWMSparkMax(Constants.DriveTrain.LeftMotor1);
    private PWMSparkMax _rightDrive2 = new PWMSparkMax(Constants.DriveTrain.LeftMotor1);

    private double _rightPower = 0;
    private double _leftPower = 0;

    public DriveTrainSubsystem() {
        _leftDrive1.addFollower(_leftDrive2);
        _rightDrive1.addFollower(_rightDrive2);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        _leftDrive1.set(_leftPower);
        _rightDrive1.set(_rightPower);
    }

    public void setRightPower(double rightPower) {
        _rightPower = rightPower;
    }

    public void setLeftPower(double leftPower) {
        _leftPower = leftPower;

    }
}
