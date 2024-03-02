// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.Devices.GyroSubsystem;

public class DriveTrainAutoTurnCommand extends Command {

    /** Creates a new DriveTrainCommand. */
    private final DriveTrainSubsystem _Drive;
    private final GyroSubsystem _Gyro;
    private double _currentAngle;
    private double _targetAngle;
    private double _tolerance;

    public DriveTrainAutoTurnCommand(DriveTrainSubsystem drive, GyroSubsystem gyro) {
        // Use addRequirements() here to declare subsystem dependencies.
        _Drive = drive;
        _Gyro = gyro;
        addRequirements(_Drive);
        addRequirements(_Gyro);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // initialize motors as off
        _Drive.setRightPower(0);
        _Drive.setLeftPower(0);
        _Gyro.reset();
        _currentAngle = 0;
        _targetAngle = 0;
        _tolerance = Constants.DriveTrain.Tolerance;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        _currentAngle = _Gyro.getZ();
        double deltaAngle = _currentAngle - _targetAngle;
        if (deltaAngle >= 0) {
            _Drive.setLeftPower(-.05*deltaAngle);
            _Drive.setRightPower(.05*deltaAngle);
        }
        if (_currentAngle - _targetAngle < 0) {
            _Drive.setLeftPower(.05*deltaAngle);
            _Drive.setRightPower(-.05*deltaAngle);
        }
    }

    public void turnPower(double targetAngle, double tolerance) {
        _targetAngle = targetAngle;
        _tolerance = tolerance; 
    }
        // set left power until angle enters tolerance


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _Drive.setRightPower(0);
        _Drive.setLeftPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(Math.abs((_currentAngle - _targetAngle)) > _tolerance){
            return true;
        } else {
        return false;
        }
    }
}
