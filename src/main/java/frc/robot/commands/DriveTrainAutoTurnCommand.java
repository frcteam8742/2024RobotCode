// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import edu.wpi.first.math.proto.System;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.Devices.GyroSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class DriveTrainAutoTurnCommand extends Command {

    /** Creates a new DriveTrainCommand. */
    private DriveTrainSubsystem _Drive;
    private GyroSubsystem _Gyro;
    private double _currentAngle;
    private double _targetAngle;
    // private double _tolerance;
    private double _DeltaAngle;


    public DriveTrainAutoTurnCommand(DriveTrainSubsystem drive, GyroSubsystem gyro, double angle) {
        // Use addRequirements() here to declare subsystem dependencies.
        _Drive = drive;
        _Gyro = gyro;
        _targetAngle = angle;
        addRequirements(_Drive);
        addRequirements(_Gyro);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // initialize motors as off
        _currentAngle = 0;
        _Drive.setRightPower(0);
        _Drive.setLeftPower(0);
        // _tolerance = Constants.DriveTrain.Tolerance;
        System.out.println("Turning start");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        _currentAngle = _Gyro.getZ();
        double deltaAngle = _currentAngle - _targetAngle;
        // double deltaAngle = (_targetAngle - _currentAngle + 360) % 360;
        System.out.println("ANGLE:" + deltaAngle + "   THIS IS DELTA ANGLE");
        System.out.println(Math.abs(_DeltaAngle)+ "ABSOLOUTE VALUE OF IT");


        if (deltaAngle <= 0) {
            System.out.println("turning RIGHT");
            // _Drive.setLeftPower(.001*Math.abs(deltaAngle));
            // _Drive.setRightPower(-.001*Math.abs(deltaAngle));
            _Drive.setLeftPower(-.6);
            _Drive.setRightPower(.6);
        } else if (deltaAngle > 0) {
            System.out.println("turning LEFT");
            _Drive.setLeftPower(.6);
            _Drive.setRightPower(-.6);
            // _Drive.setLeftPower(-.001*(Math.abs(deltaAngle)));
            // _Drive.setRightPower(.001*(Math.abs(deltaAngle)));
        }
        _DeltaAngle = deltaAngle;
    }

    // public void turnPower(double targetAngle, double tolerance) {
    //     _targetAngle = targetAngle;
    //     _tolerance = tolerance; 
    // }
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
        if(Math.abs((_DeltaAngle)) < 5){
            // System.out.println(Math.abs((_currentAngle - _targetAngle)));
            // System.out.println(_currentAngle + " Our Angle");
            // System.out.println(_targetAngle + " Angle we want");
            return true;
        } else {
        System.out.println("Turn stopped");
        return false;
        }
    }
}
