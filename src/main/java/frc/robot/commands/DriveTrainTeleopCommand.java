// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveTrainTeleopCommand extends Command {

    /** Creates a new DriveTrainCommand. */
    private final DriveTrainSubsystem _Drive;

    // private Joystick _rightDriveJoystick;
    // private Joystick _leftDriveJoystick;
    private XboxController _tempDriverController;

    public DriveTrainTeleopCommand(DriveTrainSubsystem drive, XboxController tXboxController) {
    // public DriveTrainTeleopCommand(DriveTrainSubsystem drive, Joystick ljoystick, Joystick rjoystick) {

        // Use addRequirements() here to declare subsystem dependencies.
        _Drive = drive;
        // _leftDriveJoystick = ljoystick;
        // _rightDriveJoystick = rjoystick;
        _tempDriverController = tXboxController;
        addRequirements(_Drive);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // initialize motors as off
        _Drive.setRightPower(0);
        _Drive.setLeftPower(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // the right and left Flight Sticks tie to the right and left drive motors
        // respectively

        // The commented code here is for if we would like to have a turbo button and
        // turn down the speed of the regular movement. The only thing that would need
        // to be changed is setting (inside the else statement) the speeds to lower
        // values

        // if(_rightDriveJoystick.getRawButton(3)){
        // _Drive.setRightPower(-1*_rightDriveJoystick.getY());
        // _Drive.setLeftPower(-1*_leftDriveJoystick.getY());
        // } else {
        // _Drive.setRightPower(-.5 * _rightDriveJoystick.getY());
        // _Drive.setLeftPower(-.5 * _leftDriveJoystick.getY());
        // }

        _Drive.setRightPower(_tempDriverController.getRightY()/1.5);
        _Drive.setLeftPower(-_tempDriverController.getLeftY()/1.5);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _Drive.setRightPower(0);
        _Drive.setLeftPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
