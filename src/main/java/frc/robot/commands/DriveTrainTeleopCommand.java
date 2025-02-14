// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class DriveTrainTeleopCommand extends Command {

    /** Creates a new DriveTrainCommand. */
    private final DriveTrainSubsystem _Drive;

    private Joystick _rightDriveJoystick;
    private Joystick _leftDriveJoystick;

    public DriveTrainTeleopCommand(DriveTrainSubsystem drive, Joystick ljoystick, Joystick rjoystick) {
        // Use addRequirements() here to declare subsystem dependencies.
        _Drive = drive;
        _leftDriveJoystick = ljoystick;
        _rightDriveJoystick = rjoystick;
        addRequirements(_Drive);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        _Drive.setRightPower(0);
        _Drive.setLeftPower(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        _Drive.setRightPower(_rightDriveJoystick.getY());
        _Drive.setLeftPower(_leftDriveJoystick.getY());
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
