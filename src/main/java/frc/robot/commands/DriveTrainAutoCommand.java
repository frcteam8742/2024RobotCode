// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class DriveTrainAutoCommand extends Command {

    /** Creates a new DriveTrainCommand. */
    private final DriveTrainSubsystem _Drive;
    private double _CurrentTime = 0;
    private double _InitialTime = 0;
    public DriveTrainAutoCommand(DriveTrainSubsystem drive) {
        // Use addRequirements() here to declare subsystem dependencies.
        _Drive = drive;
        addRequirements(_Drive);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // initialize motors as off
        _CurrentTime = 0;
        _InitialTime = System.currentTimeMillis();
        _Drive.setRightPower(0);
        _Drive.setLeftPower(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        _CurrentTime = System.currentTimeMillis() - _InitialTime;
        System.out.println(_CurrentTime);
        // _Time = Timer.getFPGATimestamp();
        // the right and left Flight Sticks tie to the right and left drive motors
        // respectively
    }

    public void setDrivePID(double PIDSpeedR, double PIDSpeedL) {
        _Drive.setRightSpeed(PIDSpeedR);
        _Drive.setLeftSpeed(PIDSpeedL);

    }

    // lwkdnawfm
        // if it has reached time, stop what you are doing
        // or we can write a command that uses seconds to declare that the command is finsihed with the waitcommand


    public void setPower(double rSpeed, double lSpeed) {
        _Drive.setRightPower(rSpeed);
        _Drive.setLeftPower(lSpeed);
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
