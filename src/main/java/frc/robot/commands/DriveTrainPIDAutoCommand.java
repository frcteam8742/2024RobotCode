// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveTrainPIDAutoCommand extends Command {
  /** Creates a new DriveTrainPIDAutoCommand. */
  private DriveTrainSubsystem _Drive;
  private Timer _Timer;
  private double _Speed;
  private double _RunTime = 0;

  public DriveTrainPIDAutoCommand(DriveTrainSubsystem drive, double speed, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    _Timer = new Timer();
    _Drive = drive;
    _RunTime = time;
    _Speed = speed;
    addRequirements(_Drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _Timer.reset();
    _Timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _Drive.setRightSpeed(_Speed);
    _Drive.setLeftSpeed(_Speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _Drive.setRightSpeed(0);
    _Drive.setLeftSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (_Timer.get() > _RunTime){
      System.out.println("Drive (PID) stopped");
      return true;
      } else {
        return false;
  }
}
}
