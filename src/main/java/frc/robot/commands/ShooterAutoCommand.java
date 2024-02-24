// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterAutoCommand extends Command {
  /** Creates a new ShooterAutoCommand. */
  private final ShooterSubsystem _Shooter;
  public ShooterAutoCommand(ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
  _Shooter = shooter;
  addRequirements(_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  _Shooter.off();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  public void setShooterPID(double PIDSpeed){
    //create
  }

  public void setPower(double power){
    _Shooter.autoPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _Shooter.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
