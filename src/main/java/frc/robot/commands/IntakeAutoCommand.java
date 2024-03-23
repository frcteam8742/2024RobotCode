// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;


public class IntakeAutoCommand extends Command {
  private final IntakeSubsystem _Intake;
  private Timer _Timer;
  private double _RunTime = 0;
  private double _Power;

  /** Creates a new IntakeAutoCommand. */
  public IntakeAutoCommand(IntakeSubsystem intake, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    _Timer = new Timer();
    _RunTime = time;
    _Intake = intake;
    _Power = -0.3;
    addRequirements(_Intake);
  }

  /** Creates a new IntakeAutoCommand. */
  public IntakeAutoCommand(IntakeSubsystem intake, double time, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    _Timer = new Timer();
    _RunTime = time;
    _Intake = intake;
    _Power = power;
    addRequirements(_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("started Intake");

     _Intake.on(0);
        if(_Timer.get() > 1){
            _Timer.reset();
            } else {
            _Timer.start();
            }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _Intake.on(_Power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     _Intake.on(0);
     _Timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (_Timer.get() > _RunTime){
      System.out.println("stopped Intake");
      return true;
      }  else {
          return false;
      }
  }
}
