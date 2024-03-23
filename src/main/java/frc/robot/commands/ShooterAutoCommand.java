// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ShooterAutoCommand extends Command {
    /** Creates a new ShooterAutoCommand. */
    private final ShooterSubsystem _Shooter;
    private Timer _Timer;
    private double _RunTime = 0;
    private double _Power = 0;

    public ShooterAutoCommand(ShooterSubsystem shooter, double time, double power) {
        // Use addRequirements() here to declare subsystem dependencies.
        _Timer = new Timer();
        _RunTime = time;
        _Shooter = shooter;
        _Power = power;
        addRequirements(_Shooter);
    }
    public ShooterAutoCommand(ShooterSubsystem shooter, double time) {
        // Use addRequirements() here to declare subsystem dependencies.
        _Timer = new Timer();
        _RunTime = time;
        _Shooter = shooter;
        _Power = Constants.Shooter.HighSpeed;
        addRequirements(_Shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("started Shooter");
        _Shooter.off();
        if(_Timer.get() > 1){
            _Timer.reset();
            } else {
            _Timer.start();
            }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        _Shooter.autoPower(_Power);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _Shooter.off();
        _Timer.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (_Timer.get() > _RunTime){
            System.out.println("stopped Shooter");
            return true;
            }  else {
                return false;
            }
    }
}
