// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveTrainAutoCommand extends Command {

    /** Creates a new DriveTrainCommand. */
    private DriveTrainSubsystem _Drive;
    private Timer _Timer;
    private double _Power;
    private double _RunTime = 0;

    public DriveTrainAutoCommand(DriveTrainSubsystem drive, double power, double time) {
        _Timer = new Timer();
        _Drive = drive;
        _RunTime = time;
        _Power = power;
        addRequirements(_Drive);
    }

    @Override
    public void initialize() {
        _Timer.reset();
        _Timer.start();
    }

    @Override
    public void execute() {
        // System.out.println(_Timer.get());
        _Drive.setRightPower(_Power);
        _Drive.setLeftPower(_Power);
    }

    @Override
    public void end(boolean interrupted) {
        _Drive.setRightPower(0);
        _Drive.setLeftPower(0);
    }

    @Override
    public boolean isFinished() {
        if (_Timer.get() > _RunTime){
        System.out.println("stopped");
        return true;
        }  else {
            return false;
        }
    }
}
