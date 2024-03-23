// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class IndexerAutoCommand extends Command {
    /** Creates a new IndexerAutoCommand. */
    private final IndexerSubsystem _Indexer;
    private Timer _Timer;
    private double _RunTime = 0;
    private double _Power = 0;

    public IndexerAutoCommand(IndexerSubsystem Indexer, double time, double power) {
        // Use addRequirements() here to declare subsystem dependencies.
        _Timer = new Timer();
        _Power = power;
        _RunTime = time;
        _Indexer = Indexer;
        addRequirements(_Indexer);

    }
    public IndexerAutoCommand(IndexerSubsystem Indexer, double time) {
        // Use addRequirements() here to declare subsystem dependencies.
        _Timer = new Timer();
        _Power = -1;
        _RunTime = time;
        _Indexer = Indexer;
        addRequirements(_Indexer);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("started Indexer");
        _Indexer.autoPower(0);
         if(_Timer.get() > 1){
            _Timer.reset();
            } else {
            _Timer.start();
            }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        _Indexer.autoPower(_Power);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _Indexer.autoPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (_Timer.get() > _RunTime){
            System.out.println("stopped Indexer");
            return true;
            }  else {
                return false;
            }
    }
}
