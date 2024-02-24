// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerAutoCommand extends Command {
    /** Creates a new IndexerAutoCommand. */
    private final IndexerSubsystem _Indexer;

    public IndexerAutoCommand(IndexerSubsystem Indexer) {
        // Use addRequirements() here to declare subsystem dependencies.
        _Indexer = Indexer;
        addRequirements(_Indexer);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        _Indexer.forward(Constants.Indexer.IndexerSpeed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    public void setIndexerPID() {

    }

    public void setPower(double power) {
        _Indexer.autoPower(power);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
