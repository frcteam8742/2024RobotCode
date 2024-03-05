// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.Constants;

//Change to SequentialCommandGroup running both shooter and indexer with a timeout between
public class IndexerTeleopCommand extends Command {
    private final IndexerSubsystem _Index;
    private XboxController _Xbox;
    private boolean _IndexerButtonPressed;
    private boolean _ReverseButtonPressed;
    private boolean _LowDisable;

    /** Creates a new IndexerTeleopSubsystem. */
    public IndexerTeleopCommand(IndexerSubsystem index, XboxController xbox) {
        // Use addRequirements() here to declare subsystem dependencies.
        _Index = index;
        _Xbox = xbox;
        addRequirements(_Index);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        _Index.off();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        _IndexerButtonPressed = _Xbox.getRightBumper();
        _ReverseButtonPressed = _Xbox.getLeftBumper();

        if (_Xbox.getRightTriggerAxis() > Constants.Intake.TriggerDeadZone) {
            if(!_LowDisable){
                _Index.forward(Constants.Indexer.IndexerLowSpeed);
            } else {
                _Index.off();
            }
        } else if (_Xbox.getLeftTriggerAxis() > Constants.Intake.TriggerDeadZone) {
            if(!_LowDisable){
                _Index.forward(Constants.Indexer.IndexerLowSpeed);
            } else {
                _Index.off();
            }
        } else {
            if (_IndexerButtonPressed == true) {
                _Index.forward(Constants.Indexer.IndexerSpeed);
            } else if (_ReverseButtonPressed == true) {
                _Index.reverse(Constants.Indexer.IndexerSpeed);
            } else {
                _Index.off();
            }
        }

    }

    public void disableLow(){
        _LowDisable = true;
    }
    public void enableLow(){
        _LowDisable = false;
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
