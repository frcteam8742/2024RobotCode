// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;

public class IntakeTeleopCommand extends Command {
    /** Creates a new IntakeCommand. */
    private final IntakeSubsystem _Intake;
    private XboxController _Xbox;

    public IntakeTeleopCommand(IntakeSubsystem intake, XboxController xbox) {
        // Use addRequirements() here to declare subsystem dependencies.
        _Intake = intake;
        _Xbox = xbox;
        addRequirements(_Intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        _Intake.off();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double Right = _Xbox.getRightTriggerAxis();
        double Left = _Xbox.getLeftTriggerAxis();
        if (Right < Constants.Intake.TriggerDeadZone) {
            Right = 0;
        }
        if (Left < Constants.Intake.TriggerDeadZone) {
            Left = 0;
        }
        if (Right >= Left) {
            _Intake.on(Right);
        } else {
            _Intake.reverse(Left);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _Intake.off();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
