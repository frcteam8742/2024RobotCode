// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangerSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class HangerTeleopCommand extends Command {
    /** Creates a new HangerCommand. */
    private final XboxController _Xbox;
    private final HangerSubsystem _Hanger;
    public HangerTeleopCommand(HangerSubsystem hanger, XboxController xbox) {
        // Use addRequirements() here to declare subsystem dependencies.
        _Xbox = xbox;
        _Hanger = hanger;
        addRequirements(_Hanger);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        _Hanger.IndexRetract();
    }

    // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (_Xbox.getPOV() == 0){
      _Hanger.IndexExtend();
     } else if (_Xbox.getPOV() == 180) {
      _Hanger.IndexRetract();
     } else {_Hanger.IndexIdle();
     }
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
