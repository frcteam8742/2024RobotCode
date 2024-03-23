// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class AmpTeleopCommand extends Command {
    /** Creates a new AmpCommand. */
    private final XboxController _Xbox;
    private final AmpSubsystem _Amp;
    public AmpTeleopCommand(AmpSubsystem amp, XboxController xbox) {
        // Use addRequirements() here to declare subsystem dependencies.
        _Xbox = xbox;
        _Amp = amp;
        addRequirements(_Amp);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        _Amp.AmpRetract();
    }

    // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (_Xbox.getBButton()){
      _Amp.AmpExtend();
     } else if (_Xbox.getAButton()) {
      _Amp.AmpRetract();
     } else {_Amp.AmpIdle();
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
