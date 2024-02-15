// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterTeleopCommand extends Command {
    /** Creates a new ShooterTeleopCommand. */
    private final ShooterSubsystem _Shooter;
    private XboxController _Xbox;

    public ShooterTeleopCommand(ShooterSubsystem shooter, XboxController xbox) {
        
        // Use addRequirements() here to declare subsystem dependencies.
        _Shooter = shooter;
        _Xbox = xbox;
        addRequirements(_Shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        _Shooter.off();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (_Xbox.getYButton()) {
            _Shooter.high();
        } else if (_Xbox.getXButton()) { //else if (_Xbox.getBButton()) 
            _Shooter.low();
        } else {
            _Shooter.off();
        }



        //test code
        // if (_Xbox.getXButtonPressed()){
        //     _Shooter.high();
        //     System.out.print("_Xbox.getXButton was pressed\n");
        //  } else if (_Xbox.getXButtonReleased()){
        //     System.out.print("Xbox button is released\n");
        //     _Shooter.off();
        // }
        



        //  if need be, make above statement if (B button pressed) and make another mode
        // for on if motors are getting too hot on low
        // } else {
        // _Shooter.on();
        // }
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
