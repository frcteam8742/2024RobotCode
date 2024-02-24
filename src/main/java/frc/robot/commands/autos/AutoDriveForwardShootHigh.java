// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.commands.*;

public class AutoDriveForwardShootHigh extends SequentialCommandGroup{
  /** Creates a new AutoDriveForwardShootHigh. */

  private DriveTrainSubsystem _DriveTrainSubsystem;
  private ShooterSubsystem _ShooterSubsystem;

  public AutoDriveForwardShootHigh(DriveTrainSubsystem drive_Subsystem, ShooterSubsystem shooter_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    _DriveTrainSubsystem = drive_Subsystem;
    _ShooterSubsystem = shooter_Subsystem;
    DriveTrainAutoCommand backwards = new DriveTrainAutoCommand(drive_Subsystem);
    ShooterAutoCommand shootHigh = new ShooterAutoCommand(shooter_Subsystem);
    backwards.setPower(-.25, -.25);
    backwards.setPower(0, 0);
    shootHigh.setPower(Constants.Shooter.HighSpeed);

    addCommands(
      
    )
  }

}
