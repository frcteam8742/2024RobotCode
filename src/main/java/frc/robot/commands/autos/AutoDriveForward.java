// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveTrainAutoCommand;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoDriveForward extends SequentialCommandGroup {
    /** Creates a new AutoLowNote. */
    DriveTrainSubsystem Drive;
    public AutoDriveForward(DriveTrainSubsystem drive) {
        // Use addRequirements() here to declare subsystem dependencies.
        Drive = drive;
        DriveTrainAutoCommand driveForward = new DriveTrainAutoCommand(drive);
        DriveTrainAutoCommand stop = new DriveTrainAutoCommand(drive);

        driveForward.setDrivePID(8, 8);
        stop.setPower(0, 0);

        addCommands(
            driveForward.withTimeout(10),
            stop
        );


    }
}
