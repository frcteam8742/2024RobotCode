// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.commands.*;

public class AutoDriveForwardShootHigh extends SequentialCommandGroup {
    /** Creates a new AutoDriveForwardShootHigh. */

    private DriveTrainSubsystem _DriveTrainSubsystem;
    private ShooterSubsystem _ShooterSubsystem;
    private IndexerSubsystem _IndexerSubsystem;

    public AutoDriveForwardShootHigh(DriveTrainSubsystem drive_Subsystem, ShooterSubsystem shooter_Subsystem,
            IndexerSubsystem indexer_Subsystem) {

        // Use addRequirements() here to declare subsystem dependencies.
        _DriveTrainSubsystem = drive_Subsystem;
        _ShooterSubsystem = shooter_Subsystem;
        _IndexerSubsystem = indexer_Subsystem;
        DriveTrainAutoCommand forward = new DriveTrainAutoCommand(drive_Subsystem);
        DriveTrainAutoCommand backwards = new DriveTrainAutoCommand(drive_Subsystem);
        DriveTrainAutoCommand stop = new DriveTrainAutoCommand(drive_Subsystem);
        ShooterAutoCommand shootHigh = new ShooterAutoCommand(shooter_Subsystem);
        IndexerAutoCommand index = new IndexerAutoCommand(indexer_Subsystem);
        forward.setPower(.5, .5);
        backwards.setPower(-.5, -.5);
        stop.setPower(0, 0);
        shootHigh.setPower(Constants.Shooter.HighSpeed);
        index.setPower(Constants.Indexer.IndexerSpeed);

        addCommands(
                new ParallelCommandGroup(
                        shootHigh.withTimeout(3),
                        forward).withTimeout(2),
                new ParallelCommandGroup(
                        stop,
                        shootHigh).withTimeout(1),
                new ParallelCommandGroup(
                        shootHigh,
                        index).withTimeout(2),
                backwards.withTimeout(3),
                stop);
    }
}
