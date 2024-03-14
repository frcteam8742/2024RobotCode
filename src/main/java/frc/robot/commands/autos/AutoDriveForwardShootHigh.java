// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Devices.GyroSubsystem;
import frc.robot.Constants;
import frc.robot.commands.*;

public class AutoDriveForwardShootHigh extends SequentialCommandGroup {
    /** Creates a new AutoDriveForwardShootHigh. */

  private DriveTrainSubsystem _DriveTrainSubsystem;
    private ShooterSubsystem _ShooterSubsystem;
    private IndexerSubsystem _IndexerSubsystem;
    private IntakeSubsystem _IntakeSubsystem;
    private GyroSubsystem _GyroSubsystem;

    public AutoDriveForwardShootHigh(DriveTrainSubsystem drive_Subsystem, ShooterSubsystem shooter_Subsystem,
            IndexerSubsystem indexer_Subsystem, IntakeSubsystem intake_Subsystem, GyroSubsystem gyro_Subsystem) {

                //WILL FIX TOMORROW0.

    //     // Use addRequirements() here to declare subsystem dependencies.
        _DriveTrainSubsystem = drive_Subsystem;
        _ShooterSubsystem = shooter_Subsystem;
        _IndexerSubsystem = indexer_Subsystem;
        _IntakeSubsystem = intake_Subsystem;
        _GyroSubsystem = gyro_Subsystem;

        // Step 0
        DriveTrainAutoCommand backwards1 = new DriveTrainAutoCommand(drive_Subsystem, -.5, .6);
        // Step 1
        ShooterAutoCommand shootHigh2 = new ShooterAutoCommand(shooter_Subsystem, 1);
        IndexerAutoCommand indexHigh1 = new IndexerAutoCommand(indexer_Subsystem, 1);
        // Step 2
        DriveTrainAutoCommand backwards2 = new DriveTrainAutoCommand(drive_Subsystem, -.5, .4);

    //     DriveTrainAutoCommand stop = new DriveTrainAutoCommand(drive_Subsystem);
    //     ShooterAutoCommand shootHigh = new ShooterAutoCommand(shooter_Subsystem);
    //     IndexerAutoCommand index = new IndexerAutoCommand(indexer_Subsystem);
    //     backwards.setPower(-.5, -.5);
    //     stop.setPower(0, 0);
    //     shootHigh.setPower(Constants.Shooter.HighSpeed);
    //     index.setPower(Constants.Indexer.IndexerSpeed);

        addCommands(
            backwards1,
        new ParallelCommandGroup(
            shootHigh2,
            indexHigh1
        ),
        backwards2
    );
    }
}
