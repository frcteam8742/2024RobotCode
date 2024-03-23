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
import frc.robot.Constants.Intake;
import frc.robot.commands.*;

public class AutoDriveForwardDualNote extends SequentialCommandGroup {
    /** Creates a new AutoDriveForwardDualNote. */
    private DriveTrainSubsystem _DriveTrainSubsystem;
    private ShooterSubsystem _ShooterSubsystem;
    private IndexerSubsystem _IndexerSubsystem;
    private IntakeSubsystem _IntakeSubsystem;
    private GyroSubsystem _GyroSubsystem;

    public AutoDriveForwardDualNote(DriveTrainSubsystem drive_Subsystem, ShooterSubsystem shooter_Subsystem,
            IndexerSubsystem indexer_Subsystem, IntakeSubsystem intake_Subsystem, GyroSubsystem gyro_Subsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        _DriveTrainSubsystem = drive_Subsystem;
        _ShooterSubsystem = shooter_Subsystem;
        _IndexerSubsystem = indexer_Subsystem;
        _IntakeSubsystem = intake_Subsystem;
        _GyroSubsystem = gyro_Subsystem;

        // Update notes from Win 10 laptop

        // Step : Drives backwards to optimal shooting position from straight on
        DriveTrainAutoCommand backwards1 = new DriveTrainAutoCommand(drive_Subsystem, -.5, .3);
        // Step : Does nothing, step can possibly be removed
        DriveTrainAutoCommand doNothing1 = new DriveTrainAutoCommand(drive_Subsystem, 0, .3);
        // Step : Shoots note
        ShooterAutoCommand shootHigh1 = new ShooterAutoCommand(shooter_Subsystem, 1);
        IndexerAutoCommand indexHigh1 = new IndexerAutoCommand(indexer_Subsystem, 1);
        // Step : Drives backward toward 2nd note and picks it up, placing it into the shooter
        DriveTrainAutoCommand backwards2 = new DriveTrainAutoCommand(drive_Subsystem, -.5, .9);
        IntakeAutoCommand intake1 = new IntakeAutoCommand(intake_Subsystem, 2);
        IndexerAutoCommand indexer1 = new IndexerAutoCommand(indexer_Subsystem, 2, -.3); // LOOK
        //Step : Stop to grab it
        IntakeAutoCommand intake2 = new IntakeAutoCommand(intake_Subsystem, 1);
        IndexerAutoCommand indexer2 = new IndexerAutoCommand(indexer_Subsystem, 1, -.1); // LOOK
        // Step : Drives forward toward the speaker covering most of the distance quickly
        DriveTrainAutoCommand forwards1 = new DriveTrainAutoCommand(drive_Subsystem, .5, 1);
        // Step : Drives forward at a slower pace
        DriveTrainAutoCommand forwards2 = new DriveTrainAutoCommand(drive_Subsystem, .2, .75); // LOOK
        // Step : Drives backwards to our best shooting distance
        DriveTrainAutoCommand backwards3 = new DriveTrainAutoCommand(drive_Subsystem, -.5, .3);
        // Step : Shoots note
        ShooterAutoCommand shootHigh2 = new ShooterAutoCommand(shooter_Subsystem, .75);
        IndexerAutoCommand indexHigh2 = new IndexerAutoCommand(indexer_Subsystem, .75);
        // Step : Drives out of zone
        DriveTrainAutoCommand backwards4 = new DriveTrainAutoCommand(drive_Subsystem, -.5, 1.1);

        addCommands(
                backwards1,
                doNothing1,
                new ParallelCommandGroup(
                        shootHigh1,
                        indexHigh1),
                new ParallelCommandGroup(
                        intake1,
                        indexer1,
                        backwards2),
                new ParallelCommandGroup(
                        intake2,
                        indexer2
                ),
                forwards1,
                forwards2,
                backwards3,
                new ParallelCommandGroup(
                        shootHigh2,
                        indexHigh2),
                backwards4
        );

    }
}
