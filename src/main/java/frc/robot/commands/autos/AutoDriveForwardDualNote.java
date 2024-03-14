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

        // Look in example for context why this is the way it is,
        // the commands in order of when they happen

        // Step 0
        ShooterAutoCommand shootHigh1 = new ShooterAutoCommand(shooter_Subsystem, 1);
        DriveTrainAutoCommand backwards1 = new DriveTrainAutoCommand(drive_Subsystem, -.5, .75);
        // Step 1
        ShooterAutoCommand shootHigh2 = new ShooterAutoCommand(shooter_Subsystem, 1);
        IndexerAutoCommand indexHigh1 = new IndexerAutoCommand(indexer_Subsystem, 1);
        // Step 2
        DriveTrainAutoCommand backwards2 = new DriveTrainAutoCommand(drive_Subsystem, -.5, 1);
        IntakeAutoCommand intake1 = new IntakeAutoCommand(intake_Subsystem, 1);
        IndexerAutoCommand indexer6 = new IndexerAutoCommand(indexer_Subsystem, 1);
        // Step 2.5
        IntakeAutoCommand intake2 = new IntakeAutoCommand(intake_Subsystem, 1);
        IndexerAutoCommand indexer9 = new IndexerAutoCommand(indexer_Subsystem, 1, -.3);
        // Step 3
        IntakeAutoCommand intake3 = new IntakeAutoCommand(intake_Subsystem, 1);
        IndexerAutoCommand indexer8 = new IndexerAutoCommand(indexer_Subsystem, .5, -.3);
        DriveTrainAutoCommand forward2 = new DriveTrainAutoCommand(drive_Subsystem, .5, .5);
        ShooterAutoCommand shootHigh4 = new ShooterAutoCommand(shooter_Subsystem, 1);
        // Step 5
        DriveTrainAutoCommand forward3 = new DriveTrainAutoCommand(drive_Subsystem, .5, .7); // .3
        ShooterAutoCommand shootHigh5 = new ShooterAutoCommand(shooter_Subsystem, .3);
        // Step 6
        ShooterAutoCommand shootHigh3 = new ShooterAutoCommand(shooter_Subsystem, .35);
        IndexerAutoCommand indexer2 = new IndexerAutoCommand(indexer_Subsystem, 0, -.5);
        // Step Whatever
        // DriveTrainAutoTurnCommand turn1 = new DriveTrainAutoTurnCommand(drive_Subsystem, gyro_Subsystem, 0);
        // Step 7
        DriveTrainAutoCommand backwards4 = new DriveTrainAutoCommand(drive_Subsystem, -.5, .1); // 2
        //Step yadada
        IndexerAutoCommand indexHigh2 = new IndexerAutoCommand(indexer_Subsystem, 1);
        ShooterAutoCommand shootHigh6 = new ShooterAutoCommand(shooter_Subsystem, 1);
        // Step 8
        // DriveTrainAutoTurnCommand turn2 = new DriveTrainAutoTurnCommand(drive_Subsystem, gyro_Subsystem, 0);

        //Last one
        DriveTrainAutoCommand backwards3 = new DriveTrainAutoCommand(drive_Subsystem, -.5, 1); // 2

        addCommands(
                new ParallelCommandGroup(
                        backwards1,
                        shootHigh1),
                new ParallelCommandGroup(
                        indexHigh1,
                        shootHigh2),
                new ParallelCommandGroup(
                        backwards2,
                        intake1,
                        indexer6
                        ),
                new ParallelCommandGroup(
                        indexer9,
                        intake2),
                new ParallelCommandGroup(
                        forward2,
                        // intake3
                        indexer8),
                        forward3,
                // new ParallelCommandGroup(
                //         indexer2
                //         ),
                // turn1,
                backwards4,
                new ParallelCommandGroup(
                        indexHigh2,
                        shootHigh6),
                        // turn2,
                backwards3);
    }
}
