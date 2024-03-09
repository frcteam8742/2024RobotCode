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
import frc.robot.Constants.Intake;
import frc.robot.commands.*;

public class AutoDriveForwardDualNote extends SequentialCommandGroup {
    /** Creates a new AutoDriveForwardDualNote. */
    private DriveTrainSubsystem _DriveTrainSubsystem;
    private ShooterSubsystem _ShooterSubsystem;
    private IndexerSubsystem _IndexerSubsystem;
    private IntakeSubsystem _IntakeSubsystem;

    public AutoDriveForwardDualNote(DriveTrainSubsystem drive_Subsystem, ShooterSubsystem shooter_Subsystem,
            IndexerSubsystem indexer_Subsystem, IntakeSubsystem intake_Subsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        _DriveTrainSubsystem = drive_Subsystem;
        _ShooterSubsystem = shooter_Subsystem;
        _IndexerSubsystem = indexer_Subsystem;
        _IntakeSubsystem = intake_Subsystem;

        //Look in example for context why this is the way it is, 
        // the commands in order of when they happen

        ShooterAutoCommand shootHigh1 = new ShooterAutoCommand(shooter_Subsystem, 2);
        //Group 1
        ShooterAutoCommand shootHigh2 = new ShooterAutoCommand(shooter_Subsystem, 1);
        IndexerAutoCommand indexHigh1 = new IndexerAutoCommand(indexer_Subsystem, 1, 1);
        //Group 2
        DriveTrainAutoCommand backwards1 = new DriveTrainAutoCommand(drive_Subsystem, -.5, .1); //2
        IntakeAutoCommand intake1 = new IntakeAutoCommand(intake_Subsystem, 2);
        IntakeAutoCommand intake2 = new IntakeAutoCommand(intake_Subsystem, 2);
        //Group 3
        DriveTrainAutoCommand forward1 = new DriveTrainAutoCommand(drive_Subsystem, .5, .1); //1.5
        ShooterAutoCommand shootHigh3 = new ShooterAutoCommand(shooter_Subsystem, 1.5);
        IntakeAutoCommand intake3 = new IntakeAutoCommand(intake_Subsystem, 1.5);

        DriveTrainAutoCommand forward2 = new DriveTrainAutoCommand(drive_Subsystem, .5, .1); //.2
        ShooterAutoCommand shootHigh4 = new ShooterAutoCommand(shooter_Subsystem, .2);
        IndexerAutoCommand indexLow1 = new IndexerAutoCommand(indexer_Subsystem, .2, .2);

        DriveTrainAutoCommand forward3 = new DriveTrainAutoCommand(drive_Subsystem, .5, .1); //.3
        ShooterAutoCommand shootHigh5 = new ShooterAutoCommand(shooter_Subsystem, .3);

        IndexerAutoCommand indexHigh2 = new IndexerAutoCommand(indexer_Subsystem, .7, .1); //1
        ShooterAutoCommand shootHigh6 = new ShooterAutoCommand(shooter_Subsystem, 1);

        DriveTrainAutoCommand backwards2 = new DriveTrainAutoCommand(drive_Subsystem, -.5, .1); //2



        addCommands(
                shootHigh1,
                new ParallelCommandGroup(
                        shootHigh2,
                        indexHigh1),
                new ParallelCommandGroup(
                        backwards1,
                        intake1),        
                intake2,
                new ParallelCommandGroup(
                        forward1,
                        shootHigh3,
                        intake3),
                new ParallelCommandGroup(
                        forward2,
                        shootHigh4,
                        indexLow1),
                new ParallelCommandGroup(
                        forward3,
                        shootHigh5),
                new ParallelCommandGroup(
                        indexHigh2,
                        shootHigh6).withTimeout(1),
                backwards2
                );
    }
}
