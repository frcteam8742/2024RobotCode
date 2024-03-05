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

        DriveTrainAutoCommand forward = new DriveTrainAutoCommand(drive_Subsystem);
        DriveTrainAutoCommand backwards = new DriveTrainAutoCommand(drive_Subsystem);
        DriveTrainAutoCommand stop = new DriveTrainAutoCommand(drive_Subsystem);
        ShooterAutoCommand shootHigh = new ShooterAutoCommand(shooter_Subsystem);
        IndexerAutoCommand indexHigh = new IndexerAutoCommand(indexer_Subsystem);
        IndexerAutoCommand indexLow = new IndexerAutoCommand(indexer_Subsystem);
        IntakeAutoCommand intake = new IntakeAutoCommand(intake_Subsystem);

        forward.setPower(.5, .5);
        backwards.setPower(-.5, -.5);
        stop.setPower(0, 0);
        shootHigh.setPower(Constants.Shooter.HighSpeed);
        indexHigh.setPower(Constants.Indexer.IndexerSpeed);
        indexLow.setPower(-.2);
        intake.setPower(1);

        addCommands(
                shootHigh.withTimeout(2)
                // new ParallelCommandGroup(
                //         shootHigh,
                //         indexHigh).withTimeout(1),
                // new ParallelCommandGroup(
                //         backwards,
                //         intake).withTimeout(2),
                // new ParallelCommandGroup(
                //         stop,
                //         intake).withTimeout(.5),
                // new ParallelCommandGroup(
                //         forward,
                //         shootHigh,
                //         intake).withTimeout(1.5),
                // new ParallelCommandGroup(
                //         forward,
                //         shootHigh,
                //         indexLow).withTimeout(.2),
                // new ParallelCommandGroup(
                //         forward,
                //         shootHigh).withTimeout(.3),
                // new ParallelCommandGroup(
                //         indexHigh,
                //         shootHigh).withTimeout(1),
                // backwards.withTimeout(1.5)
                );
    }
}
