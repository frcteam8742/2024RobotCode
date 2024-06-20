// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveTrainAutoCommand;
import frc.robot.commands.DriveTrainAutoTurnCommand;
import frc.robot.commands.IndexerAutoCommand;
import frc.robot.commands.IntakeAutoCommand;
import frc.robot.commands.ShooterAutoCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Devices.GyroSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoThreeNoteLeft extends SequentialCommandGroup {
    /** Creates a new AutoThreeNoteCenter. */
    private DriveTrainSubsystem _DriveTrainSubsystem;
    private ShooterSubsystem _ShooterSubsystem;
    private IndexerSubsystem _IndexerSubsystem;
    private IntakeSubsystem _IntakeSubsystem;
    private GyroSubsystem _GyroSubsystem;

    public AutoThreeNoteLeft(DriveTrainSubsystem drive_Subsystem, ShooterSubsystem shooter_Subsystem,
            IndexerSubsystem indexer_Subsystem, IntakeSubsystem intake_Subsystem, GyroSubsystem gyro_Subsystem) {
        // Add your commands in the addCommands() call, e.g.

        // Step : Drives backwards to optimal shooting position from straight on
        DriveTrainAutoCommand backwards1 = new DriveTrainAutoCommand(drive_Subsystem, -.5, .3);
        // Step : Does nothing, step can possibly be removed
        DriveTrainAutoCommand doNothing1 = new DriveTrainAutoCommand(drive_Subsystem, 0, .5);
        // Step : Shoots note
        ShooterAutoCommand shootHigh1 = new ShooterAutoCommand(shooter_Subsystem, .75); //CHANGED
        IndexerAutoCommand indexHigh1 = new IndexerAutoCommand(indexer_Subsystem, .75);
        // Step : Drives backward toward 2nd note and picks it up, placing it into the
        // shooter
        DriveTrainAutoCommand backwards2 = new DriveTrainAutoCommand(drive_Subsystem, -.5, 1);
        IntakeAutoCommand intake1 = new IntakeAutoCommand(intake_Subsystem, 1.55);
        IndexerAutoCommand indexer1 = new IndexerAutoCommand(indexer_Subsystem, 1.55, -.3); // LOOK
        // Step : Stop to grab it
        IntakeAutoCommand intake2 = new IntakeAutoCommand(intake_Subsystem, 1.25, -.35);
        IndexerAutoCommand indexer2 = new IndexerAutoCommand(indexer_Subsystem, 1.25, -.3); // LOOK
        // Step : Drives forward toward the speaker covering most of the distance quickly
        DriveTrainAutoCommand forwards1 = new DriveTrainAutoCommand(drive_Subsystem, .5, 1);
        // Step : Drives forward at a slower pace
        DriveTrainAutoCommand forwards2 = new DriveTrainAutoCommand(drive_Subsystem, .2, 1); // LOOK
        // Step : Drives backwards to our best shooting distance
        DriveTrainAutoCommand backwards3 = new DriveTrainAutoCommand(drive_Subsystem, -.5, .3);
        // Step : Shoots note
        ShooterAutoCommand shootHigh2 = new ShooterAutoCommand(shooter_Subsystem, .75);
        IndexerAutoCommand indexHigh2 = new IndexerAutoCommand(indexer_Subsystem, .75);
        // Step : Turns to angle
        DriveTrainAutoTurnCommand turnToNote2 = new DriveTrainAutoTurnCommand(drive_Subsystem, gyro_Subsystem, 40); // 320 //35

        // Step : Backs up and Noms it
        DriveTrainAutoCommand backwards4 = new DriveTrainAutoCommand(drive_Subsystem, -.5, 1.4);
        IntakeAutoCommand intake3 = new IntakeAutoCommand(intake_Subsystem, 1.55);
        IndexerAutoCommand indexer3 = new IndexerAutoCommand(indexer_Subsystem, 1.55, -.3);
        // Step : Stop and intake the ball
        IntakeAutoCommand intake4 = new IntakeAutoCommand(intake_Subsystem, 1, -.35);
        IndexerAutoCommand indexer4 = new IndexerAutoCommand(indexer_Subsystem, 1, -.3);
        // Step : Drives to shooting spot
        DriveTrainAutoCommand forwards3 = new DriveTrainAutoCommand(drive_Subsystem, .5, 1.4);
        //Step : Pauses
        DriveTrainAutoCommand doNothing2 = new DriveTrainAutoCommand(drive_Subsystem, 0, .3);
        // Step : Turns to shoot
        DriveTrainAutoTurnCommand turnToShoot2 = new DriveTrainAutoTurnCommand(drive_Subsystem, gyro_Subsystem, 325);
        // Bcks up a widdle bit
        // DriveTrainAutoCommand backwards5 = new DriveTrainAutoCommand(drive_Subsystem, -.4, .1); // add command before to
        //Step : Pauses
        DriveTrainAutoCommand doNothing3 = new DriveTrainAutoCommand(drive_Subsystem, 0, .5);
        // Step : Shoots
        ShooterAutoCommand shootHigh3 = new ShooterAutoCommand(shooter_Subsystem, 1);
        IndexerAutoCommand indexHigh3 = new IndexerAutoCommand(indexer_Subsystem, 1);
        // Step : backs up
        DriveTrainAutoCommand backwards6 = new DriveTrainAutoCommand(drive_Subsystem, -.5, .4); // add command before to
                                                                                               // back in

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
                        indexer2),
                forwards1,
                forwards2,
                backwards3,
                new ParallelCommandGroup(
                        shootHigh2,
                        indexHigh2),
                turnToNote2,
                new ParallelCommandGroup(
                backwards4,
                intake3,
                indexer3),
                new ParallelCommandGroup(
                    intake4,
                    indexer4),
                forwards3,
                doNothing2,
                turnToShoot2,
                // backwards5,
                doNothing3,
                new ParallelCommandGroup(
                    shootHigh3,
                    indexHigh3),
                backwards6
                );
    }
}
