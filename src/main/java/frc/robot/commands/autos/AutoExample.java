// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Devices.*;
import edu.wpi.first.wpilibj.Timer;

public class AutoExample extends SequentialCommandGroup {
    /** Creates a new AutoDriveForward. */

    private DriveTrainSubsystem _DriveSubsystem;
    private IndexerSubsystem _IndexerSubsystem;
    private ShooterSubsystem _ShooterSubsystem;
    private IntakeSubsystem _IntakeSubsystem;
    private GyroSubsystem _Gyro;


    public AutoExample(DriveTrainSubsystem drive_subsystem, IndexerSubsystem index_subsystem, ShooterSubsystem shooter_subsystem, IntakeSubsystem intake_subsystem, GyroSubsystem gyro) {
        
        _DriveSubsystem = drive_subsystem;
        _IndexerSubsystem = index_subsystem;
        _ShooterSubsystem = shooter_subsystem;
        _IntakeSubsystem = intake_subsystem;
        _Gyro = gyro;

        //REMINDER: DO NOT CALL ANY OF THESE TWICE OR SYSTEM WILL CRASH
        DriveTrainAutoCommand drive1 = new DriveTrainAutoCommand(drive_subsystem, -.1, .1);
        DriveTrainAutoCommand drive2 = new DriveTrainAutoCommand(drive_subsystem, .1, .1);
        IndexerAutoCommand index = new IndexerAutoCommand(index_subsystem, .1);
        ShooterAutoCommand shootHigh1 = new ShooterAutoCommand(shooter_subsystem, .1);
        ShooterAutoCommand shootHigh2 = new ShooterAutoCommand(shooter_subsystem, .1);
        IntakeAutoCommand intake = new IntakeAutoCommand(intake_subsystem, 3, -.275);
        DriveTrainAutoTurnCommand turn1 = new DriveTrainAutoTurnCommand(drive_subsystem, gyro, 45);
        DriveTrainPIDAutoCommand PIDTest = new DriveTrainPIDAutoCommand(drive_subsystem, 1, 1);

        // DO NOT RUN, just an example for programmers

        
                // drives backward for .2 seconds then stops for .5
        addCommands(
            PIDTest
        );
    }
}
