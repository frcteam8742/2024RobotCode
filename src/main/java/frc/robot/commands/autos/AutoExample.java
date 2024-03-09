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
        DriveTrainAutoCommand drive1 = new DriveTrainAutoCommand(drive_subsystem, .2, 3);
        DriveTrainAutoCommand drive2 = new DriveTrainAutoCommand(drive_subsystem, .5, 3);
        IndexerAutoCommand index = new IndexerAutoCommand(index_subsystem, .5, 2);
        ShooterAutoCommand shootHigh = new ShooterAutoCommand(shooter_subsystem, 1);
        IntakeAutoCommand intake = new IntakeAutoCommand(intake_subsystem, 2);


        // DriveTrainAutoCommand drive2 = new DriveTrainAutoCommand(drive_subsystem);
        // DriveTrainAutoTurnCommand turn1 = new DriveTrainAutoTurnCommand(drive_subsystem, gyro);

        // drive2.setPower(0, 0);
        // turn1.turnPower(54, Constants.DriveTrain.Tolerance);

        // DO NOT RUN, just an example for programmers

        
                // drives backward for .2 seconds then stops for .5
        addCommands(
            new ParallelCommandGroup(
                index,
                drive1), 
            drive2,
            shootHigh,
            intake
        );
    }
}
                // drive2.withTimeout(0.5),
                // waits (doesn' t do anything) for 1 second
                // new WaitCommand(1),
                // drives for another second
                // drive1.withTimeout(1),
                // turn1.withTimeout(4),
                // does 2 commands at the same time, in this case do not run because it is doing
                // it on the same motors
                // new ParallelCommandGroup(
                        // drive2).withTimeout(4)

//         );
//     }
//     // Use addRequirements() here to declare subsystem dependencies.
// }
