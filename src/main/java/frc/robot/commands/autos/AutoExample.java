// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Devices.GyroSubsystem;
import frc.robot.Constants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoExample extends SequentialCommandGroup {
    /** Creates a new AutoDriveForward. */

    private DriveTrainSubsystem _DriveSubsystem;
    private GyroSubsystem _Gyro;


    public AutoExample(DriveTrainSubsystem drive_subsystem, GyroSubsystem gyro) {
        _DriveSubsystem = drive_subsystem;
        _Gyro = gyro;
        DriveTrainAutoCommand drive1 = new DriveTrainAutoCommand(drive_subsystem);
        DriveTrainAutoCommand drive2 = new DriveTrainAutoCommand(drive_subsystem);
        DriveTrainAutoTurnCommand turn1 = new DriveTrainAutoTurnCommand(drive_subsystem, gyro);
        drive1.setPower(-.25, -.25);
        drive2.setPower(0, 0);
        turn1.turnPower(54, Constants.DriveTrain.Tolerance);

        // DO NOT RUN, just an example for programmers

        addCommands(
                // drives backward for .2 seconds then stops for .5
                drive1.withTimeout(0.2),
                drive2.withTimeout(0.5),
                // waits (doesn' t do anything) for 1 second
                new WaitCommand(1),
                // drives for another second
                drive1.withTimeout(1),
                turn1.withTimeout(4),
                // does 2 commands at the same time, in this case do not run because it is doing
                // it on the same motors
                new ParallelCommandGroup(
                        drive1,
                        drive2).withTimeout(4)

        );
    }
    // Use addRequirements() here to declare subsystem dependencies.
}
