// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoExample extends SequentialCommandGroup {
    /** Creates a new AutoDriveForward. */

    private DriveTrainSubsystem _DriveSubsystem;

    public AutoExample(DriveTrainSubsystem drive_subsystem) {
        _DriveSubsystem = drive_subsystem;
        DriveTrainAutoCommand drive1 = new DriveTrainAutoCommand(drive_subsystem);
        DriveTrainAutoCommand drive2 = new DriveTrainAutoCommand(drive_subsystem);
        drive1.setPower(-.25, -.25);
        drive2.setPower(0, 0);

        addCommands(
                drive1.withTimeout(0.2),
                drive2.withTimeout(0.5),
                drive1.withTimeout(1),
                new ParallelCommandGroup(
                        drive1,
                        drive2).withTimeout(4)

        );
    }
    // Use addRequirements() here to declare subsystem dependencies.
}
