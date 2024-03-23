// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NetworkTables;
import edu.wpi.first.networktables.*;

public class AutoChooserCommand extends Command {
    /** Creates a new AutoChooserCommand. */
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // Standard robot network tables
    NetworkTable table = inst.getTable("datatable");
    private NetworkTableEntry _AutoChoice = table.getEntry("AutoChoice");

    String AutoChoice = "";

    public AutoChooserCommand() {
        // Use addRequirements() here to declare subsystem dependencies.

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        AutoChoice = _AutoChoice.getString("");
        // System.out.println(AutoChoice);
    }

    public String getValue() {
        return AutoChoice;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
