// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.commands.autos.*;

//HIDS
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    // HIDS
    Joystick _LeftDriveFlightJoystick = new Joystick(Constants.Operator.LeftFlightStickControllerPort);
    Joystick _RightDriveFlightJoystick = new Joystick(Constants.Operator.RightFlightStickControllerPort);
    XboxController _OperatorController = new XboxController(Constants.Operator.OperatorControllerPort);

    // Subsystems
    DriveTrainSubsystem _DriveTrainSubsystem = new DriveTrainSubsystem();
    IndexerSubsystem _IndexerSubsystem = new IndexerSubsystem();
    IntakeSubsystem _IntakeSubsystem = new IntakeSubsystem();
    ShooterSubsystem _ShooterSubsystem = new ShooterSubsystem();
    // Teleop Commands
    DriveTrainTeleopCommand _DriveTrainTeleopCommand = new DriveTrainTeleopCommand(_DriveTrainSubsystem,
            _LeftDriveFlightJoystick, _RightDriveFlightJoystick);
    IndexerTeleopCommand _IndexerTeleopCommand = new IndexerTeleopCommand(_IndexerSubsystem, _OperatorController);
    IntakeTeleopCommand _IntakeTeleopCommand = new IntakeTeleopCommand(_IntakeSubsystem, _OperatorController);
    ShooterTeleopCommand _ShooterTeleopCommand = new ShooterTeleopCommand(_ShooterSubsystem, _OperatorController);

    // Auto Commands
    AutoExample _AutoExample = new AutoExample(_DriveTrainSubsystem);
    AutoDriveForward _AutoDriveForward = new AutoDriveForward(_DriveTrainSubsystem);
    AutoDriveForwardShootHigh _AutoDriveForwardShootHigh = new AutoDriveForwardShootHigh(_DriveTrainSubsystem,
            _ShooterSubsystem, _IndexerSubsystem);

    enum AutoChooser {
        AUTO_EXAMPLE,
        AUTO_DRIVE_FORWARD,
        AUTO_DRIVE_FORWARD_SHOOT_HIGH
    }

    AutoChooser _AutoChooserState = AutoChooser.AUTO_EXAMPLE;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.

    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {

        switch (_AutoChooserState) {
            case AUTO_EXAMPLE:
                _AutoExample.schedule();
                break;
            case AUTO_DRIVE_FORWARD:
                _AutoDriveForward.schedule();
                break;
            case AUTO_DRIVE_FORWARD_SHOOT_HIGH:
                _AutoDriveForwardShootHigh.schedule();
            default:
                _AutoDriveForward.schedule();
                break;
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running.

        _DriveTrainTeleopCommand.schedule();
        _IndexerTeleopCommand.schedule();
        _IntakeTeleopCommand.schedule();
        _ShooterTeleopCommand.schedule();

    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
