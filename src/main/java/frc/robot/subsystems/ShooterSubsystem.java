// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import com.revrobotics.CANSparkMAX;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class ShooterSubsystem extends SubsystemBase {
    private PWMSparkMax _TopShooterMotor = new PWMSparkMax(Constants.CANIDS.TopShooterMotor);
    private PWMSparkMax _BottomShooterMotor = new PWMSparkMax(Constants.CANIDS.BottomShooterMotor);
    private double _Power = 0;

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        _TopShooterMotor.addFollower(_BottomShooterMotor);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        _TopShooterMotor.set(_Power);
        _BottomShooterMotor.set(_Power);
    }

    public void low() {
        _Power = Constants.Shooter.LowPower;
    }

    public void high() {
        _Power = Constants.Shooter.HighPower;
    }

    public void off() {
        _Power = 0;
    }
}