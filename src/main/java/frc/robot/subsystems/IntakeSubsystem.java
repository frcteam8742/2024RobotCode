// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import com.revrobotics.CANSparkMAX;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class IntakeSubsystem extends SubsystemBase {
    private PWMSparkMax _TopIntakeMotor = new PWMSparkMax(Constants.CANIDS.TopIntakeMotor);
    private PWMSparkMax _BottomIntakeMotor = new PWMSparkMax(Constants.CANIDS.BottomIntakeMotor);
    private double _Power = 0;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        _BottomIntakeMotor.set(_Power);
        _TopIntakeMotor.set(-Constants.Intake.IntakeOffsetMultiplicative*_Power);
    }
    public void off() {
        _Power = 0;
    }

    public void on(double Power) {
        _Power = Power;
    }

    public void reverse(double Power) {
        _Power = -Power;
    }
        
}
