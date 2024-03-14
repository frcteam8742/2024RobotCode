// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.*;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax _TopIntakeMotor = new CANSparkMax(Constants.CANIDS.TopIntakeMotor, MotorType.kBrushed);
    private CANSparkMax _BottomIntakeMotor = new CANSparkMax(Constants.CANIDS.BottomIntakeMotor, MotorType.kBrushless);
    private double _Power = 0;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        _BottomIntakeMotor.set(Constants.Intake.IntakeOffsetMultiplicativ2*_Power);
        _TopIntakeMotor.set(-Constants.Intake.IntakeOffsetMultiplicative* _Power);
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
