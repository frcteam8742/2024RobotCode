// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.*;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax _TopIntakeMotor = new CANSparkMax(Constants.CANIDS.TopIntakeMotor, MotorType.kBrushless);
    private CANSparkMax _BottomIntakeMotor = new CANSparkMax(Constants.CANIDS.BottomIntakeMotor, MotorType.kBrushless);
    private CANSparkMax _IndexerMotor = new CANSparkMax(Constants.CANIDS.IndexerMotor, MotorType.kBrushless);
    private DigitalInput _BeamBreakLimitSwitch = new DigitalInput(Constants.Sensors.BeamBreakLimitSwitch);
    private double _Power = 0;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    if(_BeamBreakLimitSwitch.get()){
        _BottomIntakeMotor.set(0);
        _TopIntakeMotor.set(0);
        _IndexerMotor.set(0);
    } else{
        _BottomIntakeMotor.set(-_Power);
        _TopIntakeMotor.set(-Constants.Intake.IntakeOffsetMultiplicative * _Power);
        _IndexerMotor.set(-Constants.Intake.IntakeOffsetMultiplicative * _Power);
    }
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
