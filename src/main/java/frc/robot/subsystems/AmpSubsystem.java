// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class AmpSubsystem extends SubsystemBase {
    private final DoubleSolenoid _ampSolenoidA = new DoubleSolenoid(6, PneumaticsModuleType.CTREPCM, 3, 2);
    private DoubleSolenoid.Value _val = DoubleSolenoid.Value.kOff;

    /** Creates a new Hanger. */
    public AmpSubsystem() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        _ampSolenoidA.set(_val);
    }

    public void AmpExtend() {
        _val = DoubleSolenoid.Value.kReverse;
    }

    public void AmpIdle() {
        _val = DoubleSolenoid.Value.kOff;
    }

    public void AmpRetract() {
        _val = DoubleSolenoid.Value.kForward;
    }
}
