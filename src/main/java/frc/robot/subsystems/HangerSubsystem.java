// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class HangerSubsystem extends SubsystemBase {
    private final DoubleSolenoid _hangerSolenoidA = new DoubleSolenoid(6, PneumaticsModuleType.CTREPCM, 5, 4);
    private final DoubleSolenoid _hangerSolenoidB = new DoubleSolenoid(6, PneumaticsModuleType.CTREPCM, 7, 6);
    private DoubleSolenoid.Value _val = DoubleSolenoid.Value.kOff;

    /** Creates a new Hanger. */
    public HangerSubsystem() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        _hangerSolenoidA.set(_val);
        _hangerSolenoidB.set(_val);
    }

    public void IndexExtend() {
        _val = DoubleSolenoid.Value.kReverse;
    }

    public void IndexIdle() {
        _val = DoubleSolenoid.Value.kOff;
    }

    public void IndexRetract() {
        _val = DoubleSolenoid.Value.kForward;
    }
}
