// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;


public class HangerSubsystem extends SubsystemBase {
  private final DoubleSolenoid _hangerSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 1, 0);


  /** Creates a new Hanger. */
  public HangerSubsystem() {
 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void IndexExtend() {
//   hangerSolenoid.set(DoubleSolenoid.Value.kForward);
  } 
  
  public void IndexIdle() {
//   hangerSolenoid.set(DoubleSolenoid.Value.kOff);
  }
 
  public void IndexRetract() {
//   hangerSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
}
