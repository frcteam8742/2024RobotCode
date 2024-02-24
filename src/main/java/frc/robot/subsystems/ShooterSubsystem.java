// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

    public CANSparkMax _TopShooterMotor = new CANSparkMax(Constants.CANIDS.TopShooterMotor, MotorType.kBrushless);
    public CANSparkMax _BottomShooterMotor = new CANSparkMax(Constants.CANIDS.BottomShooterMotor, MotorType.kBrushless);

    public RelativeEncoder _TopEncoder;
    public RelativeEncoder _BottomEncoder;

    public SparkPIDController _TopShooterPID;
    // public SparkPIDController _BottomShooterPID;

    public double TP, TI, TD, TIZ, TFF, TMAX;
    // public double BP, BI, BD, BIZ, BFF, BMAX;

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        _TopShooterMotor.follow(_BottomShooterMotor);

        _TopShooterPID = _TopShooterMotor.getPIDController();
        // _BottomShooterPID = _BottomShooterMotor.getPIDController();

        _TopEncoder = _TopShooterMotor.getEncoder();
        _BottomEncoder = _BottomShooterMotor.getEncoder();

        // Initializing PID
        TP = 0.5;
        TI = 0;
        TD = 0;
        TIZ = 0;
        TFF = 0;

        // BP = 0.05;
        // BI = 0;
        // BD = 0;
        // BIZ = 0;
        // BFF = 0;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // _BottomShooterMotor.set(_Power);

        // Setting PID for Top
        _TopShooterPID.setP(TP);
        _TopShooterPID.setI(TI);
        _TopShooterPID.setD(TD);
        _TopShooterPID.setIZone(TIZ);
        _TopShooterPID.setFF(TFF);
        _TopShooterPID.setOutputRange(-1, 1);

        // Setting PID for Bottom
        // _BottomShooterPID.setP(BP);
        // _BottomShooterPID.setI(BI);
        // _BottomShooterPID.setD(BD);
        // _BottomShooterPID.setIZone(BIZ);
        // _BottomShooterPID.setFF(BFF);
        // _BottomShooterPID.setOutputRange(-1, 1);

    }

    public void autoPower(double power) {
        _BottomShooterMotor.set(power);
    }

    public void low() {
        // System.out.println("Low Set");
        _BottomShooterMotor.set(Constants.Shooter.LowSpeed);
        // _TopShooterPID.setReference(Constants.Shooter.LowSpeed,
        // CANSparkMax.ControlType.kVelocity);
    }

    public void high() {
        // System.out.println("High Set");
        _BottomShooterMotor.set(Constants.Shooter.HighSpeed);
        // _TopShooterPID.setReference(Constants.Shooter.HighSpeed,
        // CANSparkMax.ControlType.kVelocity);
    }

    public void off() {
        // System.out.print("Off set\n");
        _BottomShooterMotor.set(0);
    }
}