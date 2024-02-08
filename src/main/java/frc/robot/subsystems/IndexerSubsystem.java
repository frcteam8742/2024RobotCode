package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.*;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IndexerSubsystem extends SubsystemBase {    
    
    //PID
    private double _P;
    private double _I;
    private double _D;

    private double _Power = 0;
    
    private CANSparkMax _IndexerMotor = new CANSparkMax(Constants.CANIDS.IndexerMotor, MotorType.kBrushless);
    private SparkPIDController _IndexerPID;




    /** Creates a new IntakeSubsystem. */
    public IndexerSubsystem() {
        _IndexerPID = _IndexerMotor.getPIDController();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        _IndexerMotor.set(_Power);
    }

    public void off() {
        _Power = 0;
    }

    public void fire(double Power) {
        _Power = Power;
    }

    public void reverse(double Power) {
        _Power = -Power;
    }

}