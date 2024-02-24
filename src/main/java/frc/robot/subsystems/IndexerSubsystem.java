package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IndexerSubsystem extends SubsystemBase {

    private double _Power = 0;
    private CANSparkMax _IndexerMotor = new CANSparkMax(Constants.CANIDS.IndexerMotor, MotorType.kBrushless);

    /** Creates a new IntakeSubsystem. */
    public IndexerSubsystem() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        _IndexerMotor.set(_Power);
    }

    public void autoPower(double power) {
        _Power = power;
        _IndexerMotor.set(_Power);
    }

    public void PIDSpeed() {

    }

    public void off() {
        _Power = 0;
    }

    public void forward(double Power) {
        _Power = Power;
    }

    public void reverse(double Power) {
        _Power = -Power;
    }

}