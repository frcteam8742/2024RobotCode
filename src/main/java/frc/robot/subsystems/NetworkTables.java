package frc.robot.subsystems;

import edu.wpi.first.networktables.*;

public class NetworkTables {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // Standard robot network tables
    NetworkTable table = inst.getTable("datatable");

    public NetworkTableEntry _TopShooterP = table.getEntry("TSP");
    public NetworkTableEntry _TopShooterI = table.getEntry("TSI");
    public NetworkTableEntry _TopShooterD = table.getEntry("TSD");
    public NetworkTableEntry _TopShooterIz = table.getEntry("TSIz");
    public NetworkTableEntry _TopShooterFF = table.getEntry("TSFF");

    // public NetworkTableEntry _BottomShooterP = table.getDoubleTopic("BSP").subscribe(0.0);
    // public NetworkTableEntry _BottomShooterI = table.getDoubleTopic("BSI").subscribe(0.0);
    // public NetworkTableEntry _BottomShooterD = table.getDoubleTopic("BSD").subscribe(0.0);
    // public NetworkTableEntry _BottomShooterIz = table.getDoubleTopic("BSIz").subscribe(0.0);
    // public NetworkTableEntry _BottomShooterFF = table.getEntry("BSF").subscribe(0.0);

    // Shooter speed readout
    public NetworkTableEntry _TopShooterVel = table.getEntry("TopShooterVelocity");
    public NetworkTableEntry _BottomShooterVel = table.getEntry("BottomShooterVelocity");

    public NetworkTableEntry _TopShooterOutput = table.getEntry("TopShooterOutput");
    public NetworkTableEntry _BottomShooterOutput = table.getEntry("BottomShooterOutput");

    private ShooterSubsystem _Shooter;

    NetworkTables(ShooterSubsystem shooter) {
        _Shooter = shooter;
    }

    public void periodic() {
        
        _Shooter.TP = _TopShooterP.getDouble(0);
        _Shooter.TI = _TopShooterI.getDouble(0);
        _Shooter.TD = _TopShooterD.getDouble(0);
        _Shooter.TIZ = _TopShooterIz.getDouble(0);
        _Shooter.TFF = _TopShooterFF.getDouble(0);

        // _Shooter.BP = _BottomShooterP.get();
        // _Shooter.BI = _BottomShooterI.get();
        // _Shooter.BD = _BottomShooterD.get();
        // _Shooter.BIZ = _BottomShooterIz.get();
        // _Shooter.BFF = _BottomShooterFF.get();

        _TopShooterVel.setDouble(_Shooter._TopEncoder.getVelocity());
        _BottomShooterVel.setDouble(_Shooter._BottomEncoder.getVelocity());

        _TopShooterOutput.setDouble(_Shooter._TopShooterMotor.get());
        _BottomShooterOutput.setDouble(_Shooter._BottomShooterMotor.get());

    }
}