package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightController extends SubsystemBase {
    private final NetworkTableInstance _instance = NetworkTableInstance.getDefault();
    private final NetworkTable _table = _instance.getTable("limelight");
    private final PIDController _targetPid = new PIDController(0.05, 0.01, 0);
    private final double _setpoint = 0;

    public LimelightController() {
        super();
        init();
    }

    public void init(){
        _targetPid.setSetpoint(_setpoint);
        _targetPid.setTolerance(0.1);
        SendableRegistry.setName(_targetPid, "limelight Controller", "Target PID");
        turnLightOff();
    }

    @Override
    public void periodic() {
        var txEntry= _table.getEntry("tx");
        var tx = txEntry.getNumber(_setpoint);
        var tyEntry= _table.getEntry("ty");
        var ty = tyEntry.getNumber(_setpoint);
        SmartDashboard.putNumber("tx", tx.doubleValue());
        SmartDashboard.putNumber("ty", ty.doubleValue());
    }

    /**
     * Calculates the rotation necessary to track the upper
     * target
     * @return
     */
    public double trackTarget() {
        var txEntry= _table.getEntry("tx");
        var tx = txEntry.getNumber(_setpoint);
        var rotation = -_targetPid.calculate(tx.doubleValue());
        rotation = Math.min(rotation, 1);
        rotation = Math.max(rotation, -1);
        return rotation;
    }

    /**
     * Calculates the shooter and kicker shot powers based on
     * limelight target values
     * @return [shotPower, kickerPower]
     */
    public double[] getShotPower() {
        var shotPowers = new double[2];
        var tyEntry= _table.getEntry("ty");
        var ty = tyEntry.getNumber(0);

        // Shot power needs to change according to if TY is highr or lower. Higher number for TY means it is
        // closer therefore needing more backspin and less forward velocity, lower number means it is farther
        // away, and means it needs it needs more forward velocity and a little less backspin. 

        shotPowers[0] = 0.7;
        shotPowers[1] = 0.5;
        return shotPowers;
    }
        

    public void turnLightOn(){
        var ledEntry = _table.getEntry("ledMode");
        ledEntry.setNumber(3);
    }

    public void turnLightOff(){
        var ledEntry = _table.getEntry("ledMode");
        ledEntry.setNumber(1);
    }
}