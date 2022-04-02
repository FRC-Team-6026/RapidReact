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
    private final PIDController _targetPid = new PIDController(0.03, 0, 0.0007);
    private final double _setpoint = 0;

    public LimelightController() {
        super();
        init();
    }

    public void init(){
        _targetPid.setSetpoint(_setpoint);
        _targetPid.setTolerance(0.5);
        SendableRegistry.setName(_targetPid, "limelight Controller", "Target PID");
        setCameraPipeline();
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
        var ty = tyEntry.getNumber(0).doubleValue();
        var ft = 46.8 + -7.54*ty + 0.222*Math.pow(ty,2);

        // Shot power needs to change according to if TY is highr or lower. Higher number for TY means it is
        // closer therefore needing more backspin and less forward velocity, lower number means it is farther
        // away, and means it needs it needs more forward velocity and a little less backspin. 

            //Shooter
        shotPowers[0] = 0.341 + -7.36E-03*ft + 3.39E-03*Math.pow(ft, 2);
            //Kicker
        shotPowers[1] = 0.9; 
        return shotPowers;
    }
    
    public boolean isTargeting(){
        var pipelineEntry = _table.getEntry("pipeline");
        return pipelineEntry.getNumber(1).intValue() == 0;
    }

    public void setTargetPipeline(){
        var pipelineEntry = _table.getEntry("pipeline");
        pipelineEntry.setNumber(0);
    }

    public void setCameraPipeline(){
        var pipelineEntry = _table.getEntry("pipeline");
        pipelineEntry.setNumber(1);
    }
}