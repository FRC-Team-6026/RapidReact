package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final double _runRpm = 500;
    private final double _kv = 0.060519;
    private final double _ks = 0.20893;
    private final CANSparkMax _intake = new CANSparkMax(5, MotorType.kBrushless);
    private final RelativeEncoder _intakeEncoder;
    private final SparkMaxPIDController _intakePID;

    public Intake() {
        super();

        _intake.restoreFactoryDefaults();
        _intake.setIdleMode(IdleMode.kCoast);
        _intakePID = _intake.getPIDController();
        _intakeEncoder = _intake.getEncoder();
        _intakePID.setP(0.09379);
        _intakePID.setOutputRange(-1, 1);
        _intakePID.setSmartMotionMaxAccel(500, 0);
        _intakePID.setFeedbackDevice(_intakeEncoder);
    }

    public void runIn() {
        var feedForwardVolts = -_ks-(_runRpm*_kv);
        _intakePID.setReference(-_runRpm, ControlType.kSmartVelocity, 0, feedForwardVolts, ArbFFUnits.kVoltage);
    }

    public void runOut() {
        var feedForwardVolts = _ks+(_runRpm*_kv);
        _intakePID.setReference(_runRpm, ControlType.kSmartVelocity, 0, feedForwardVolts, ArbFFUnits.kVoltage);
    }
    
}
