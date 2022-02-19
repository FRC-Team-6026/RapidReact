package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final double _runRpm = 2000;
    private final double _kv = 0.060519 / 60;
    private final double _ks = 0.20893;
    private final CANSparkMax _intake = new CANSparkMax(5, MotorType.kBrushless);
    private final RelativeEncoder _intakeEncoder;
    private final SparkMaxPIDController _intakePID;

    public Intake() {
        super();

        _intake.restoreFactoryDefaults();
        _intake.setIdleMode(IdleMode.kCoast);
        _intake.setInverted(true);
        _intakePID = _intake.getPIDController();
        _intakeEncoder = _intake.getEncoder();
        _intakePID.setP(9.9E-08);
        _intakePID.setOutputRange(-1, 1);
        _intakePID.setSmartMotionMaxVelocity(500, 0);
        _intakePID.setSmartMotionMaxAccel(100, 0);
        _intakePID.setFeedbackDevice(_intakeEncoder);
        _intake.burnFlash();
    }
    @Override
    public void periodic() {
      SmartDashboard.putNumber("Intake Velocity", _intakeEncoder.getVelocity());
      SmartDashboard.putNumber("Intake Position", _intakeEncoder.getPosition());
    }
    public void runOut() {
        var feedForwardVolts = -_ks-(_runRpm*_kv);
        _intakePID.setReference(-_runRpm, ControlType.kSmartVelocity, 0, feedForwardVolts, ArbFFUnits.kVoltage);
        SmartDashboard.putNumber("Feed forward voltage", feedForwardVolts);
        //_intakePID.setReference(0.2, ControlType.kDutyCycle);
    }

    public void runIn() {
        var feedForwardVolts = _ks+(_runRpm*_kv);
        _intakePID.setReference(_runRpm, ControlType.kSmartVelocity, 0, feedForwardVolts, ArbFFUnits.kVoltage);
        SmartDashboard.putNumber("Feed forward voltage", feedForwardVolts);
        //_intakePID.setReference(-0.2, ControlType.kDutyCycle);
    }

    public void stop() {
        _intake.stopMotor();
    }
    
}
