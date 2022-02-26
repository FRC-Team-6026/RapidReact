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

    private final CANSparkMax _armIntake = new CANSparkMax(5, MotorType.kBrushless);
    private final RelativeEncoder _armIntakeEncoder;
    private final SparkMaxPIDController _armIntakePID;

    private final CANSparkMax _intake = new CANSparkMax(6, MotorType.kBrushless);
    private final RelativeEncoder _intakeEncoder;
    private final SparkMaxPIDController _intakePID;

    private final CANSparkMax _conveyor = new CANSparkMax(6, MotorType.kBrushless);
    private final RelativeEncoder _conveyorEncoder;
    private final SparkMaxPIDController _conveyorPID;
    public Intake() {
        super();
        _armIntakePID = _armIntake.getPIDController();
        _armIntakeEncoder = _armIntake.getEncoder();
        initializeMotor(_armIntake, _armIntakeEncoder, _armIntakePID, 9.9E-08, true, IdleMode.kCoast);

        _intakePID = _intake.getPIDController();
        _intakeEncoder = _intake.getEncoder();
        initializeMotor(_intake, _intakeEncoder, _intakePID, 0, true, IdleMode.kBrake);

        _conveyorPID = _conveyor.getPIDController();
        _conveyorEncoder = _conveyor.getEncoder();
        initializeMotor(_conveyor, _conveyorEncoder, _conveyorPID, 0, true, IdleMode.kBrake);
    }
    @Override
    public void periodic() {
      SmartDashboard.putNumber("Intake Velocity", _armIntakeEncoder.getVelocity());
      SmartDashboard.putNumber("Intake Position", _armIntakeEncoder.getPosition());
    }
    public void runOut() {
        var feedForwardVolts = -_ks-(_runRpm*_kv);
        _armIntakePID.setReference(-_runRpm, ControlType.kSmartVelocity, 0, feedForwardVolts, ArbFFUnits.kVoltage);
        SmartDashboard.putNumber("Feed forward voltage", feedForwardVolts);
        _intakePID.setReference(-0.4, ControlType.kDutyCycle);
        _conveyorPID.setReference(-0.4, ControlType.kDutyCycle);

    }

    public void runIn() {
        var feedForwardVolts = _ks+(_runRpm*_kv);
        _armIntakePID.setReference(_runRpm, ControlType.kSmartVelocity, 0, feedForwardVolts, ArbFFUnits.kVoltage);
        SmartDashboard.putNumber("Feed forward voltage", feedForwardVolts);
        _intakePID.setReference(0.4, ControlType.kDutyCycle);
        _conveyorPID.setReference(0.4, ControlType.kDutyCycle);
    }

    public void stop() {
        _armIntake.stopMotor();
        _intake.stopMotor();
        _conveyor.stopMotor();
    }
    
    private void initializeMotor (CANSparkMax motor,
        RelativeEncoder encoder,
        SparkMaxPIDController controller,
        double p,
        boolean isInverted,
        IdleMode idleMode){
    motor.restoreFactoryDefaults();
    motor.setIdleMode(idleMode);
    motor.setInverted(isInverted);
    controller.setP(p);
    controller.setOutputRange(-1, 1);
    controller.setSmartMotionMaxVelocity(2000, 0);
    controller.setSmartMotionMaxAccel(1000, 0);
    controller.setFeedbackDevice(encoder);
    motor.burnFlash();
}
}
