package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BallColor;

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

    private final CANSparkMax _conveyor = new CANSparkMax(9, MotorType.kBrushless);
    private final RelativeEncoder _conveyorEncoder;
    private final SparkMaxPIDController _conveyorPID;

    private final I2C.Port _i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 _colorSensor = new ColorSensorV3(_i2cPort);
    private final DigitalInput _intakePhotocell = new DigitalInput(0);

    private final BooleanSupplier _isAtSetPowerSupplier;
    private boolean _isBallLoading = false;

    private final Solenoid _armControl = new Solenoid(12, PneumaticsModuleType.REVPH, 0);
    public Intake(BooleanSupplier isAtSetPowerSupplier) {
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

        _isAtSetPowerSupplier = isAtSetPowerSupplier;

        this.setDefaultCommand(new RunCommand(() -> {
            var ballAtShooter = ballAtShooter();
            SmartDashboard.putString("Ball At Shooter", ballAtShooter.toString());
            if (isBallAtIntake() && ballAtShooter == BallColor.none){
                _isBallLoading = true;
            }
            if (ballAtShooter != BallColor.none){
                _isBallLoading = false;
            }

            if (isBallAtIntake() || _isBallLoading){
                _intakePID.setReference(0.3, ControlType.kDutyCycle);
            }else{
                _intake.stopMotor();
            }

            if (_isAtSetPowerSupplier.getAsBoolean() || _isBallLoading){
                _conveyorPID.setReference(0.3, ControlType.kDutyCycle);
            } else {
                _conveyor.stopMotor();
            }
        }, this));
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
        _intakePID.setReference(-0.3, ControlType.kDutyCycle);
        _conveyorPID.setReference(-0.3, ControlType.kDutyCycle);

    }

    public void runIn() {
        var feedForwardVolts = _ks+(_runRpm*_kv);
        _armIntakePID.setReference(_runRpm, ControlType.kSmartVelocity, 0, feedForwardVolts, ArbFFUnits.kVoltage);
    }

    public void stop() {
        _armIntake.stopMotor();
    }
    
    public void extendArm() {
        _armControl.set(true);
    }

    public void retractArm(){
        _armControl.set(false);
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

    public BallColor ballAtShooter(){
        var r = _colorSensor.getRed();
        var b = _colorSensor.getBlue();
        var g = _colorSensor.getGreen();
        var x = _colorSensor.getProximity();
        SmartDashboard.putNumber("red", r);
        SmartDashboard.putNumber("blue", b);
        SmartDashboard.putNumber("green", g);
        SmartDashboard.putNumber("proximity", x);

        if(x > 100){
            if(b>250){
                return BallColor.blue;
            }  else if(r>300) {
                return BallColor.red;
            }       
        }
        return BallColor.none;
    }

    private boolean isBallAtIntake(){
        return _intakePhotocell.get();
    }
}
