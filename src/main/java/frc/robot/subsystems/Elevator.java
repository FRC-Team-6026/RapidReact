package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final CANSparkMax _elevator = new CANSparkMax(11, MotorType.kBrushless);
    private final RelativeEncoder _elevatorEncoder;
    private final SparkMaxPIDController _elevatorPID;
    private final SparkMaxLimitSwitch _forwardLimitSwitch;
    private final SparkMaxLimitSwitch _reverseLimitSwitch;
    private double _maxPosition = 212;
    private double _minPosition = 1;
    private double _currentReference = 0;
    private final double _drivingSpeed = 0.5;
    
    public Elevator() {
        super();
        _elevatorEncoder=_elevator.getEncoder();
        _elevatorPID=_elevator.getPIDController();
        _elevator.restoreFactoryDefaults();
        _elevator.setIdleMode(IdleMode.kBrake);
        _elevatorPID.setFeedbackDevice(_elevatorEncoder);
        _forwardLimitSwitch = _elevator.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        _reverseLimitSwitch = _elevator.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    }

    @Override
    public void periodic() {
        var elevatorEncoder = _elevatorEncoder.getPosition();
        SmartDashboard.putNumber("elevator position", elevatorEncoder);
        //if at the bottom of travel, 0 out the encoder position.
        if (_reverseLimitSwitch.isPressed()){
            _elevatorEncoder.setPosition(0);
        } else if (_forwardLimitSwitch.isPressed()){
            _maxPosition = elevatorEncoder - 1;
        }

        if (elevatorEncoder < _minPosition && _currentReference < 0){
            _currentReference = 0;
            _elevatorPID.setReference(_currentReference, ControlType.kDutyCycle);
        }

        if (elevatorEncoder > _maxPosition && _currentReference > 0){
            _currentReference = 0;
            _elevatorPID.setReference(_currentReference, ControlType.kDutyCycle);
        }
    }

    public void extend() {
        var elevatorEncoder = _elevatorEncoder.getPosition();

        if (elevatorEncoder > _maxPosition)
            _currentReference = 0;
        else
            _currentReference = _drivingSpeed;

        _elevatorPID.setReference(_currentReference, ControlType.kDutyCycle);
    }
    public void retract() {
        var elevatorEncoder = _elevatorEncoder.getPosition();

        if (elevatorEncoder < _minPosition)
            _currentReference = 0;
        else
            _currentReference = -_drivingSpeed;

        _elevatorPID.setReference(_currentReference, ControlType.kDutyCycle);
    }
    public void stop() {
        _elevator.stopMotor();
    }
}
