package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
    
    public Elevator() {
        super();
        _elevatorEncoder=_elevator.getEncoder();
        _elevatorPID=_elevator.getPIDController();
        _elevator.restoreFactoryDefaults();
        _elevator.setIdleMode(IdleMode.kBrake);
        _elevatorPID.setFeedbackDevice(_elevatorEncoder);
    }

    @Override
    public void periodic() {
        var elevatorEncoder = _elevatorEncoder.getPosition();
        SmartDashboard.putNumber("elevator position", elevatorEncoder);
    }

    public void extend() {
        _elevatorPID.setReference(0.2, ControlType.kDutyCycle);
    }
    public void retract() {
        _elevatorPID.setReference(-0.2, ControlType.kDutyCycle);
    }
    public void stop() {
        _elevator.stopMotor();
    }
}
