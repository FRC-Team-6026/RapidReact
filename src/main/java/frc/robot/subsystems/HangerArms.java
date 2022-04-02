package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangerArms extends SubsystemBase {
    private final CANSparkMax _hangerArm = new CANSparkMax(10, MotorType.kBrushless);
    private final RelativeEncoder _hangerArmEncoder;
    private final SparkMaxPIDController _hangerArmPID;
    private final DoubleSupplier _hangerArmsInput;

    public HangerArms(DoubleSupplier hangerArmInput) {
        super();
        _hangerArmsInput = hangerArmInput;
        _hangerArm.restoreFactoryDefaults();
        _hangerArmEncoder = _hangerArm.getEncoder();
        _hangerArmPID = _hangerArm.getPIDController();
        _hangerArmPID.setFeedbackDevice(_hangerArmEncoder);
        _hangerArm.setIdleMode(IdleMode.kBrake);
        _hangerArm.burnFlash();

        this.setDefaultCommand(new FunctionalCommand(() -> {/*do nothing on init*/},
        // do arcade drive by default
        () -> {
            var input = _hangerArmsInput.getAsDouble() / 4;
            _hangerArmPID.setReference(input, ControlType.kDutyCycle);
        },
        //when interrupted set PID controls to voltage and default to 0 to stop
        interrupted ->
        {
        _hangerArmPID.setReference(0, ControlType.kVoltage);
        },
        //never end
        () -> {return false;},
        this));
    }
    @Override
    public void periodic() {
        var hangerArmEncoder = _hangerArmEncoder.getPosition();
        SmartDashboard.putNumber("hanger arm position", hangerArmEncoder);
    }
}

