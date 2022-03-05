package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final double _maxRpm = 5000;
    private final double _kvShooter = 0.12123 / 60;
    private final double _kvKicker = 0.12317 / 60;
    private final double _ksShooter = 0.0811;
    private final double _ksKicker = 0.11662;
    private final CANSparkMax _shooter = new CANSparkMax(7, MotorType.kBrushless);
    private final RelativeEncoder _shooterEncoder;
    private final SparkMaxPIDController _shooterPID;
    private final CANSparkMax _kicker = new CANSparkMax(8, MotorType.kBrushless);
    private final RelativeEncoder _kickerEncoder;
    private final SparkMaxPIDController _kickerPID;
    private DoubleSupplier _shooterSupplier;
    private final double _maxDiff = 100;

    public Shooter(DoubleSupplier shooterSupplier) {
        super();
        _shooterPID = _shooter.getPIDController();
        _shooterEncoder = _shooter.getEncoder();
        _kickerPID = _kicker.getPIDController();
        _kickerEncoder = _kicker.getEncoder();
        initializeMotor(_shooter, _shooterEncoder, _shooterPID, 1.8162E-08);
        initializeMotor(_kicker, _kickerEncoder, _kickerPID, 9.4347E-08);
        _shooterSupplier = shooterSupplier;

        this.setDefaultCommand(new FunctionalCommand(() -> {/*do nothing on init*/},
        // fire by default
        () -> {fire();},
        //when interrupted set PID controls to voltage and default to 0 to stop
        interrupted ->
        {
        _shooterPID.setReference(0, ControlType.kVoltage);
        _kickerPID.setReference(0, ControlType.kVoltage);
        },
        //never end
        () -> {return false;},
        this));
    }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Shooter Velocity", _shooterEncoder.getVelocity());
      SmartDashboard.putNumber("Shooter Position", _shooterEncoder.getPosition());
      SmartDashboard.putNumber("Kicker Velocity", _kickerEncoder.getVelocity());
      SmartDashboard.putNumber("Kicker Position", _kickerEncoder.getPosition());
    }

    private void initializeMotor (CANSparkMax motor,
        RelativeEncoder encoder,
        SparkMaxPIDController controller,
        double p){
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        //motor.setInverted(true);
        controller.setP(p);
        controller.setOutputRange(-1, 1);
        controller.setSmartMotionMaxVelocity(_maxRpm, 0);
        controller.setSmartMotionMaxAccel(1000, 0);
        controller.setFeedbackDevice(encoder);
        motor.burnFlash();
    }

    public void fire(){
        var power = _shooterSupplier.getAsDouble();
        var setRpm = power*_maxRpm;
        SmartDashboard.putNumber("setRpm", setRpm);
        var shooterVolts = _ksShooter + setRpm*_kvShooter;
        _shooterPID.setReference(setRpm, ControlType.kSmartVelocity, 0, shooterVolts, ArbFFUnits.kVoltage);
        var kickerVolts = (-_ksKicker - setRpm*_kvKicker);
        _kickerPID.setReference(-setRpm, ControlType.kSmartVelocity, 0, kickerVolts, ArbFFUnits.kVoltage);
    }

    public boolean isAtSetPower(){
        var power = _shooterSupplier.getAsDouble();
        if (power < 0.1){
            return false;
        }
        var setRpm = power*_maxRpm;
        var shooterVelocity = _shooterEncoder.getVelocity();
        var kickerVelocity = _kickerEncoder.getVelocity();
        var shooterDiff = Math.abs(shooterVelocity-setRpm);
        var kickerDiff = Math.abs(kickerVelocity-setRpm);

        if (shooterDiff < _maxDiff && kickerDiff < _maxDiff){
            return true;
        }
        return false;

    }

}
