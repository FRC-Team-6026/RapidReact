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
    private double _shooterPower;
    private double _kickerPower;
    private final double _maxDiff = 200;

    public Shooter() {
        super();
        _shooterPID = _shooter.getPIDController();
        _shooterEncoder = _shooter.getEncoder();
        _kickerPID = _kicker.getPIDController();
        _kickerEncoder = _kicker.getEncoder();
        initializeMotor(_shooter, _shooterEncoder, _shooterPID, 1.8162E-08);
        initializeMotor(_kicker, _kickerEncoder, _kickerPID, 9.4347E-08);
        setKickerPower(0.3);
        setShooterPower(0.3);
        SmartDashboard.putNumber("Shooter Power", _shooterPower);
        SmartDashboard.putNumber("Kicker Power", _kickerPower);
    }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Shooter Velocity", _shooterEncoder.getVelocity());
      SmartDashboard.putNumber("Shooter Position", _shooterEncoder.getPosition());
      SmartDashboard.putNumber("Kicker Velocity", _kickerEncoder.getVelocity());
      SmartDashboard.putNumber("Kicker Position", _kickerEncoder.getPosition());

      var shooterPower = SmartDashboard.getNumber("Shooter Power", 0.5);
      var kickerPower = SmartDashboard.getNumber("Kicker Power", 0.5);

        if(shooterPower!=_shooterPower){
            _shooterPower=shooterPower;
        }
        if(kickerPower!=_kickerPower){
            _kickerPower=kickerPower;
        }
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

    public void setShooterPower(double shooterPower){
        _shooterPower = shooterPower;
        SmartDashboard.putNumber("Shooter Power", _shooterPower);
    }

    public void setKickerPower(double kickerPower){
        _kickerPower = kickerPower;
        SmartDashboard.putNumber("Kicker Power", _kickerPower);
    }

    public void fire(){
        var shooterRpm = _shooterPower*_maxRpm;
        var kickerRpm = _kickerPower*_maxRpm;
        SmartDashboard.putNumber("Kicker Rpm", kickerRpm);
        SmartDashboard.putNumber("Shooter Rpm", shooterRpm);
        var shooterVolts = _ksShooter + shooterRpm*_kvShooter;
        _shooterPID.setReference(shooterRpm, ControlType.kSmartVelocity, 0, shooterVolts, ArbFFUnits.kVoltage);
        var kickerVolts = (-_ksKicker - kickerRpm*_kvKicker);
        _kickerPID.setReference(-kickerRpm, ControlType.kSmartVelocity, 0, kickerVolts, ArbFFUnits.kVoltage);
    }

    public void stop(){
        _shooter.stopMotor();
        _kicker.stopMotor();
    }

    public boolean isAtSetPower(){
        var shooterRpm = _shooterPower*_maxRpm;
        var kickerRpm = -_kickerPower*_maxRpm;
        var shooterVelocity = _shooterEncoder.getVelocity();
        var kickerVelocity = _kickerEncoder.getVelocity();
        var shooterDiff = Math.abs(shooterVelocity-shooterRpm);
        var kickerDiff = Math.abs(kickerVelocity-kickerRpm);

        if (shooterDiff < _maxDiff && kickerDiff < _maxDiff){
            return true;
        }
        return false;

    }

}
