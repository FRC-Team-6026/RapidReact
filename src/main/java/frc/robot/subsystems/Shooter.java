// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final TalonSRX _shooter = new WPI_TalonSRX(4);
  private final int _timeoutMs = 30;
  private final double _p = 0.25;
  private final double _i = 0.001;
  private final double _d = 20;
  private final double _f = 1023.0/40000;
  private double _shooterRPM = 0;
  private XboxController _controller = new XboxController(0);
  private final double _maxRPM = 6000;
  /** Creates a new ExampleSubsystem. */
  public Shooter() {
    _shooter.configFactoryDefault();

    _shooter.setInverted(false);
    _shooter.setSensorPhase(true);

    _shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, _timeoutMs);
    _shooter.configNominalOutputForward(0, _timeoutMs);
    _shooter.configNominalOutputReverse(0, _timeoutMs);
    _shooter.configPeakOutputForward(1, _timeoutMs);
    _shooter.configPeakOutputReverse(-1, _timeoutMs);

    _shooter.config_kF(0, _f, _timeoutMs);
    _shooter.config_kP(0, _p, _timeoutMs);
    _shooter.config_kI(0, _i, _timeoutMs);
    _shooter.config_kD(0, _d, _timeoutMs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(_controller.getRightBumperPressed() && _shooterRPM < _maxRPM){
      _shooterRPM += 100;
    }
    if(_controller.getLeftBumperPressed() && _shooterRPM > -_maxRPM){
      _shooterRPM -= 100;
    }
    if(_controller.getBButtonPressed()){
      _shooterRPM = 0;
    }
    //var speed = -_controller.getRightY();
    //_shooter.set(ControlMode.PercentOutput, speed);
    setShooterRPM(_shooterRPM);
    updateDashboard();
  }

  private void setShooterRPM(double rpm){
    var pulsesPer100Ms = (rpm/600.0) * 4096.0;
    _shooter.set(ControlMode.Velocity, pulsesPer100Ms);
  }

  private void updateDashboard(){
    SmartDashboard.putNumber("Target RPM", _shooterRPM);
    var measuredRPM = (_shooter.getSelectedSensorVelocity()/4096.0) * 600.0;
    SmartDashboard.putNumber("Measured RPM", measuredRPM);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
