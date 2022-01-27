package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.*;

public class Drive extends SubsystemBase
{
    private final CANSparkMax _left1 = new CANSparkMax(9, MotorType.kBrushless);
    private final CANSparkMax _left2 = new CANSparkMax(10, MotorType.kBrushless);
    private final RelativeEncoder _leftEncoder;
    private final SparkMaxPIDController _leftPid;

    private final CANSparkMax _right1 = new CANSparkMax(8, MotorType.kBrushless);
    private final CANSparkMax _right2 = new CANSparkMax(7, MotorType.kBrushless);
    private final RelativeEncoder _rightEncoder;
    private final SparkMaxPIDController _rightPid;

    private DoubleSupplier _speedSupplier;
    private DoubleSupplier _rotationSupplier;

    private final ADIS16448_IMU _imu = new ADIS16448_IMU();

    private final DifferentialDriveOdometry _odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(_imu.getGyroAngleZ()));

    public Drive(DoubleSupplier speedSupplier, DoubleSupplier rotationSupplier) {
        super();
        _speedSupplier = speedSupplier;
        _rotationSupplier = rotationSupplier;

        setupSparkMax(_left1, false);
        setupSparkMax(_left2, false);
        setupSparkMax(_right1, true);
        setupSparkMax(_right2, true);
        _left2.follow(_left1);
        _right2.follow(_right1);

        _leftPid = _left1.getPIDController();
        _rightPid = _right1.getPIDController();

        _leftEncoder = _left1.getEncoder();
        _rightEncoder = _right1.getEncoder();

        setupPid(_leftPid, _leftEncoder);
        setupPid(_rightPid, _rightEncoder);

        _leftEncoder.setPositionConversionFactor(Constants.kMetersPerMotorRevolution);
        _leftEncoder.setVelocityConversionFactor(Constants.kMetersPerSecondPerRPM);

        _rightEncoder.setPositionConversionFactor(Constants.kMetersPerMotorRevolution);
        _rightEncoder.setVelocityConversionFactor(Constants.kMetersPerSecondPerRPM);

        resetEncoders();

        _left1.burnFlash();
        _left2.burnFlash();
        _right1.burnFlash();
        _right2.burnFlash();

        this.setDefaultCommand(new FunctionalCommand(() -> {/*do nothing on init*/},
          // do arcade drive by default
          () -> {arcadeDrive();},
          //when interrupted set PID controls to voltage and default to 0 to stop
          interrupted ->
          {
            _leftPid.setReference(0, ControlType.kVoltage);
            _rightPid.setReference(0, ControlType.kVoltage);
          },
          //never end
          () -> {return false;},
          this));
    }

    @Override
    public void periodic() {
        var leftEncoder = _leftEncoder.getPosition();
        var rightEncoder = _rightEncoder.getPosition();
        var rotation = Rotation2d.fromDegrees(_imu.getGyroAngleZ());
        _odometry.update(rotation, leftEncoder, rightEncoder);
        SmartDashboard.putNumber("leftEncoder", leftEncoder);
        SmartDashboard.putNumber("rightEncoder", rightEncoder);
        SmartDashboard.putNumber("rotation", rotation.getDegrees());
    }

    /**
     * Returns the current estimated pose of the robot
     * 
     * @return The pose
     */
    public Pose2d getPose() {
        return _odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot
     * 
     * @return the current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(_leftEncoder.getVelocity(), _rightEncoder.getVelocity());
    }

    public void resetEncoders() {
        _leftEncoder.setPosition(0);
        _rightEncoder.setPosition(0);
    }

    /**
     * Resets the odometry to the specified pose.
     * 
     * @param pose The pose to set the odometry to.
     */
    public void resetOdometry(Pose2d pose){
        resetEncoders();
        _odometry.resetPosition(pose, Rotation2d.fromDegrees(_imu.getGyroAngleZ()));
    }

    /**
     * Drives the robot using arcade controls.
     * 
     * @param forward the commanded forward movement
     * @param rotation the commanded rotation
     */
    public void arcadeDrive() {
        var speed = _speedSupplier.getAsDouble();
        var rotation = _rotationSupplier.getAsDouble();

        var leftLinearRpm = speed * Constants.kMaxRpm;
        var rightLinearRpm = speed * Constants.kMaxRpm;
        var leftRotationRpm = rotation * Constants.kRotationDiffRpm;
        var rightRotationRpm = -rotation * Constants.kRotationDiffRpm;

        var leftVelocityRpm = leftLinearRpm + leftRotationRpm;
        var rightVelocityRpm = rightLinearRpm + rightRotationRpm;

        var leftFeedForward = leftVelocityRpm * Constants.kvVoltMinutesPerMotorRotation;
        if (leftFeedForward != 0) {
          leftFeedForward = leftFeedForward > 0 ? leftFeedForward + Constants.ksVolts : leftFeedForward - Constants.ksVolts;
        }
        var rightFeedForward = rightVelocityRpm * Constants.kvVoltMinutesPerMotorRotation;
        if (rightFeedForward != 0) {
          rightFeedForward = rightFeedForward > 0 ? rightFeedForward + Constants.ksVolts : rightFeedForward - Constants.ksVolts;
        }

        _leftPid.setReference(leftVelocityRpm, ControlType.kVelocity, 0, leftFeedForward, ArbFFUnits.kVoltage);
        _rightPid.setReference(rightVelocityRpm, ControlType.kVelocity, 0, rightFeedForward, ArbFFUnits.kVoltage);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     * 
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts){
        _leftPid.setReference(leftVolts, ControlType.kVoltage);
        _rightPid.setReference(rightVolts, ControlType.kVoltage);
    }

    /**
     * Gets the average position of the two encoders.
     * 
     * @return the average of the two encoder positions.
     */
    public double getAverageEncoderDistance() {
        return (_leftEncoder.getPosition() + _rightEncoder.getPosition()) / 2.0;
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        _imu.reset();
    }

    /**
     * Returns the heading of the robot
     * 
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        //getGyroAngleZ is an accumulated heading, and is not normalized -180 to 180
        return Rotation2d.fromDegrees(_imu.getGyroAngleZ()).getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     * 
     * @return the turn rate of the robot in degrees per second.
     */
    public double getTurnRate() {
        //TODO we need to figure out how to get turn rate now that the IMU libraries have changed.
        return 0;
    }

    public void setSpeedSupplier(DoubleSupplier speedSupplier) {
        _speedSupplier = speedSupplier;
    }

    public void setRotationSupplier(DoubleSupplier rotationSupplier) {
        _rotationSupplier = rotationSupplier;
    }

    private void setupSparkMax(CANSparkMax motor, boolean inverted)
    {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(inverted);
    }

    private void setupPid(SparkMaxPIDController controller, RelativeEncoder encoder){
        controller.setFF(Constants.kFf);
        controller.setP(Constants.kP);
        controller.setI(Constants.kI);
        controller.setD(Constants.kD);
        controller.setOutputRange(Constants.minOutput, Constants.maxOutput);
        controller.setSmartMotionMaxVelocity(Constants.kMaxRpm, 0);
        controller.setSmartMotionMaxAccel(Constants.kMaxAccelRpmPerSec, 0);
        controller.setFeedbackDevice(encoder);
    }
}