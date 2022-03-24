// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final double kLowerDeadband = 0.17;
  private static final double kUpperDeadband = 0.95;
  private final XboxController _driverController = new XboxController(0);
  private final SendableChooser<String> _teamChooser = new SendableChooser<String>();
  // The robot's subsystems and commands are defined here...
  private final DoubleSupplier _driverSpeedSupplier = () -> filterControllerInputs(-_driverController.getLeftY());
  private final DoubleSupplier _driverRotationSupplier = () -> filterControllerInputs(_driverController.getRightX());
  private final Drive _drive = new Drive(_driverSpeedSupplier, _driverRotationSupplier);
  private final Shooter _shooter = new Shooter();
  private final Intake _intake = new Intake(() -> _shooter.isAtSetPower(), _teamChooser);
  private final Elevator _elevator = new Elevator();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    _teamChooser.addOption("Blue", "Blue");
    _teamChooser.addOption("Red", "Red");
    SmartDashboard.putData("Team Color", _teamChooser);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    var driverRightBumper = new JoystickButton(_driverController, XboxController.Button.kRightBumper.value);
    var driverLeftBumper = new JoystickButton(_driverController, XboxController.Button.kLeftBumper.value);
    var driverAButton = new JoystickButton(_driverController, XboxController.Button.kA.value);
    var driverBButton = new JoystickButton(_driverController, XboxController.Button.kB.value);
    var driverYButton = new JoystickButton(_driverController, XboxController.Button.kY.value);
    var driverXButton = new JoystickButton(_driverController, XboxController.Button.kX.value);
    var leftTrigger = new Trigger(() -> {
      return _driverController.getLeftTriggerAxis() > 0.5;
    });
    var rightTrigger = new Trigger(() -> {
      return _driverController.getRightTriggerAxis() > 0.5;
    });

    leftTrigger.whenActive(new InstantCommand(() -> {
      _intake.runIn();
      _intake.extendArm();
      _intake.overrideIntakeSensor(true);
    }, _intake), true)
    .whenInactive(new InstantCommand(() -> {
      _intake.stop();
      _intake.retractArm();
      _intake.overrideIntakeSensor(false);
    },_intake), true);

    var reverseConveyorCommand = new StartEndCommand(()-> {_intake.reverseConveyor();},
    () -> {_intake.stop();}, _intake).withTimeout(0.1);

    var fireCommand = new InstantCommand(() -> {_shooter.fire();}, _shooter);

   rightTrigger.whenActive(reverseConveyorCommand.andThen(fireCommand))
    .whenInactive(new InstantCommand(() -> {
    _shooter.stop();
   },_shooter), true);

    driverRightBumper.whenPressed(new InstantCommand(() -> _elevator.extend(), _elevator), true)
      .whenReleased(new InstantCommand(()-> _elevator.stop(), _elevator), true);
    
    driverLeftBumper.whenPressed(new InstantCommand(() -> _elevator.retract(), _elevator), true)
      .whenReleased(new InstantCommand(()-> _elevator.stop(), _elevator), true);

    driverAButton.whenPressed(new InstantCommand(() -> _intake.overrideIntakeSensor(true), _intake), true)
      .whenReleased(new InstantCommand(()->  _intake.overrideIntakeSensor(false), _intake), true);
    driverYButton.whenPressed(new InstantCommand(() -> {
      _shooter.setShooterPower(0.5);
      _shooter.setKickerPower(0.9);
    }, _shooter), true);
    driverXButton.whenPressed(new InstantCommand(() -> {
      _shooter.setShooterPower(0.3);
      _shooter.setKickerPower(0.3);
    }, _shooter), true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //first, fire
    var fireCommand = new StartEndCommand(() -> {
      _shooter.setShooterPower(0.3);
      _shooter.setKickerPower(0.3);
      _shooter.fire();
    },() -> {
       _shooter.stop();
       _shooter.setShooterPower(0.5);
       _shooter.setKickerPower(0.9);
    }, _shooter)
      .withTimeout(3);


    var driveBackCommand = new StartEndCommand(() -> {
      _drive.setSpeedSupplier(() -> -0.3);
    } , () -> {
      _drive.setSpeedSupplier(_driverSpeedSupplier);
    }, _shooter).withTimeout(3);
    
    return fireCommand.andThen(driveBackCommand);

  }

  private static double filterControllerInputs(double input) {
    if (Math.abs(input) < kLowerDeadband){
      return 0;
    }

    //The XBox controller sometimes won't go all the way to 1
    if (Math.abs(input) > kUpperDeadband){
      return input > 0 ? 1 : -1;
    }

    //cubing input so we have more control at lower values
    var filtered = input * input * input;

    return filtered;
  }

  private static double filteredShooterInput(double leftInput, double rightInput)
  {
    var leftFiltered = filterControllerInputs(leftInput);
    var rightFiltered = filterControllerInputs(rightInput);
    if (leftFiltered > rightFiltered){
      return -leftFiltered;
    } else {
      return rightFiltered;
    }
  }
}
