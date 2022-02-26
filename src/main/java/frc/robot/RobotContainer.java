// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  // The robot's subsystems and commands are defined here...
  private final DoubleSupplier _driverSpeedSupplier = () -> filterControllerInputs(-_driverController.getLeftY());
  private final DoubleSupplier _driverRotationSupplier = () -> filterControllerInputs(_driverController.getRightX());
  private final DoubleSupplier _driverShooterSupplier = () -> filteredShooterInput(_driverController.getLeftTriggerAxis(), _driverController.getRightTriggerAxis());
  private final Drive _drive = new Drive(_driverSpeedSupplier, _driverRotationSupplier);
  private final Intake _intake = new Intake();
  private final Shooter _shooter = new Shooter(_driverShooterSupplier);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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

    driverRightBumper.whenPressed(new InstantCommand(() -> _intake.runIn(), _intake), true)
      .whenReleased(new InstantCommand(()-> _intake.stop(), _intake), true);
    
    driverLeftBumper.whenPressed(new InstantCommand(() -> _intake.runOut(), _intake), true)
      .whenReleased(new InstantCommand(()-> _intake.stop(), _intake), true);

    driverAButton.whenPressed(new InstantCommand(() -> _intake.extendArm(), _intake), true);
    driverBButton.whenPressed(new InstantCommand(() -> _intake.retractArm(), _intake), true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
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
