// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final XboxController m_driverController = new XboxController(Constants.Ports.primaryController);
  public final Joystick m_leftStick = new Joystick(1);
  public final Joystick m_rightStick = new Joystick(2);

  public final Drivetrain m_drivetrain = new Drivetrain();
  public final Intake m_intake = new Intake();
  public final Shooter m_shooter = new Shooter();

  public ShuffleboardTab driveTrainTab = Shuffleboard.getTab("Drivetrain");
  public NetworkTableEntry controlMode = driveTrainTab.add("Control Mode", true).withWidget(BuiltInWidgets.kToggleButton).getEntry();
  public ShuffleboardTab shooterIntakeTab = Shuffleboard.getTab("ShooterIntake");
  public NetworkTableEntry defaultBreakbeams = shooterIntakeTab.add("Default breakbeams", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
  public final NetworkButton changeControlMode = new NetworkButton(controlMode);
  public final JoystickButton intakeButton = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
  public final JoystickButton shooterButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Set default drivetrain command (user drive)
    m_drivetrain.setDefaultCommand(new UserArcadeDrive(
      () -> -m_driverController.getLeftY(),
      () -> -m_driverController.getRightX(),
      () -> m_driverController.getRightTriggerAxis() > .1,
      m_drivetrain));
    SmartDashboard.putData(m_drivetrain);
    SmartDashboard.putData(m_intake);
    //configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    changeControlMode.whenActive(new InstantCommand(() -> m_drivetrain.setDefaultCommand(new UserArcadeDrive(
      () -> -m_driverController.getLeftY(),
      () -> -m_driverController.getRightX(),
      () -> m_driverController.getRightTriggerAxis() > .1,
      m_drivetrain)), m_drivetrain));
    changeControlMode.whenInactive(new InstantCommand(() -> m_drivetrain.setDefaultCommand(new UserTankDrive(
      () -> -m_leftStick.getY(),
      () -> -m_rightStick.getY(),
      m_rightStick::getTrigger,
      m_drivetrain)), m_drivetrain));

      intakeButton.whenPressed(new ConditionalCommand(
        m_intake.sensorlessIntakeCommand().until(m_driverController::getRightBumperReleased),
        new UserIntake(m_driverController::getRightBumperPressed, m_intake),
        () -> defaultBreakbeams.getBoolean(false)), false);

      shooterButton.whenPressed(new ConditionalCommand(
        ShooterIntakeCommands.defaultedShootCommand(m_shooter, m_intake).until(m_driverController::getAButtonReleased),
        ShooterIntakeCommands.shootCommand(true, m_shooter, m_intake),
        () -> defaultBreakbeams.getBoolean(false)), false);

  }
}
