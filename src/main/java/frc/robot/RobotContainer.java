// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TagCenteringCommand;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The camera used for april tag detection
  public final PhotonCamera m_aprilTagCamera = new PhotonCamera("USB_ATag_Camera");

  // The driver's controller
  private CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  // Integration with shuffleboard to get the auto selected by the technician 
  private SendableChooser<Command> m_autoChooser;

  // The default command for the robot's drivetrain
  public final RunCommand m_defaultDriveCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Define the default drive command
    m_defaultDriveCommand = new RunCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        () ->
            m_robotDrive.drive(
                -MathUtil.applyDeadband(
                    m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
        m_robotDrive);

    // Configure default commands
    m_robotDrive.setDefaultCommand(this.m_defaultDriveCommand);

    // Define the auto chooser
    m_autoChooser = AutoBuilder.buildAutoChooser();

    // Integrate the autonomous chooser into shuffleboard
    SmartDashboard.putData("Autonomous", m_autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    Trigger slowModeTrigger = m_driverController.rightTrigger();

    slowModeTrigger.onTrue(m_robotDrive.enableSlowMode());
    slowModeTrigger.onFalse(m_robotDrive.disableSlowMode());

    Trigger zeroHeadingTrigger = m_driverController.b();

    zeroHeadingTrigger.onTrue(m_robotDrive.zeroHeading());

    Trigger centerOnTagTrigger = m_driverController.y();

    centerOnTagTrigger.onTrue(new TagCenteringCommand(this));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected().andThen();
  }
}
