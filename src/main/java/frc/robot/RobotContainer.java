// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

    // First create subsytems in container
    private final LEDSubsystem m_ledSubsystem = new LEDSubsystem(0); // PWM port 0
    
    private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_ledSubsystem);
    

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // create autoChooser
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

////////////////////////////////////////////////////////////////////////////////////////////////////////

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  // The operator's controller
  CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_robotDrive.resetGyroToFieldBackwards();
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
m_robotDrive.setDefaultCommand(
    new RunCommand(
        () -> {
            boolean manualSlowMode = m_driverController.rightBumper().getAsBoolean();
            m_robotDrive.updateDriveSlowMode(manualSlowMode); // Auto/Manual slow mode

            m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true
            );
        }, m_robotDrive
    )
);

    // Set the ball intake to in/out when not running based on internal state
    // m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.idleCommand());

    // register auto options to the shuffleboard           
    autoChooser.addOption("LE", "LE");
    autoChooser.addOption("LF", "LF");
    autoChooser.addOption("RC", "RC");
    autoChooser.addOption("RB", "RB");
    autoChooser.addOption("MDA", "MDA");
    autoChooser.addOption("MDC", "MDC");

    // Creating a new shuffleboard tab and adding the autoChooser
    //Shuffleboard.getTab("PathPlanner Autonomous").add(BlueautoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    //Shuffleboard.getTab("PathPlanner Autonomous").add(RedautoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    Shuffleboard.getTab("PathPlanner Autonomous").add(autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  private void configureButtonBindings() {

    // Driver Controller
    // Left Stick Button -> Set swerve to X
    m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());
  
    // Start Button -> Zero swerve heading
    m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());

    // Right Bumper -> Enable Slow Mode While Held
    m_driverController.rightBumper()
    .whileTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true)))
    .onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false)));


/////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // Operator Controller

    // Start Button -> Zero swerve heading
    m_operatorController.start().onTrue(m_robotDrive.zeroHeadingCommand());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   public Command getAutonomousCommand() {
    // Check if a path is selected
    
    if (autoChooser.getSelected() == null) {
        return null;
    }

    String selectedPath = autoChooser.getSelected();

    // Build and return the selected autonomous command
    return AutoBuilder.buildAuto(selectedPath);
}

}