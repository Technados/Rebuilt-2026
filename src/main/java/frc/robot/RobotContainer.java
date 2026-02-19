// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

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
    
    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

    private final FeederSubsystem m_feederSubsystem = new FeederSubsystem();

    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // create autoChooser
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    
    m_visionSubsystem.setDefaultCommand(
      new RunCommand( 
        () -> {
          double yawDeg = m_robotDrive.getPose().getRotation().getDegrees();
          double yawRateDegPerSec = m_robotDrive.getTurnRate();
          
          String[] llnames = {Constants.VisionConstants.kFrontLimelightName, Constants.VisionConstants.kBackLimelightName};

          for (String llname: llnames) {
            m_visionSubsystem.getVisionMeasurementMT2(llname, yawDeg, yawRateDegPerSec).ifPresent(m -> {
              if (Math.abs(yawRateDegPerSec) > 360.0) return;

              if (m.getPose().getTranslation().getDistance(m_robotDrive.getPose().getTranslation()) 
                > Constants.VisionConstants.kMaxAcceptedPoseJumpMeters) return;

              m_robotDrive.addVisionMeasurement(
                m.getPose(),
                m.getTimestampSeconds(),
                m.getStdDevs()
              );
            });
          }
        }, 
        m_visionSubsystem
      )
    );

    m_shooterSubsystem.setDefaultCommand(m_shooterSubsystem.idleShooterCommand());

    // Adds button to dashboard that resets the robot's pose to the test pose
    SmartDashboard.putData(
      "Reset Buttons/Reset Pose: Test Start",
      new InstantCommand(
        () -> m_robotDrive.resetOdometry(Constants.TestPoses.kTestStartPose),
        m_robotDrive
      )
    );

    // Adds button to dashboard that resets the intake pivot encoder to zero
    // SmartDashboard.putData(
    //   "Reset Buttons/Reset Pivot to Zero",
    //   m_intakeSubsystem.resetEncoder()
    // );

    // register auto options to the shuffleboard           
    autoChooser.addOption("LE", "LE");
    autoChooser.addOption("LF", "LF");
    autoChooser.addOption("RC", "RC");
    autoChooser.addOption("RB", "RB");
    autoChooser.addOption("MDA", "MDA");
    autoChooser.addOption("MDC", "MDC");

    // Creating a new shuffleboard tab and adding the autoChooser
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
    // m_driverController.rightBumper()
    // .whileTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true)))
    // .onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false)));

    m_driverController.x().whileTrue(m_intakeSubsystem.runIntakeCommand());

    m_driverController.y().onTrue(m_intakeSubsystem.tempZeroPivotAtInCommand());
    
    m_driverController.b().onTrue(m_intakeSubsystem.pivotInCommand());
    
    m_driverController.a().onTrue(m_intakeSubsystem.pivotOutCommand());

    m_driverController.leftBumper().whileTrue(m_intakeSubsystem.pivotJogCommand(0.1));

    m_driverController.rightBumper().whileTrue(m_intakeSubsystem.pivotJogCommand(-0.1));

/////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // Operator Controller

    // Start Button -> Zero swerve heading
    m_operatorController.start().onTrue(m_robotDrive.zeroHeadingCommand());

    m_operatorController.x().whileTrue(m_feederSubsystem.runFeederCommand());

    m_operatorController.y().whileTrue(m_shooterSubsystem.runShooterCommand());

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