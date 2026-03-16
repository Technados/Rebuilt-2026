// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.shooting.ShotMap;
import frc.robot.shooting.ShotMapData;
import frc.robot.shooting.ShotParameters;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TestingSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

    // First create subsytems in container
    private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();
    
    private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_ledSubsystem);
    
    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

    private final FeederSubsystem m_feederSubsystem = new FeederSubsystem();

    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

    private final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();

    // ShotMap is pure math/data; safe to live in RobotContainer.
    private final ShotMap m_shotMap = ShotMapData.createAllianceZoneShotMap();

    private final TestingSubsystem m_testing = new TestingSubsystem(); 

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

  private boolean passLeft;
  private boolean passRight;

  private DoubleSupplier rpmSupplier;
  private DoubleSupplier hoodSupplier;

  private BooleanSupplier readySupplier;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> {
          double x = -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband);
          double y = -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband);
          double rot = -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband);

          boolean manualSlowMode = m_driverController.rightBumper().getAsBoolean();
          m_robotDrive.updateDriveSlowMode(manualSlowMode); // Auto/Manual slow mode

          if (m_robotDrive.isTraversalMode()) {
            m_robotDrive.driveTraversalAssist(x, y, true);
          } else {
            m_robotDrive.drive(x, y, rot, true);
          }
        },
        m_robotDrive
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

    //m_intakeSubsystem.setDefaultCommand(m_intakeSubsystem.pivotIdleCommand());

    // Adds button to dashboard that resets the robot's pose to the test pose
    SmartDashboard.putData(
      "Reset Buttons/Reset Pose: Test Start",
      new InstantCommand(
        () -> m_robotDrive.resetOdometry(Constants.TestPoses.kTestStartPose),
        m_robotDrive
      )
    );

    // Pathplanner Commands
    NamedCommands.registerCommand("Intake", m_intakeSubsystem.runIntakeCommand());
    NamedCommands.registerCommand("Pivot-In", m_intakeSubsystem.pivotInCommand());
    NamedCommands.registerCommand("Pivot-Out", m_intakeSubsystem.pivotOutCommand());

    NamedCommands.registerCommand("Shoot", 
      new ParallelCommandGroup(
        m_shooterSubsystem.holdVelocityCommand(3000),
        m_hoodSubsystem.holdPositionCommand(0),
        m_feederSubsystem.feedWhen(() -> m_shooterSubsystem.shooterAtVelocity())
      )
    ); // Test positions/velocity later

    NamedCommands.registerCommand("Shoot-Far", 
      new ParallelCommandGroup(
        m_shooterSubsystem.holdVelocityCommand(3500),
        m_hoodSubsystem.holdPositionCommand(0),
        m_feederSubsystem.feedWhen(() -> m_shooterSubsystem.shooterAtVelocity())
      )
    ); // Test positions/velocity later

    // register auto options to the shuffleboard 
    autoChooser.addOption("null", null);         
    autoChooser.addOption("RT-O", "RT-O");
    autoChooser.addOption("RB-C", "RB-C");
    autoChooser.addOption("LT-D", "LT-D");
    autoChooser.addOption("HC-O", "HC-O");
    autoChooser.addOption("HC-D", "HC-D");
    autoChooser.addOption("1-Meter", "1-Meter");

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

    // X Button -> Run intake while true
    m_driverController.rightTrigger().whileTrue(m_intakeSubsystem.runIntakeCommand());

    m_driverController.leftTrigger()
      .onTrue(new InstantCommand(() -> m_robotDrive.enableTraversalMode()))
      .onFalse(new InstantCommand(() -> m_robotDrive.disableTraversalMode()));
    
    // Y Button -> Zero pivot on true
    m_driverController.y().whileTrue(m_intakeSubsystem.tempZeroPivotAtInCommand());
    
    // X Button -> Agitate pivot while true
    // m_driverController.x().whileTrue(
    //   m_intakeSubsystem.pivotAgitateCommand()
    // );
    
    // B Button -> Retract pivot on true
    m_driverController.b().onTrue(m_intakeSubsystem.pivotInCommand());
    
    // A Button -> Extend pivot on true
    m_driverController.a().onTrue(m_intakeSubsystem.pivotOutCommand());

    m_driverController.povLeft().whileTrue(m_intakeSubsystem.pivotJogCommand(0.1));

    m_driverController.povRight().whileTrue(m_intakeSubsystem.pivotJogCommand(-0.1));

/////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // Operator Controller

    // Start Button -> Zero swerve heading
    m_operatorController.start().onTrue(m_robotDrive.zeroHeadingCommand());

    passLeft = m_operatorController.a().getAsBoolean();
    passRight = m_operatorController.b().getAsBoolean();

    // Operator LT = "Ready to Shoot" (aim + set shooter rpm + set hood)
    
    var hubSupplier = (Supplier<Translation2d>) FieldConstants::getAllianceHub;

    var distanceSupplier = (DoubleSupplier) () ->
      m_robotDrive.getPose().getTranslation().getDistance(hubSupplier.get());

    var leftFieldTargetSupplier = 
      (Supplier<Translation2d>) FieldConstants::getLeftFieldTarget;

    var rightFieldTargetSupplier = 
      (Supplier<Translation2d>) FieldConstants::getRightFieldTarget;

    if (passLeft) {
      distanceSupplier = (DoubleSupplier) () ->
        m_robotDrive.getPose().getTranslation().getDistance(hubSupplier.get());

    } else if (passRight) {
      distanceSupplier = (DoubleSupplier) () ->
        m_robotDrive.getPose().getTranslation().getDistance(hubSupplier.get());

    }

    this.setShotSuppliers(distanceSupplier);

    m_operatorController.leftTrigger()
      .whileTrue(
          // Keep updating continuously while held
          m_robotDrive.aimAtCommand(hubSupplier)
            .alongWith(m_shooterSubsystem.holdVelocityCommand(rpmSupplier))
            .alongWith(m_hoodSubsystem.holdPositionCommand(hoodSupplier))
      );

    m_operatorController.povRight()
      .whileTrue(
          // Keep updating continuously while held
          m_robotDrive.aimAtCommand(leftFieldTargetSupplier)
            .alongWith(m_shooterSubsystem.holdVelocityCommand(rpmSupplier))
            .alongWith(m_hoodSubsystem.holdPositionCommand(hoodSupplier))
      );

    m_operatorController.povLeft()
      .whileTrue(
          // Keep updating continuously while held
          m_robotDrive.aimAtCommand(rightFieldTargetSupplier)
            .alongWith(m_shooterSubsystem.holdVelocityCommand(rpmSupplier))
            .alongWith(m_hoodSubsystem.holdPositionCommand(hoodSupplier))
      );

    // Operator RT = Fire (feed only when ready)
    m_operatorController.rightTrigger()
      .whileTrue(
        m_feederSubsystem.feedWhen(readySupplier)
          .alongWith(m_intakeSubsystem.pivotAgitateCommand(readySupplier))
      );

    m_operatorController.x()
      .whileTrue(
        m_shooterSubsystem.holdVelocityCommand(m_testing.shootingVelocity)
      );

    m_operatorController.y()
      .whileTrue(
        m_feederSubsystem.runFeederCommand()
      );

    //m_operatorController.a().whileTrue(m_hoodSubsystem.hoodJogCommand(0.01));

    //m_operatorController.b().whileTrue(m_hoodSubsystem.hoodJogCommand(-0.01));

  }

  public void setShotSuppliers(DoubleSupplier distance) {
    var shotSupplier = (java.util.function.Supplier<ShotParameters>) () ->
      m_shotMap.get(distance.getAsDouble());

    rpmSupplier = (java.util.function.DoubleSupplier) () -> shotSupplier.get().rpm();
    hoodSupplier = (java.util.function.DoubleSupplier) () -> shotSupplier.get().hoodPos();

    readySupplier = (java.util.function.BooleanSupplier) () ->
      //m_robotDrive.isAimedAt(hubSupplier.get()) &&
      m_shooterSubsystem.shooterAtVelocity()
      && m_hoodSubsystem.atTarget();
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