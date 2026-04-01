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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.shooting.ShotMap;
import frc.robot.shooting.ShotMapData;
import frc.robot.shooting.ShotParameters;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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

          boolean manualSlowMode = m_driverController.leftBumper().getAsBoolean();
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
          double yawDeg = m_robotDrive.getGyroRotation().getDegrees();
          double yawRateDegPerSec = m_robotDrive.getTurnRate();
          double pitchDeg = m_robotDrive.getGyroPitch();
          double rollDeg = m_robotDrive.getGyroRoll();

          // Hold DRIVER X to aggressively relocalize right before scoring/passing
          boolean recoveryMode = 
            m_driverController.x().getAsBoolean() ||
            m_operatorController.leftTrigger().getAsBoolean();
          
          boolean bumpMode = 
            Math.abs(m_robotDrive.getGyroPitch()) > VisionConstants.kPitchToleranceDeg;

          SmartDashboard.putBoolean("Vision/RecoveryMode", recoveryMode);
          SmartDashboard.putBoolean("Vision/BumpMode", bumpMode);
          
          boolean preciseMode = recoveryMode || bumpMode;

          double maxYawRate =
            preciseMode
              ? Constants.VisionConstants.kRecoveryMaxVisionYawRateDegPerSec
              : Constants.VisionConstants.kMaxVisionYawRateDegPerSec;

          double maxPoseJump =
            preciseMode
              ? Constants.VisionConstants.kRecoveryMaxAcceptedPoseJumpMeters
              : Constants.VisionConstants.kMaxAcceptedPoseJumpMeters;

          // In recovery mode, use FRONT limelight only.
          // This is safer for tomorrow because scoring is front-facing and tag-rich.
          String[] llnames =
            preciseMode
              ? new String[] {m_visionSubsystem.getLimelightWithMostTags()}
              : new String[] {
                  Constants.VisionConstants.kFrontLimelightName,
                  Constants.VisionConstants.kBackLimelightName
                };

          if (Math.abs(yawRateDegPerSec) > maxYawRate) {
            return;
          }

          for (String llname : llnames) {
            m_visionSubsystem
              .getVisionMeasurementMT2(
                llname, 
                yawDeg, 
                yawRateDegPerSec, 
                pitchDeg,
                rollDeg,
                preciseMode)
              .ifPresent(m -> {
                double poseJump =
                  m.getPose().getTranslation().getDistance(
                    m_robotDrive.getPose().getTranslation()
                  );

                SmartDashboard.putNumber("Vision/" + llname + "/PoseJump", poseJump);

                if (poseJump > maxPoseJump) return;

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

    // Pathplanner Commands
    NamedCommands.registerCommand(
      "Intake-2.5", 
      m_intakeSubsystem.runIntakeCommand()
        .withTimeout(2.5)
    );

    NamedCommands.registerCommand(
      "Intake-5", 
      m_intakeSubsystem.runIntakeCommand()
        .withTimeout(5)
    );

    NamedCommands.registerCommand(
      "Hopper-Deploy", 
      m_shooterSubsystem.holdVelocityCommand(1000)
        .withTimeout(.25) 
    );

    NamedCommands.registerCommand("Hood-To-Start", m_hoodSubsystem.holdPositionCommand(.28));

    NamedCommands.registerCommand("Pivot-In", m_intakeSubsystem.pivotInCommand());
    NamedCommands.registerCommand("Pivot-Out", m_intakeSubsystem.pivotOutCommand());

    NamedCommands.registerCommand("Shoot", 
      new ParallelCommandGroup(
        m_shooterSubsystem.holdVelocityCommand(2920),
        m_feederSubsystem.feedWhen(() -> m_shooterSubsystem.shooterSafeToFeed()),
        new WaitCommand(1)
          .andThen(m_intakeSubsystem.pivotAgitateCommand(readySupplier))
      )
    ); // Test positions/velocity later

    // register auto options to the shuffleboard 
      autoChooser.addOption("R-SAFE", "R-SAFE");
      autoChooser.addOption("RB-C", "RB-C");
      autoChooser.addOption("Full-R", "Full-R");
      autoChooser.addOption("C-SAFE", "C-SAFE");
      autoChooser.addOption("HC-CL", "HC-CL");
      autoChooser.addOption("HC-CR", "HC-CR");
      autoChooser.addOption("Full-HR", "Full-HR");
      autoChooser.addOption("Full-HL", "Full-HL");
      autoChooser.addOption("L-SAFE", "L-SAFE");
      autoChooser.addOption("LB-C", "LB-C");
      autoChooser.addOption("Full-L", "Full-L");

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

    m_driverController.rightTrigger().whileTrue(m_intakeSubsystem.runIntakeCommand());
    
    //reverse everything
    m_driverController.rightBumper().whileTrue(
      m_intakeSubsystem.runIntakeReverseCommand()
        .alongWith(m_feederSubsystem.runFeederReverseCommand())
        .alongWith(m_shooterSubsystem.holdVelocityCommand(-2000))
    );

    m_driverController.leftTrigger()
      .onTrue(new InstantCommand(() -> m_robotDrive.enableTraversalMode()))
      .onFalse(new InstantCommand(() -> m_robotDrive.disableTraversalMode()));
    
    // Y Button -> Zero pivot on true
    m_driverController.y().whileTrue(m_intakeSubsystem.tempZeroPivotAtInCommand());
    
    // B Button -> Retract pivot on true
    m_driverController.b().onTrue(m_intakeSubsystem.pivotInCommand());
    
    // A Button -> Extend pivot on true
    m_driverController.a().onTrue(m_intakeSubsystem.pivotOutCommand());

    // Left on D-Pad -> Jog pivot up
    m_driverController.povLeft().whileTrue(m_intakeSubsystem.pivotJogCommand(0.35));
    
    // Right on D-Pad -> Jog pivot down
    m_driverController.povRight().whileTrue(m_intakeSubsystem.pivotJogCommand(-0.35));

/////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // Operator Controller

    // Start Button -> Zero swerve heading
    m_operatorController.start().onTrue(m_robotDrive.zeroHeadingCommand());

    passLeft = m_operatorController.povRight().getAsBoolean();
    passRight = m_operatorController.povLeft().getAsBoolean();
    
    var hubSupplier = (Supplier<Translation2d>) FieldConstants::getAllianceHub;

    var distanceSupplier = (DoubleSupplier) () ->
      m_robotDrive.getPose().getTranslation().getDistance(hubSupplier.get());

    var leftFieldTargetSupplier = 
      (Supplier<Translation2d>) FieldConstants::getLeftFieldTarget;

    var rightFieldTargetSupplier = 
      (Supplier<Translation2d>) FieldConstants::getRightFieldTarget;

    if (passLeft) {
      distanceSupplier = (DoubleSupplier) () ->
        m_robotDrive.getPose().getTranslation().getDistance(leftFieldTargetSupplier.get());

    } else if (passRight) {
      distanceSupplier = (DoubleSupplier) () ->
        m_robotDrive.getPose().getTranslation().getDistance(rightFieldTargetSupplier.get());

    }

    this.setShotSuppliers(distanceSupplier);

    // Operator LT -> "Ready to Shoot" (aim + set shooter rpm + set hood)
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

    m_operatorController.povUp()
      .whileTrue(
        m_shooterSubsystem.holdVelocityCommand(6750)
      );

    m_operatorController.povDown()
      .whileTrue(
        m_shooterSubsystem.holdVelocityCommand(-6750)
      );

    // Operator RT = Fire (feed only when ready)
    m_operatorController.rightTrigger()
      .whileTrue(
        m_feederSubsystem.feedWhen(readySupplier)
          .alongWith(
            new WaitCommand(.25)
              .andThen(m_intakeSubsystem.pivotAgitateCommand(readySupplier))
          )
      );

    // Operator X -> Manual fallback shot prep (fixed RPM + fixed hood)
    m_operatorController.x()
      .whileTrue(
        m_shooterSubsystem.holdVelocityCommand(2770)
          .alongWith(m_hoodSubsystem.holdPositionCommand(0.23))
          .alongWith(m_robotDrive.setXCommand())
      );

    m_operatorController.a()
      .whileTrue(
        m_shooterSubsystem.holdVelocityCommand(4000)
          .alongWith(m_hoodSubsystem.holdPositionCommand(.68))
        );

    // Operator Y -> Manual fallback fire (feed + pivot agitate)
    m_operatorController.y()
      .whileTrue(
        m_feederSubsystem.runFeederCommand()
          .alongWith(m_intakeSubsystem.pivotAgitateCommand(() -> true))
      );

    m_operatorController.b()
      .whileTrue(
        m_hoodSubsystem.holdPositionCommand(.2)
      );

    m_operatorController.leftBumper()
      .whileTrue(
        m_robotDrive.resetOdometryCommand(
          FieldConstants.getTestPose()
        )
      );

  }

  public void setShotSuppliers(DoubleSupplier distance) {
    var shotSupplier = (java.util.function.Supplier<ShotParameters>) () ->
    m_shotMap.get(distance.getAsDouble());

    SmartDashboard.putNumber("rpmSupplier", shotSupplier.get().rpm());
    SmartDashboard.putNumber("hoodSupplier", shotSupplier.get().hoodPos());

    rpmSupplier = (java.util.function.DoubleSupplier) () -> shotSupplier.get().rpm();
    hoodSupplier = (java.util.function.DoubleSupplier) () -> shotSupplier.get().hoodPos();

    readySupplier = (java.util.function.BooleanSupplier) () ->
      m_shooterSubsystem.shooterSafeToFeed()
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