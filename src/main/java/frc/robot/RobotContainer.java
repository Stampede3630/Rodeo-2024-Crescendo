// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;

// top motor: 3
// bottom motor: 2
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final BottomShooterMotor m_bottomShooterMotor = new BottomShooterMotor();
  private final TopShooterMotor m_topShooterMotor = new TopShooterMotor();
  private final Amp m_amp = new Amp();
  private final Climber m_climber = new Climber();

  private final double maxSpeed = 6; // 6 meters per second desired top speed
  private final double maxAngularRate = ((2) * Math.PI); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  // private final CommandXboxController joystick = new CommandXboxController(0);
  // // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.FieldCentricFacingAngle faceAngle = new SwerveRequest.FieldCentricFacingAngle();
  private final Telemetry logger = new Telemetry(maxSpeed);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_bottomShooterMotor.setDefaultCommand(m_bottomShooterMotor.stop());
    m_topShooterMotor.setDefaultCommand(m_topShooterMotor.stop());
    m_amp.setDefaultCommand(m_amp.stop());
    m_climber.setDefaultCommand(m_climber.stop());
    NamedCommands.registerCommand("Shoot",m_topShooterMotor.spin()
            .alongWith(Commands.waitSeconds(0.5).andThen(m_bottomShooterMotor.spin())));

    // Configure the trigger bindings, 
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when
    // `exampleConditiTrigger(m_exampleSubsystem::exampleCondition)
    // Schedule `ExampleCommand` when
    // `exampleConditiTrigger(m_exampleSubsystem::exampleCondition)

    // Top Shooter Motor intaking a note (one)
    m_driverController.leftTrigger()
        .whileTrue(m_topShooterMotor.intake()
            .alongWith(m_bottomShooterMotor.intake()));

    // Top Shooter Motor shooting a note
    m_driverController.rightTrigger()
        .whileTrue(m_topShooterMotor.spin()
            .alongWith(Commands.waitSeconds(0.5).andThen(m_bottomShooterMotor.spin())));

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * maxSpeed * .8) // Drive
                                                                                                          // forward
                                                                                                          // with
            // negative Y (forward)
                            .withVelocityY(-m_driverController.getLeftX() * maxSpeed * .6) // Drive left with negative X (left)
                            .withRotationalRate(-m_driverController.getRightX() * maxAngularRate * 1) // Drive counterclockwise with
                                                                                      // negative X (left)
        ));

    m_driverController.rightStick().whileTrue( // face the source
            drivetrain.applyRequest(() -> faceAngle.withVelocityX(-m_driverController.getLeftY() * maxSpeed * .5) // Drive
            // forward
            // with
            // negative Y (forward)
                    .withVelocityY(-m_driverController.getLeftX() * maxSpeed * .5) // Drive left with negative X (left)
            .withTargetDirection(
                DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Red) ? Rotation2d.fromDegrees(60)
                    : Rotation2d.fromDegrees(300)) // Drive counterclockwise with
        ) // negative X (left)))
    );

    m_driverController.leftStick().whileTrue( // line up the shot
            drivetrain.applyRequest(() -> faceAngle.withVelocityX(-m_driverController.getLeftY() * maxSpeed * .5) // Drive
            // forward
            // with
            // negative Y (forward)
                    .withVelocityY(-m_driverController.getLeftX() * maxSpeed * .5) // Drive left with negative X (left)
            .withTargetDirection(Rotation2d.fromDegrees(0)) // Drive counterclockwise with
        ) // negative X (left)))
    );

    // Here are the amp commands
    m_driverController.a().whileTrue(m_amp.getNote());
    m_driverController.b().whileTrue(m_amp.scoreNote());
    // Here are the climber commands
    // Here are the climber commands
    m_driverController.y().whileTrue(m_climber.extend());
    m_driverController.x().whileTrue(m_climber.retract());

    // reset the field-centric heading on left bumper press
    // m_driverController.leftBumper().onTrue(drivetrain.runOnce(() ->
    // drivetrain.seedFieldRelative()));



    // align to speaker
    m_driverController.leftBumper().whileTrue(
            drivetrain.applyRequest(() -> faceAngle.withVelocityX(-m_driverController.getLeftY() * maxSpeed * .5) // Drive
                            // forward
                            // with
                            // negative Y (forward)
                            .withVelocityY(-m_driverController.getLeftX() * maxSpeed * .5) // Drive left with negative X (left)
                            .withTargetDirection(Rotation2d.fromDegrees(60)) // Drive counterclockwise with
                    // negative X (left)
            ));

    // align to source
    m_driverController.rightBumper().whileTrue(
            drivetrain.applyRequest(() -> faceAngle.withVelocityX(-m_driverController.getLeftY() * maxSpeed * .5) // Drive
                            // forward
                            // with
                            // negative Y (forward)
                            .withVelocityY(-m_driverController.getLeftX() * maxSpeed * .5) // Drive left with negative X (left)
                            .withTargetDirection(DriverStation.getAlliance().orElse(DriverStation.Alliance.Red).equals(DriverStation.Alliance.Red)
                                    ? Rotation2d.fromDegrees(300) : Rotation2d.fromDegrees(240)) // Drive counterclockwise with
                    // negative X (left)
            ));


    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
