// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.BottomShooterMotorLeft;
import frc.robot.subsystems.BottomShooterMotorRight;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.TopShooterMotorLeft;
import frc.robot.subsystems.TopShooterMotorRight;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
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
  private final BottomShooterMotorLeft m_bottomShooterMotorLeft = new BottomShooterMotorLeft();
  private final TopShooterMotorRight m_topShooterMotorRight = new TopShooterMotorRight();
  private final BottomShooterMotorRight m_bottomShooterMotorRight = new BottomShooterMotorRight();
  private final TopShooterMotorLeft m_topShooterMotorLeft = new TopShooterMotorLeft();
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
        private LEDs m_leds = LEDs.getInstance();

  //private double MaxSpeed = 6; // 6 meters per second desired top speed
  //private double MaxAngularRate = ((2) * Math.PI); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  // private final CommandXboxController joystick = new CommandXboxController(0);
  // // My joystick
  //private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  //private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //   .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //   .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
    //                                                            // driving in open loop
//   private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
//   private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
//   private final SwerveRequest.FieldCentricFacingAngle faceAngle = new SwerveRequest.FieldCentricFacingAngle();
//   private final Telemetry logger = new Telemetry(MaxSpeed);
private SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_bottomShooterMotorLeft.setDefaultCommand(m_bottomShooterMotorLeft.stop());
    m_topShooterMotorLeft.setDefaultCommand(m_topShooterMotorLeft.stop());
    m_topShooterMotorRight.setDefaultCommand(m_topShooterMotorRight.stop());
    m_bottomShooterMotorRight.setDefaultCommand(m_bottomShooterMotorRight.stop());
    m_amp.setDefaultCommand(m_amp.stop());
    m_climber.setDefaultCommand(m_climber.stop());
    NamedCommands.registerCommand("Shoot",
        Commands.parallel(
                m_topShooterMotorLeft.spin(),
                m_topShooterMotorRight.spin()
        ).withTimeout(2)
        .andThen(Commands.parallel(
                 m_bottomShooterMotorLeft.spin(),
                m_bottomShooterMotorRight.spin())
          .withTimeout(2)
        )
        .andThen(Commands.parallel(
                m_topShooterMotorLeft.stop(),
                m_topShooterMotorRight.stop(),
                m_bottomShooterMotorLeft.stop(),
                m_bottomShooterMotorRight.stop()).withTimeout(.2)
                ));
                
    
    
    // m_topShooterMotorLeft.spin()
    //         .alongWith(m_topShooterMotorRight.spin())
    //         .alongWith(Commands.waitSeconds(0.5).andThen(m_bottomShooterMotorLeft.spin()))
    //         .alongWith(Commands.waitSeconds(0.5).andThen(m_bottomShooterMotorRight.spin()));


    // Configure the trigger bindings, 
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    faceAngle.HeadingController = new PhoenixPIDController(1, 0, 0);
    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
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

    // Top Shooter Motor and Bottom Shooter Motor intaking a note (one)
    m_driverController.leftTrigger()
        .whileTrue(m_topShooterMotorLeft.intake()
            .alongWith(m_topShooterMotorRight.intake())
            .alongWith(m_bottomShooterMotorLeft.intake())
            .alongWith(m_bottomShooterMotorRight.intake()));

    // Top Shooter Motor and Bottom Shooter Motor shooting a note
    m_driverController.rightTrigger()
        .whileTrue(m_topShooterMotorLeft.spin()
            .alongWith(m_topShooterMotorRight.spin())
            .alongWith(Commands.waitSeconds(0.5).andThen(m_bottomShooterMotorLeft.spin()))
            .alongWith(Commands.waitSeconds(0.5).andThen(m_bottomShooterMotorRight.spin())));

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * maxSpeed) // Drive
                                                                                                          // forward
                                                                                                          // with
            // negative Y (forward)
            .withVelocityY(-m_driverController.getLeftX() * maxSpeed) // Drive left with negative X (left)
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
                    drivetrain.getCurrentAlliance().equals(Alliance.Red) ? Rotation2d.fromDegrees(60)
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
    m_driverController.rightBumper()
    .whileTrue(m_topShooterMotorLeft.ampSpin()
        .alongWith(m_topShooterMotorRight.ampSpin())
        .alongWith(Commands.waitSeconds(0.5).andThen(m_bottomShooterMotorLeft.ampSpin()))
        .alongWith(Commands.waitSeconds(0.5).andThen(m_bottomShooterMotorRight.ampSpin())));

    // Here are the climber commands
    // Here are the climber commands
    m_driverController.x().whileTrue(m_climber.extend());
    m_driverController.y().whileTrue(m_climber.retract());

    // reset the field-centric heading on left bumper press
    m_driverController.b().onTrue(drivetrain.runOnce(() ->
    drivetrain.seedFieldRelative()));

//     new Trigger(DriverStation.isDisabled()).whileTrue(getAutonomousCommand())
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
