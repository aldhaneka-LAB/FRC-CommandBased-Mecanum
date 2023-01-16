// used for organising subsystems and commands

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CommandAuto;
import frc.robot.commands.TeleOpCmd;
// import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsytem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static Field2d m_Field2d = new Field2d();

  public static ExampleSubsystem p = new ExampleSubsystem();

  // The robot's subsystems and commands are defined here...
  private final DriveSubsytem m_robotDrive = new DriveSubsytem(m_Field2d);

  private CommandAuto m_auto = new CommandAuto(m_robotDrive);

  XboxController m_driverController = new XboxController(DriveConstants.XboxPort);
  // PS4Controller m_driverController = new PS4Controller(DriveConstants.XboxPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData("Field", m_Field2d);
    SmartDashboard.putData(m_robotDrive);
    // SmartDashboard.getData("Field");
// SendableBuilder
    System.out.println("Robot Container");
    System.out.println(m_driverController.getRightX());

    m_robotDrive.setDefaultCommand(
     new TeleOpCmd(
      m_robotDrive,
       () ->  -m_driverController.getLeftY(),
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getRightX()));


    p.setDefaultCommand(new RunCommand(() ->  {
      SmartDashboard.putNumber("X Right", m_driverController.getRightX());
      SmartDashboard.putNumber("Y Right", m_driverController.getRightY());

      SmartDashboard.putNumber("X Left", m_driverController.getLeftX());
      SmartDashboard.putNumber("Y Left", m_driverController.getLeftY());

    }, p));


      // ExampleMoto
  }
  



  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    System.out.println("Configure Button Bindings");
    // XboxController.B

    // m_driverController.getRightStickButtonPressed()
    // Drive at half speed when the right bumper is held
  
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> System.out.println("YESS")))
        .onFalse(new InstantCommand(() -> System.out.println("NOOO")));

          
    // new JoystickButton(m_driverController, PS4Controller.Button.kOptions.value)
    // .onTrue(new InstantCommand(() -> System.out.println("YESS")))
    // .onFalse(new InstantCommand(() -> System.out.println("NOOO")));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // m_autoCommand.wai
    // Command.InterruptionBehavior
    // Commands.run
    // Commands.waitSeconds(0)
    // Commands.sequence(null)
    // Commands.Pare
    // An ExampleCommand will run in autonomous

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);


    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            trajectoryConfig);
            // ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
            // ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
   
            // exampleTrajectory.sample(0)
    MecanumControllerCommand mecanumControllerCommand =
        new MecanumControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            DriveConstants.kFeedForward,
            DriveConstants.kDriveKinematics,

            // Position contollers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),

            // Needed for normalizing wheel speeds
            AutoConstants.kMaxSpeedMetersPerSecond,

            // Velocity PID's
            new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
            new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
            new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
            new PIDController(DriveConstants.kPRearRightVel, 0, 0),
            m_robotDrive::getCurrentWheelSpeeds,
            m_robotDrive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());


    // MecanumAutoBuilder
    return m_auto;
  }
}
