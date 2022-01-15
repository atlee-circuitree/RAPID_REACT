// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.commands.DriveWithXbox;
import frc.robot.commands.RecalibrateModules;
import frc.robot.commands.SmartDashboardCommand;
import frc.robot.commands.TestDriveCommand;
import frc.robot.commands.TestRotateModules;
import frc.robot.commands.runTestMotor;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.TestSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  //Controllers
  public static XboxController xbox;
  
  //Subsystems
  private final Drivetrain drivetrain;
  private final LimeLightSubsystem limelight;
  private final TestSubsystem test;
 
  //Commands
  private final DriveWithXbox driveWithXbox;
  private final TestRotateModules testRotateModules;
  private final TestDriveCommand testDriveCommand;
  private final SmartDashboardCommand smartDashboardCommand;
  private final PerpetualCommand DWXwithSDC;
  private final RecalibrateModules recalibrateModules;
  private final runTestMotor runTestMotor;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    test = new TestSubsystem();
    drivetrain = new Drivetrain();
    limelight = new LimeLightSubsystem();

    

    driveWithXbox = new DriveWithXbox(drivetrain);
    driveWithXbox.addRequirements(drivetrain);

    smartDashboardCommand = new SmartDashboardCommand(limelight);
    DWXwithSDC = new PerpetualCommand(driveWithXbox.alongWith(smartDashboardCommand));
    runTestMotor = new runTestMotor(.3, test);

    configureButtonBindings();

    testDriveCommand = new TestDriveCommand(drivetrain);
    //testDriveCommand.addRequirements(drivetrain);
    //drivetrain.setDefaultCommand(testDriveCommand);

    //Auto Setup
    testRotateModules = new TestRotateModules(drivetrain);

    

    //Other Setup

    recalibrateModules = new RecalibrateModules(drivetrain, xbox);

    drivetrain.setDefaultCommand(DWXwithSDC);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    xbox = new XboxController(0);

    JoystickButton DriverA = new JoystickButton(xbox, XboxController.Button.kA.value);

    DriverA.whenHeld(runTestMotor);

  }

  public Command getAutonomousCommand() {
    // Create config for trajectory
    
    TrajectoryConfig config =
        new TrajectoryConfig(
                4.0, 4.0)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.driveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            1.0, 0, 0, Constants.thetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            drivetrain::getPose, // Functional interface to feed supplier
            Constants.driveKinematics,

            // Position controllers
            new PIDController(1.0, 0, 0),
            new PIDController(1.0, 0, 0),
            thetaController,
            drivetrain::setSwerveModuleStates,
            drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivetrain.driveAllModulesNonLinear(0));
    
    //return testRotateModules; 
  }
}
