// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.commands.DriveWithXbox;
import frc.robot.commands.MoveHookCommand;
import frc.robot.commands.RecalibrateModules;
import frc.robot.commands.RunFeederCommand;
import frc.robot.commands.SimpleRunShooter;
import frc.robot.commands.SmartDashboardCommand;
import frc.robot.commands.TestDriveCommand;
import frc.robot.commands.TestRotateModules;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FeederSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
 
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  //Controllers
  public static XboxController Xbox1 = new XboxController(0);
  public static XboxController Xbox2 = new XboxController(1);
  public Joystick Fightstick = new Joystick(2);
  
  //Subsystems
  private final Drivetrain drivetrain;
  private final LimeLightSubsystem limelight;
  private final ClimbSubsystem climb;
  private final FeederSubsystem feed;
  private final TurretSubsystem turret;
  
  //Commands
  private final DriveWithXbox driveWithXbox;
  private final TestRotateModules testRotateModules;
  private final TestDriveCommand testDriveCommand;
  private final SmartDashboardCommand smartDashboardCommand;
  private final PerpetualCommand DWXwithSDC;
  private final RecalibrateModules recalibrateModules;
  private final MoveHookCommand MoveHook;
  private final RunFeederCommand RunFeeder;
  private final RunFeederCommand StopFeeder;

  //Buttons
  JoystickButton DriverA = new JoystickButton(Xbox1, XboxController.Button.kA.value);
  JoystickButton DriverB = new JoystickButton(Xbox1, XboxController.Button.kB.value);

  JoystickButton FightR3 = new JoystickButton(Fightstick, 10);
  JoystickButton FightOption = new JoystickButton(Fightstick, 8);

  public RobotContainer() {
 
    //Subsystems
    drivetrain = new Drivetrain();
    limelight = new LimeLightSubsystem();
    climb = new ClimbSubsystem();
    feed = new FeederSubsystem();
    turret = new TurretSubsystem();

    //Single Commands (One Use)
    driveWithXbox = new DriveWithXbox(drivetrain);
    driveWithXbox.addRequirements(drivetrain);
    smartDashboardCommand = new SmartDashboardCommand(limelight);
    DWXwithSDC = new PerpetualCommand(driveWithXbox.alongWith(smartDashboardCommand));
    testDriveCommand = new TestDriveCommand(drivetrain);
    MoveHook = new MoveHookCommand(Fightstick, climb);
    RunFeeder = new RunFeederCommand(.5, feed);
    StopFeeder = new RunFeederCommand(0, feed);
    //testDriveCommand.addRequirements(drivetrain);
    //drivetrain.setDefaultCommand(testDriveCommand);

    //Auto Setup
    testRotateModules = new TestRotateModules(drivetrain);

    //Other Setup
    configureButtonBindings();
    recalibrateModules = new RecalibrateModules(drivetrain, Xbox1);
    drivetrain.setDefaultCommand(DWXwithSDC);

  }

  //Mult Commands (Many Uses)
  public Command simpleTurretCommand(double velocity) {
    Command m_turretCommand = new SimpleRunShooter(velocity, turret);
    return m_turretCommand;
  }

  

  private void configureButtonBindings() {
    
    DriverA.whileHeld(RunFeeder);
    DriverB.whileHeld(simpleTurretCommand(4800));
    
  }

  public Command getAutonomousCommand() {
    // Create config for trajectory
    
    TrajectoryConfig config =
        new TrajectoryConfig(
                8.0, 8.0)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.driveKinematics);

    //THIS IS WHERE YOU DECLARE POINTS
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these interior waypoints
            List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
            // End straight ahead of where we started, facing forward
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
