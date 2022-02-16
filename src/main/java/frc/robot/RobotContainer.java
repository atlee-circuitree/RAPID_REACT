// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
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
import frc.robot.commands.MoveClimbPistonCommand;
import frc.robot.commands.MoveHookCommand;
import frc.robot.commands.RecalibrateModules;
import frc.robot.commands.SmartDashboardCommand;
import frc.robot.commands.TestDriveCommand;
import frc.robot.commands.TestRotateModules;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.LimeLightSubsystem;
 
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  //Controllers
  public static XboxController Xbox1;
  public static XboxController Xbox2;
  public static Joystick Fightstick = new Joystick(2);
  
  //Subsystems
  private final Drivetrain drivetrain;
  private final LimeLightSubsystem limelight;
  private final ClimbSubsystem climb;
  
  //Commands
  private final DriveWithXbox driveWithXbox;
  private final TestRotateModules testRotateModules;
  private final TestDriveCommand testDriveCommand;
  private final SmartDashboardCommand smartDashboardCommand;
  private final PerpetualCommand DWXwithSDC;
  private final RecalibrateModules recalibrateModules;
  private final MoveHookCommand MoveHook;
 

  public RobotContainer() {
 
    //Subsystems
    drivetrain = new Drivetrain();
    limelight = new LimeLightSubsystem();
    climb = new ClimbSubsystem();

    //Single Commands (One Use)
    driveWithXbox = new DriveWithXbox(drivetrain);
    driveWithXbox.addRequirements(drivetrain);
    smartDashboardCommand = new SmartDashboardCommand(limelight);
    DWXwithSDC = new PerpetualCommand(driveWithXbox.alongWith(smartDashboardCommand));
    testDriveCommand = new TestDriveCommand(drivetrain);
    MoveHook = new MoveHookCommand(Fightstick, climb);
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
  public Command climbCommand(boolean goingUp) {
    Command m_climbCommand = new MoveClimbPistonCommand(goingUp, climb);
    return m_climbCommand;
  }

  private void configureButtonBindings() {

    //Buttons
    JoystickButton Xbox1_A = new JoystickButton(Xbox1, XboxController.Button.kA.value);
    JoystickButton Xbox1_B = new JoystickButton(Xbox1, XboxController.Button.kB.value);
    JoystickButton Xbox1_X = new JoystickButton(Xbox1, XboxController.Button.kX.value);
    JoystickButton Xbox1_Y = new JoystickButton(Xbox1, XboxController.Button.kY.value);
    JoystickButton Xbox1_LBumper = new JoystickButton(Xbox1, XboxController.Button.kLeftBumper.value);
    JoystickButton Xbox1_RBumper = new JoystickButton(Xbox1, XboxController.Button.kRightBumper.value);
    JoystickButton Xbox1_Start = new JoystickButton(Xbox1, XboxController.Button.kStart.value);
    JoystickButton Xbox1_Back = new JoystickButton(Xbox1, XboxController.Button.kBack.value);

    JoystickButton Xbox2_A = new JoystickButton(Xbox2, XboxController.Button.kA.value);
    JoystickButton Xbox2_B = new JoystickButton(Xbox2, XboxController.Button.kB.value);
    JoystickButton Xbox2_X = new JoystickButton(Xbox2, XboxController.Button.kX.value);
    JoystickButton Xbox2_Y = new JoystickButton(Xbox2, XboxController.Button.kY.value);
    JoystickButton Xbox2_LBumper = new JoystickButton(Xbox2, XboxController.Button.kLeftBumper.value);
    JoystickButton Xbox2_RBumper = new JoystickButton(Xbox2, XboxController.Button.kRightBumper.value);
    JoystickButton Xbox2_Start = new JoystickButton(Xbox2, XboxController.Button.kStart.value);
    JoystickButton Xbox2_Back = new JoystickButton(Xbox2, XboxController.Button.kBack.value);

    JoystickButton Fight_1 = new JoystickButton(Fightstick, 1);
    JoystickButton Fight_2 = new JoystickButton(Fightstick, 2);
    JoystickButton Fight_3 = new JoystickButton(Fightstick, 1);
    JoystickButton Fight_4 = new JoystickButton(Fightstick, 2);
    JoystickButton Fight_5 = new JoystickButton(Fightstick, 1);
    JoystickButton Fight_6 = new JoystickButton(Fightstick, 2);

    //Button Assignment
    Fight_1.whenPressed(climbCommand(true));
    Fight_2.whenPressed(climbCommand(false));

    //Stick Assignment

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
