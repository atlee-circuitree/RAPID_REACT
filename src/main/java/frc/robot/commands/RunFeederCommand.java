// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.FeederSubsystem;

public class RunFeederCommand extends CommandBase {

  FeederSubsystem m_subsystem;
  double targetSpeed;
   
  
  public RunFeederCommand(double speed, FeederSubsystem feederSubsystem) {

    m_subsystem = feederSubsystem;
    addRequirements(m_subsystem);
    targetSpeed = speed;
     
  }

  
  @Override
  public void initialize() {}

  @Override
  public void execute() {

    m_subsystem.runFeeder(targetSpeed);

  }
 
  @Override
  public void end(boolean interrupted) {

    m_subsystem.runFeeder(0);

  }
 
  @Override
  public boolean isFinished() {
    return false;
  }
}
