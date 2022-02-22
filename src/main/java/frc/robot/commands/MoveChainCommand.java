// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;


public class MoveChainCommand extends CommandBase {

  ClimbSubsystem m_subsystem;
  double targetSpeed;
   
  public MoveChainCommand(double speed, ClimbSubsystem climb) {
 
    m_subsystem = climb;
    addRequirements(m_subsystem);
    targetSpeed = speed;
     
     
  }

  
  @Override
  public void initialize() {}

  @Override
  public void execute() {

    m_subsystem.runHook(targetSpeed);

  }
 
  @Override
  public void end(boolean interrupted) {

    m_subsystem.runHook(0);

  }
 
  @Override
  public boolean isFinished() {
    return false;
  }
}
