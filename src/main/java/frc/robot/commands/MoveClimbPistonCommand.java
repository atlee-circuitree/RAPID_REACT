// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSubsystem;
 
public class MoveClimbPistonCommand extends InstantCommand {

  boolean isUp;
  ClimbSubsystem m_subsystem;

  public MoveClimbPistonCommand(boolean goingUp, ClimbSubsystem climbSubsystem) {

    m_subsystem = climbSubsystem;
    addRequirements(m_subsystem);
    isUp = goingUp;

  }
 
  @Override
  public void initialize() {

  if (isUp == true) {

    m_subsystem.extendArm();

  } else {

    m_subsystem.retractArm();

  }


  }
  
}
