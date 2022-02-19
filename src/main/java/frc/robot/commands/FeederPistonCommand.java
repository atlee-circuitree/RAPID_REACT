// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.FeederSubsystem;
 
public class FeederPistonCommand extends InstantCommand {

  boolean isUp;
  FeederSubsystem m_subsystem;

  public FeederPistonCommand(boolean goingUp, FeederSubsystem feedSubsystem) {

    m_subsystem = feedSubsystem;
    addRequirements(m_subsystem);
    isUp = goingUp;

  }
 
  @Override
  public void initialize() {

  if (isUp == true) {

    m_subsystem.pullInFeeder();

  } else {

    m_subsystem.pushOutFeeder();

  }


  }
  
}
