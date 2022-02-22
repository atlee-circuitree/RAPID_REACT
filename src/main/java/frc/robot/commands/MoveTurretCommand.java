// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class MoveTurretCommand extends CommandBase {

  TurretSubsystem m_subsystem;
  double targetSpeed;
   
  
  public MoveTurretCommand(double speed, TurretSubsystem turretSubsystem) {

    m_subsystem = turretSubsystem;
    addRequirements(m_subsystem);
    targetSpeed = speed;
     
  }

  
  @Override
  public void initialize() {}

  @Override
  public void execute() {

    m_subsystem.turnTurret(targetSpeed);

  }
 
  @Override
  public void end(boolean interrupted) {

    m_subsystem.turnTurret(0);

  }
 
  @Override
  public boolean isFinished() {
    return false;
  }
}