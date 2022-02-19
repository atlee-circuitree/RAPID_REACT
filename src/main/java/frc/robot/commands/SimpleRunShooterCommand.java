// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class SimpleRunShooterCommand extends CommandBase {

  TurretSubsystem m_subsystem;
  double targetVelocity;
   
  
  public SimpleRunShooterCommand(double velocity, TurretSubsystem turretSubsystem) {

    m_subsystem = turretSubsystem;
    addRequirements(m_subsystem);
    targetVelocity = velocity;
     
  }

  
  @Override
  public void initialize() {}

  @Override
  public void execute() {

    m_subsystem.runTurretWithVelocity(targetVelocity);

  }
 
  @Override
  public void end(boolean interrupted) {

    m_subsystem.runTurretWithVelocity(0);

  }
 
  @Override
  public boolean isFinished() {
    return false;
  }
}
