// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretPistonCommand extends CommandBase {

  TurretSubsystem m_subsystem;
  
  public TurretPistonCommand(TurretSubsystem turretSubsystem) {

    m_subsystem = turretSubsystem;
    addRequirements(m_subsystem);
     
  }

  
  @Override
  public void initialize() {}

  @Override
  public void execute() {

  m_subsystem.extend(); 

  }
 
  @Override
  public void end(boolean interrupted) {

  m_subsystem.retract();

  }
 
  @Override
  public boolean isFinished() {
    return false;
  }
}
