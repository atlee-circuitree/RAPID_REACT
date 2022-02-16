// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretWithLimelightCommand extends CommandBase {

  TurretSubsystem m_subsystem;
  LimeLightSubsystem m_subsystem2;
  XboxController Xbox;
  
  public TurretWithLimelightCommand(XboxController xbox, TurretSubsystem turretSubsystem, LimeLightSubsystem limelightSubsystem) {

    Xbox = xbox;
    m_subsystem = turretSubsystem;
    m_subsystem2 = limelightSubsystem;
    addRequirements(m_subsystem, m_subsystem2);
     
  }

  
  @Override
  public void initialize() {}

  @Override
  public void execute() {

  if (Xbox.getLeftX() < .1 && Xbox.getLeftX() > -.1) {

    if (m_subsystem2.HorizontalOffset() > 10) {

      m_subsystem.turnTurretWithLimit(.2, 30);
  
    } else  if (m_subsystem2.HorizontalOffset() < -10) {
  
        m_subsystem.turnTurretWithLimit(-.2, 30);
    
    } else {

       m_subsystem.turnTurretWithLimit(0, 30);

    }
  
  } else {

    m_subsystem.turnTurretWithLimit(Xbox.getLeftX() * .5, 30);

  }

/*
    if (m_subsystem2.HorizontalOffset() > 10) {

      m_subsystem.runTurretWithLimit(.2, 150);

    }

    if (m_subsystem2.HorizontalOffset() < -10) {

      m_subsystem.runTurretWithLimit(-.2, 150);

    }
*/
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