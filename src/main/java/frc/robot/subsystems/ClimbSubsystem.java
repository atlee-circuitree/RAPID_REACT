// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

  CANSparkMax hookMotor = null;
  DoubleSolenoid climbLeft = new DoubleSolenoid(15, PneumaticsModuleType.REVPH, Constants.climbLeftPnumatic_Deploy, Constants.climbLeftPnumatic_Retract);
  DoubleSolenoid climbRight = new DoubleSolenoid(15, PneumaticsModuleType.REVPH, Constants.climbRightPnumatic_Deploy, Constants.climbRightPnumatic_Retract);
 
  public ClimbSubsystem() {

    hookMotor = new CANSparkMax(Constants.hookMotorPort, MotorType.kBrushless);

  }
  

  public void extendArm() {

    climbLeft.set(Value.kForward);
    climbRight.set(Value.kForward);

  }

  public void retractArm() {

    climbLeft.set(Value.kReverse);
    climbRight.set(Value.kReverse);

  }

  public void runHook(double speed) {

    hookMotor.set(speed);

  }
 
}
