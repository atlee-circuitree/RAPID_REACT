// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

  CANSparkMax feederMotor = null;
  DoubleSolenoid feederLift = new DoubleSolenoid(15, PneumaticsModuleType.REVPH, Constants.kickoutPnumatic_Deploy, Constants.kickoutPnumatic_Retract);
   
  public FeederSubsystem() {

    feederMotor = new CANSparkMax(Constants.feederMotorPort, MotorType.kBrushless);

     
  }

  public void pushOutFeeder() {

    feederLift.set(Value.kForward);
   
  }

  public void pullInFeeder() {

    feederLift.set(Value.kReverse);
   
  }

  public void runFeeder(double speed) {

    feederMotor.set(speed);

  }

}
