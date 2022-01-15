// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem extends SubsystemBase {

  CANSparkMax testMotor;
 
  public TestSubsystem() {

    testMotor = new CANSparkMax(Constants.testMotor, MotorType.kBrushed);

  }

  @Override
  public void periodic() {
 
  }

  public void runAtSpeed(double speed) {

    testMotor.set(speed);

  }

}
