// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  static TalonSRX topShootMotor = null;
  static TalonSRX bottomShootMotor = null;
  CANSparkMax turretMotor = null;
  //DoubleSolenoid shootPiston = null;
  public TurretSubsystem() {

    topShootMotor = new TalonSRX(Constants.topShootMotorPort);
    bottomShootMotor = new TalonSRX(Constants.bottomShootMotorPort);
    turretMotor = new CANSparkMax(Constants.turretMotorPort, MotorType.kBrushed);
    //shootPiston = new DoubleSolenoid(null, Constants.shootPnumatic_Deploy, Constants.shootPnumatic_Retract);
    
  }

  public void extend() {

    //shootPiston.set(Value.kForward);
   
  }

  public void retract() {

    //shootPiston.set(Value.kReverse);
   
  }

  public void runTurretWithVelocity(double velocity) {

    topShootMotor.set(ControlMode.Velocity, velocity);
    bottomShootMotor.set(ControlMode.Velocity, velocity);

  }

  public void turnTurret(double speed) {

    turretMotor.set(speed);

  }

  public static double getVelocity() {

    return (topShootMotor.getSelectedSensorVelocity() + bottomShootMotor.getSelectedSensorVelocity()) / 2;

  }

}
