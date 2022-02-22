// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends PIDSubsystem {

  // Giant Chunk of Copy-Paste Code
  // https://cdn.shopify.com/s/files/1/1518/8108/files/Armabot_A0085_Turret240_Encoder_Board_RevB.PDF
  public class AS5600EncoderPwm {
    private final SensorCollection sensors;
    private volatile int lastValue = Integer.MIN_VALUE;
    public AS5600EncoderPwm(SensorCollection sensors) {
    this.sensors = sensors;
    }
    public int getPwmPosition() {
    int raw = sensors.getPulseWidthRiseToFallUs();
    if (raw == 0) {
    int lastValue = this.lastValue;
    if (lastValue == Integer.MIN_VALUE) {
    return 0;
    }
    return lastValue;
    }
    int actualValue = Math.min(4096, raw - 128);
    lastValue = actualValue;
    return actualValue;
    }
   }

  private final WPI_TalonSRX yourTalon = new WPI_TalonSRX(1);
  private final AS5600EncoderPwm encoder = new
  AS5600EncoderPwm(yourTalon.getSensorCollection());
   
  static TalonSRX topShootMotor = null;
  static TalonSRX bottomShootMotor = null;
  CANSparkMax turretMotor = null;
  DoubleSolenoid shootPiston = null;

  public static String turretDashboard;

  public TurretSubsystem() {
 
    super(new PIDController(0, 0, 0));
    topShootMotor = new TalonSRX(Constants.topShootMotorPort);
    bottomShootMotor = new TalonSRX(Constants.bottomShootMotorPort);
    turretMotor = new CANSparkMax(Constants.turretMotorPort, MotorType.kBrushed);
    shootPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.shootPnumatic_Deploy, Constants.shootPnumatic_Retract);
  }
  @Override
  public void periodic() {
  
    turretDashboard = "Encoder PWM Position/" + encoder.getPwmPosition() + ";";
    turretDashboard = turretDashboard + "Encoder lastValue" + encoder.lastValue + ";";
    //turretDashboard = turretDashboard + "Encoder SparkMax" + turretMotor.getEncoder(Type.kQuadrature, 360);
  
  }

  protected void useOutput(double output, double setpoint) {
    turretMotor.set(output);
  }

  public double getMeasurement() {
    return encoder.getPwmPosition();
  }

  public double getLastEncoder() {
    return encoder.lastValue;
  }

  public void extend() {

    shootPiston.set(Value.kForward);
   
  }

  public void retract() {

    //shootPiston.set(Value.kReverse);
   
  }

  public void runTurretWithVelocity(double velocity) {

    topShootMotor.set(ControlMode.Velocity, -velocity);
    bottomShootMotor.set(ControlMode.Velocity, velocity);

  }

  public void turnTurret(double speed) {

    turretMotor.set(speed);

  }

  public static double getVelocity() {

    return (topShootMotor.getSelectedSensorVelocity() + bottomShootMotor.getSelectedSensorVelocity()) / 2;

  }

}
