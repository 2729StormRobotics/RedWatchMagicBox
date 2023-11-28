// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class motor extends SubsystemBase {
  /** Creates a new motor. */
  public final AnalogInput input = new AnalogInput(0);
  
  public final com.revrobotics.CANSparkMax m_motor;
  public final RelativeEncoder m_encoder;
  public final AnalogPotentiometer pot = new AnalogPotentiometer(0, 180, 30);

  // Constructor
  public motor () {
    m_motor = new com.revrobotics.CANSparkMax(Constants.kMotorPort, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(45);

    m_encoder = m_motor.getEncoder();
    m_encoder.setPosition(0);

  }

  public void setSpeed(double speed) {
    m_motor.set(speed);
  }

  public void stopMotor() {
    m_motor.set(0);
  }
  //hehehehaw
  public double getSpeed() {
    return m_encoder.getVelocity();
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  public void resetPosition() {
    m_encoder.setPosition(0);
  }

  public double getPotValue() {
    return pot.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } 
}
