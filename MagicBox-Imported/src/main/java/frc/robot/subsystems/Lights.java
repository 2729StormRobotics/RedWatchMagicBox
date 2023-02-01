// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
// https://www.youtube.com/watch?v=wMdkM2rr1a4

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static frc.robot.Constants.LightConstants.*;

public class Lights extends SubsystemBase {
 
  private static final double kDisabled = 0;
  private final Spark m_ledDriver;
  private final ShuffleboardTab m_ShuffleboardTab;
  private final ShuffleboardLayout m_lightValues;



  /** Creates a new Lights. */
  public Lights() {
    m_ShuffleboardTab = Shuffleboard.getTab(kShuffleboardTab);
    m_lightValues = m_ShuffleboardTab.getLayout("Light Jawndess", BuiltInLayouts.kList);

    m_ledDriver = new Spark(kBlinkinDriverPort);
    resetLights();

    m_lightValues.addNumber("Light Output", () -> getCurrentLights());
  }

  public void setDisabledColor() {
    m_ledDriver.set(kDisabled);
  }

  public void setOff() {
    m_ledDriver.set(kLightsOff);
  }

  public void intakeRed() {
    m_ledDriver.set(kRedBall);
  }

  public void intakeBlue() {
    resetLights();
    m_ledDriver.set(kBlueBall);
  }

  public void resetLights() {
    m_ledDriver.set(kDefaultColor);
  }
  public void setCube() {
    resetLights();
    m_ledDriver.set(kPurpleCube);
  }
  public void setCone() {
    resetLights();
    m_ledDriver.set(kYellowCone);
  }

  public double getCurrentLights() {
    return m_ledDriver.get();
  }

  public void setGiven(double color) {
    m_ledDriver.set(color);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}