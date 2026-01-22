/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

public class Robot extends TimedRobot {
  private DigitalInput beamBreak;
  private double voltsToMeters = 0;

  public Robot() {
    /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    beamBreak = new DigitalInput(0);

    // Initialize dashboard values
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putBoolean("BeamBreak", beamBreak.get());
  }

  @Override
  public void robotPeriodic() {
    // Display encoder position and velocity
    SmartDashboard.putBoolean("BeamBreak", beamBreak.get());
  }
}