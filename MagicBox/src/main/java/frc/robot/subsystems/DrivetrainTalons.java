// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DrivetrainTalons extends SubsystemBase {
  /** Creates a new Drivetrain. */

  // Create Talons
  private final WPI_TalonSRX leftTalon;
  private final WPI_TalonSRX rightTalon;

  private final DifferentialDrive m_drive;

  public boolean m_reverseDrive = false;

  public DrivetrainTalons() {
    // Talons
    leftTalon = new WPI_TalonSRX(Constants.kLeftTalonPort);
    rightTalon = new WPI_TalonSRX(Constants.kRightTalonPort);
    
    leftTalon.configFactoryDefault();
    rightTalon.configFactoryDefault();

    m_drive = new DifferentialDrive(leftTalon, rightTalon);
  }

  public void tankDrive(double leftPower, double rightPower, boolean squareInputs) {
    if (m_reverseDrive) {
      m_drive.tankDrive(rightPower/2, leftPower/2, squareInputs);
    }
    else {
      m_drive.tankDrive(leftPower/2, rightPower/2, squareInputs); 
    }
  }

  public void stopDrive() {
    m_drive.tankDrive(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
