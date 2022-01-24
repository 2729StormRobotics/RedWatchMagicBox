// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DrivetrainTalons extends SubsystemBase {
  /** Creates a new Drivetrain. */

  // Create Talons
  private final WPI_TalonSRX leftTalon;
  private final WPI_TalonSRX rightTalon;

  private final DifferentialDrive m_drive;

  // Shuffleboards
  private final ShuffleboardTab m_drivetrainTab;
  private final ShuffleboardLayout m_drivetrainStatus;

  public DrivetrainTalons() {
    // Talons
    leftTalon = new WPI_TalonSRX(Constants.kLeftTalonPort);
    rightTalon = new WPI_TalonSRX(Constants.kRightTalonPort);
    
    leftTalon.configFactoryDefault();
    rightTalon.configFactoryDefault();

    // leftTalon.setInverted(true);
    // rightTalon.setInverted(true);

    m_drive = new DifferentialDrive(leftTalon, rightTalon);

    // Initialize Shuffleboard
    m_drivetrainTab = Shuffleboard.getTab(Constants.kShuffleboardTabTalon);
    m_drivetrainStatus = m_drivetrainTab.getLayout("Status", BuiltInLayouts.kList)
      .withSize(3,3)
      .withProperties(Map.of("Label position", "TOP"));
    shuffleboardInit();
  }

  private double getLeftSpeed() {
    return leftTalon.getSelectedSensorVelocity();
  }

  private double getRightSpeed() {
    return rightTalon.getSelectedSensorVelocity();
  }

  private void shuffleboardInit() {
    m_drivetrainStatus.addNumber("Left Speed", () -> getLeftSpeed());
    m_drivetrainStatus.addNumber("Right Speed", () -> getRightSpeed());
    m_drivetrainStatus.addNumber("Left Output", () -> leftTalon.get());
    m_drivetrainStatus.addNumber("Right Output", () -> rightTalon.get());
    // m_drivetrainStatus.addNumber("Left Position", () -> getLeftDistance());
    // m_drivetrainStatus.addNumber("Right Position", () -> getRightDistance());
    // m_drivetrainStatus.addNumber("Angle", () -> getRobotAngle());
  }

  public void tankDrive(double leftPower, double rightPower, boolean squareInputs) {
    m_drive.tankDrive(leftPower/2, rightPower/2, squareInputs);
  }

  public void stopDrive() {
    m_drive.tankDrive(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
