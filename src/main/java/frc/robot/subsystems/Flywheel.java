// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems;

// import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;
// import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Flywheel subsystem that controls the shooter wheels.
 * Consolidates hardware implementation and subsystem logic into a single file.
 */
public class Flywheel extends SubsystemBase {
  private final SparkMax leaderMotor;
  private final SparkMax followerMotor;
  private final RelativeEncoder leaderEncoder;
  private final SparkClosedLoopController velocityController;
  private  static final double kP = 0.0001;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kF = 0.02;
  private final int LEADER_MOTOR_ID = 9;
  private final int FOLLOWER_MOTOR_ID = 10;
  private final boolean LEADER_INVERTED = false;
  private final boolean FOLLOWER_INVERTED = true;
  private final int CURRENT_LIMIT_AMPS = 60;

  private double velocitySetpointRotationsPerSec = 0.0;

  public Flywheel() {
    // Create leader motor
    leaderMotor = new SparkMax(LEADER_MOTOR_ID, MotorType.kBrushless);
    leaderEncoder = leaderMotor.getEncoder();
    velocityController = leaderMotor.getClosedLoopController();

    // Create follower motor
    followerMotor = new SparkMax(FOLLOWER_MOTOR_ID, MotorType.kBrushless);

    // Configure leader motor
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(CURRENT_LIMIT_AMPS)
        .inverted(LEADER_INVERTED)
        .voltageCompensation(12.0);

    leaderConfig.encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0 / 60.0); // Convert RPM to RPS

    leaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP, kI, kD);

    tryUntilOk(
        leaderMotor,
        5,
        () ->
            leaderMotor.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Configure follower motor
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig
        .follow(LEADER_MOTOR_ID, true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(CURRENT_LIMIT_AMPS)
        .inverted(FOLLOWER_INVERTED)
        .voltageCompensation(12.0);

    tryUntilOk(
        followerMotor,
        5,
        () ->
            followerMotor.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Reset encoder
    tryUntilOk(leaderMotor, 5, () -> leaderEncoder.setPosition(0.0));
  }

  public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        // sparkStickyFault = true;
      }
    }
}

  /**
   * Sets the flywheel velocity setpoint.
   *
   * @param velocityRotationsPerSec Target velocity in rotations per second.
   */
  public void setVelocity(double velocityRotationsPerSec) {
    velocitySetpointRotationsPerSec = velocityRotationsPerSec;
    double ffVolts = kF * velocityRotationsPerSec;
    velocityController.setReference(
        velocityRotationsPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  /**
   * Runs the flywheel at a specific voltage.
   *
   * @param volts Voltage to apply.
   */
  public void setVoltage(double volts) {
    velocitySetpointRotationsPerSec = 0.0;
    leaderMotor.setVoltage(volts);
  }

  /** Stops the flywheel motors. */
  public void stop() {
    velocitySetpointRotationsPerSec = 0.0;
    leaderMotor.stopMotor();
  }

  /** Returns the current velocity of the leader motor in RPS. */
  public double getVelocity() {
    return leaderEncoder.getVelocity();
  }

  /** Returns the current velocity setpoint in RPS. */
  public double getSetpoint() {
    return velocitySetpointRotationsPerSec;
  }

  /** Returns true if the flywheel is at the target velocity. */
  public boolean atSetpoint() {
    return Math.abs(getVelocity() - velocitySetpointRotationsPerSec) < 2.0; // 2 RPS tolerance
  }

  // =========================================================================
  //                            COMMAND FACTORIES
  // =========================================================================

  public Command runVelocityCommand(double rps) {
    return run(() -> setVelocity(rps)).withName("FlywheelVelocity: " + rps);
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("FlywheelStop");
  }

  @Override
  public void periodic() {
    // Log data to SmartDashboard for easy tuning
    SmartDashboard.putNumber("Flywheel/VelocityRPS", getVelocity());
    SmartDashboard.putNumber("Flywheel/SetpointRPS", getSetpoint());
    SmartDashboard.putBoolean("Flywheel/AtSetpoint", atSetpoint());
    SmartDashboard.putNumber("Flywheel/CurrentAmps", leaderMotor.getOutputCurrent());
  }
}