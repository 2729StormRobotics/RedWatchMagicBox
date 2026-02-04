package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.function.Supplier;

public class Hood extends SubsystemBase {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController positionController;

  private static final double kP = 0.03; 
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private final int MOTOR_ID = 12; 
  private final boolean INVERTED = false;
  private final int CURRENT_LIMIT_AMPS = 40; 

  private double positionSetpointRotations = 0.0;

  public Hood() {
    motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    positionController = motor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake) 
        .smartCurrentLimit(CURRENT_LIMIT_AMPS)
        .inverted(INVERTED)
        .voltageCompensation(12.0);

    // SOFT LIMITS: Forward is -10 (higher), Reverse is -310 (lower)
    config.softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(-1.0)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(-37.0);

    config.encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP, kI, kD);

    tryUntilOk(
        motor,
        5,
        () -> motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Zero the hood on startup
    tryUntilOk(motor, 5, () -> encoder.setPosition(0.0));
    encoder.setPosition(0);
    // Initial "Idle" target (stay at 0 until a command tells it otherwise)
    positionSetpointRotations = 15;
  }

  public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      if (command.get() == REVLibError.kOk) break;
    }
  }

  public void setPosition(double targetRotations) {
    positionSetpointRotations = targetRotations;
    positionController.setReference(targetRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public boolean isAtPosition(double target) {
    return Math.abs(getPosition() - target) < 1.0; 
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public void stop() {
    positionSetpointRotations = getPosition();
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This allows you to see the hood status in Shuffleboard/SmartDashboard
    SmartDashboard.putNumber("Hood/Position", getPosition());
    SmartDashboard.putNumber("Hood/Setpoint", positionSetpointRotations);
    SmartDashboard.putBoolean("Hood/AtSetpoint", isAtPosition(positionSetpointRotations));
  }

  // =========================================================================
  //                            COMMAND FACTORIES
  // =========================================================================

  public Command runPositionCommand(double ticks) {
    return run(() -> setPosition(-ticks)).withName("HoodPosition: " + -ticks);
  }

  public Command runPositionCommandConstant(CommandXboxController m_operatorController) {
    SmartDashboard.putNumber("Hood/leftx", (m_operatorController.getLeftX()));
        SmartDashboard.putNumber("Hood/ly", (m_operatorController.getLeftY()));
        SmartDashboard.putNumber("Hood/rx", (m_operatorController.getRightX()));
        SmartDashboard.putNumber("Hood/ry", (m_operatorController.getRightY()));
        SmartDashboard.putNumber("Hood/rt", (m_operatorController.getRightTriggerAxis()));
    return new RepeatCommand(run(() -> setPosition(-(19+(-m_operatorController.getLeftY()*18)))).withName("HoodPosition: " + -(19+(-m_operatorController.getRightY()*18))));
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("HoodStop");
  }
}