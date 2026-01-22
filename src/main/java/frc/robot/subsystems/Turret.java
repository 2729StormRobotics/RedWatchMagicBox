package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Turret Subsystem utilizing Chinese Remainder Theorem (CRT) with 19T and 21T gears.
 * Updated for REVLib 2026 (2025.x.x) Specifications.
 * * Hardware:
 * - Motor: NEO on SparkMax ID 12, geared 4:1 to a 19T pinion.
 * - Primary Encoder: Absolute encoder on SparkMax ID 12 (19T pinion).
 * - Auxiliary Encoder: Absolute encoder on SparkMax ID 11 (21T pinion).
 */
public class Turret extends SubsystemBase {
    private final SparkMax m_motor;
    private final SparkMax m_auxSpark; 
    
    private final SparkAbsoluteEncoder m_encoder19;
    private final SparkAbsoluteEncoder m_encoder21;
    
    private final SparkClosedLoopController m_closedLoopController;

    // Gear Constants
    private final double k_turretRingTeeth = 200.0; 
    private final double k_gear19 = 19.0;
    private final double k_gear21 = 21.0;
    private final double k_totalRangeTeeth = 19.0 * 21.0; // 399 teeth

    // Soft Limit Constants (Degrees)
    private final double k_forwardSoftLimit = 350.0;
    private final double k_reverseSoftLimit = -350.0;

    // Store position conversion factor for CRT calculations
    private final double m_positionConversionFactor;
    
    // Track the offset between encoder reading and absolute position
    private double m_encoderOffset = 0.0;

    private double m_targetAngle = 0.0;

    public Turret() {
        m_motor = new SparkMax(12, MotorType.kBrushless);
        m_auxSpark = new SparkMax(11, MotorType.kBrushless);

        m_encoder19 = m_motor.getAbsoluteEncoder();
        m_encoder21 = m_auxSpark.getAbsoluteEncoder();

        m_closedLoopController = m_motor.getClosedLoopController();

        /*
         * REVLib 2026 Configuration Pattern:
         * We create a config object, apply settings, and then push to the controller.
         */
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        SparkMaxConfig auxConfig = new SparkMaxConfig();

        // Position Conversion: 1 rotation of encoder = (19 / RingTeeth) * 360 degrees
        m_positionConversionFactor = (k_gear19 / k_turretRingTeeth) * 360.0;

        motorConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false);

        motorConfig.absoluteEncoder
            .positionConversionFactor(m_positionConversionFactor)
            .velocityConversionFactor(m_positionConversionFactor / 60.0);

        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(0.1)
            .i(0.0)
            .d(0.0)
            .outputRange(-1.0, 1.0);

        // Apply configs with Persist and Reset to ensure clean state
        m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Setup Aux Spark as a simple sensor node (Minimize CAN traffic)
        auxConfig.absoluteEncoder.positionConversionFactor(1.0); // Raw rotations for CRT
        m_auxSpark.configure(auxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetToAbsolute();
    }

    /**
     * Uses the Chinese Remainder Theorem to find the absolute physical position
     * of the turret without homing.
     */
    public void resetToAbsolute() {
        // Get raw encoder positions (0.0 to 1.0 rotations)
        // Encoder 19 returns position in degrees (after conversion factor)
        // We need raw rotations for CRT, so divide by conversion factor
        double r19 = m_encoder19.getPosition() / m_positionConversionFactor;
        // Encoder 21 is configured with raw rotations (conversion factor = 1.0)
        double r21 = m_encoder21.getPosition();

        // Normalize to tooth remainders (ensuring positive modulo for robustness)
        double a19 = ((r19 % 1.0) + 1.0) % 1.0 * k_gear19;
        double a21 = ((r21 % 1.0) + 1.0) % 1.0 * k_gear21;

        // CRT Solver for 19 and 21 (Co-prime)
        // Using extended Euclidean algorithm coefficients:
        // For moduli 19 and 21: 19*10 + 21*(-9) = 1
        // So: x = (a19 * 21 * 10 + a21 * 19 * 10) mod 399
        double x = (a19 * 21 * 10 + a21 * 19 * 10) % k_totalRangeTeeth;
        if (x < 0) x += k_totalRangeTeeth; // Ensure positive

        // Convert teeth to degrees
        double absoluteDegrees = (x / k_turretRingTeeth) * 360.0;
        
        // Logic to center the turret (assuming 0 is "forward")
        if (absoluteDegrees > 180.0) absoluteDegrees -= 360.0;

        // Store the offset between encoder reading and calculated absolute position
        // This allows us to convert encoder readings to absolute positions
        double currentReading = m_encoder19.getPosition();
        m_encoderOffset = currentReading - absoluteDegrees;
        
        m_targetAngle = absoluteDegrees;
    }

    /**
     * Sets the target angle for the turret to aim at.
     * Automatically clamps to soft limits.
     * @param degrees target angle in degrees.
     */
    public void setAngle(double degrees) {
        m_targetAngle = Math.max(k_reverseSoftLimit, Math.min(k_forwardSoftLimit, degrees));
        // Use setReference with position control (deprecated but functional)
        // Alternative: use m_closedLoopController.setPosition() if available
        m_closedLoopController.setSetpoint(m_targetAngle, ControlType.kPosition);
    }

    /**
     * Sets the turret motor to a percentage output.
     * Useful for manual control or testing.
     * @param speed percentage output from -1.0 to 1.0
     */
    public void setPercentOutput(double speed) {
        // Clamp speed to safe range
        speed = Math.max(-1.0, Math.min(1.0, speed));
        m_motor.set(speed);
    }

    /**
     * Gets the current absolute angle of the turret.
     * Uses the stored offset to convert encoder reading to absolute position.
     * @return current angle in degrees
     */
    public double getCurrentAngle() {
        return m_encoder19.getPosition() - m_encoderOffset;
    }

    /**
     * Checks if the turret is at the target angle within tolerance.
     * @param tolerance tolerance in degrees (default 1.0)
     * @return true if within tolerance
     */
    public boolean atTarget(double tolerance) {
        return Math.abs(getCurrentAngle() - m_targetAngle) < tolerance;
    }

    /**
     * Checks if the turret is at the target angle within default tolerance (1.0 degrees).
     * @return true if within tolerance
     */
    public boolean atTarget() {
        return atTarget(1.0);
    }

    /**
     * Gets the angle error (target - current).
     * @return angle error in degrees
     */
    public double getAngleError() {
        return m_targetAngle - getCurrentAngle();
    }

    /**
     * Gets the target angle.
     * @return target angle in degrees
     */
    public double getTargetAngle() {
        return m_targetAngle;
    }

    /**
     * Stops the turret motor.
     */
    public void stop() {
        m_motor.set(0.0);
    }

    /**
     * Checks if the requested angle is within soft limits.
     * @param angle angle to check in degrees
     * @return true if within limits
     */
    public boolean isWithinLimits(double angle) {
        return angle >= k_reverseSoftLimit && angle <= k_forwardSoftLimit;
    }

    /**
     * Gets the raw encoder position for the 19T gear (for debugging).
     * @return raw encoder position
     */
    public double getRawEncoder19() {
        return m_encoder19.getPosition();
    }

    /**
     * Gets the raw encoder position for the 21T gear (for debugging).
     * @return raw encoder position
     */
    public double getRawEncoder21() {
        return m_encoder21.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Current Angle", getCurrentAngle());
        SmartDashboard.putNumber("Turret/Target Angle", m_targetAngle);
        SmartDashboard.putNumber("Turret/Angle Error", getAngleError());
        SmartDashboard.putBoolean("Turret/At Target", atTarget());
        SmartDashboard.putNumber("Turret/Raw Encoder 19", getRawEncoder19());
        SmartDashboard.putNumber("Turret/Raw Encoder 21", getRawEncoder21());
    }
}