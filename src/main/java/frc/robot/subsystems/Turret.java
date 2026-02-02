package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Turret Subsystem utilizing Chinese Remainder Theorem (CRT) Logic.
 * Uses RIO-side ProfiledPIDController for smooth motion (replacing SparkMax MAXMotion).
 */
public class Turret extends SubsystemBase {
    private final SparkMax m_motor;
    private final SparkMax m_auxSpark; 
    
    private final SparkAbsoluteEncoder m_encoder19;
    private final SparkAbsoluteEncoder m_encoder21;
    private final RelativeEncoder m_internalEncoder;
    
    // RIO-Side PID Controller with Motion Profiling
    // Constraints: Max Velocity (deg/s), Max Acceleration (deg/s^2)
    private final TrapezoidProfile.Constraints m_constraints = 
        new TrapezoidProfile.Constraints(500.0, 600.0);
    private final ProfiledPIDController m_pidController = 
        new ProfiledPIDController(0.048, 0.0, 0.0, m_constraints);

    // Gear Constants
    private final double k_turretRingTeeth = 200.0; 
    private final double k_gear19 = 19.0;
    private final double k_gear21 = 21.0;
    private final double k_gearboxRatio = 4.0; // 4:1 Reduction Motor -> 19T Pinion

    // *** IMPORTANT: TUNE THESE OFFSETS ***
    private final double k_enc19Offset = 0.0; 
    private final double k_enc21Offset = 0.0;

    private final double k_uniqueRangeRotations = (k_gear19 * k_gear21) / k_turretRingTeeth;

    // Soft Limit Constants (Degrees)
    private final double k_forwardSoftLimit = 360.0;
    private final double k_reverseSoftLimit = -360.0;

    // Internal state
    private double m_targetAngle = 0.0;
    private double m_lastCrtError = 0.0;

    public Turret() {
        m_motor = new SparkMax(12, MotorType.kBrushless);
        m_auxSpark = new SparkMax(11, MotorType.kBrushless);

        m_encoder19 = m_motor.getAbsoluteEncoder();
        m_encoder21 = m_auxSpark.getAbsoluteEncoder();
        m_internalEncoder = m_motor.getEncoder();

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        SparkMaxConfig auxConfig = new SparkMaxConfig();

        // 1. Configure Main Motor (19T)
        motorConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false);

        // Configure Internal Encoder Conversion
        double totalGearRatio = k_gearboxRatio * (k_turretRingTeeth / k_gear19);
        double positionFactor = 360.0 / totalGearRatio;
        double velocityFactor = positionFactor / 60.0;

        motorConfig.encoder
            .positionConversionFactor(positionFactor)
            .velocityConversionFactor(velocityFactor);

        // Configure Absolute Encoder for Reading (0.0 - 1.0)
        motorConfig.absoluteEncoder
            .positionConversionFactor(1.0) 
            .velocityConversionFactor(1.0);

        // Soft Limits
        motorConfig.softLimit
            .forwardSoftLimit(k_forwardSoftLimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(k_reverseSoftLimit)
            .reverseSoftLimitEnabled(true);

        // 2. Configure Aux Motor (21T) - Sensor Only
        auxConfig.absoluteEncoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);
        
        // Apply configurations
        m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_auxSpark.configure(auxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // 3. Setup PID Tolerance
        m_pidController.setTolerance(1.0); // 1 degree tolerance

        // 4. Calculate Absolute Position
        resetToAbsolute();
    }

    // =========================================================================
    //                        COMMAND FACTORIES
    // =========================================================================

    /**
     * Command: Holds the turret at a specific angle using Motion Profiling.
     * This command never finishes (it holds the angle), so use it with .onTrue() or as a Default Command.
     */
    public Command runToAngleCommand(double degrees) {
        return run(() -> {
            // 1. Clamp Target
            m_targetAngle = MathUtil.clamp(degrees, k_reverseSoftLimit, k_forwardSoftLimit);
            
            // 2. Calculate Output
            // The ProfiledPIDController creates a smooth path from current to target
            double output = m_pidController.calculate(getCurrentAngle(), m_targetAngle);
            
            // 3. Apply Output
            m_motor.set(MathUtil.clamp(output, -1.0, 1.0));
        })
        .beforeStarting(() -> {
            // Reset the profile to start at the current speed and position
            m_pidController.reset(getCurrentAngle());
        })
        .withName("RunToAngle: " + degrees);
    }

    /**
     * Command: Runs the turret with manual joystick control.
     */
    public Command manualControlCommand(DoubleSupplier speedSupplier) {
        return run(() -> {
            double speed = speedSupplier.getAsDouble();
            // Deadband
            if (Math.abs(speed) < 0.1) speed = 0.0;
            
            // Scale speed (50% max for manual safety)
            m_motor.set(speed * 0.5);
        })
        .withName("ManualTurret");
    }

    /**
     * Command: Resets the turret to absolute position using CRT.
     * Runs once then finishes.
     */
    public Command resetToAbsoluteCommand() {
        return runOnce(this::resetToAbsolute).withName("ResetToAbsolute");
    }

    /**
     * Command: Stops the motor.
     */
    public Command stopCommand() {
        return runOnce(m_motor::stopMotor).withName("StopTurret");
    }



    // =========================================================================
    //                           LOGIC & METHODS
    // =========================================================================

    private double[] calculateAbsAngle() {
        double r19 = (m_encoder19.getPosition() - k_enc19Offset);
        double r21 = (m_encoder21.getPosition() - k_enc21Offset);

        r19 = ((r19 % 1.0) + 1.0) % 1.0;
        r21 = ((r21 % 1.0) + 1.0) % 1.0;

        double bestError = Double.MAX_VALUE;
        double bestTurretDegrees = 0.0;

        for (int k = 0; k < 21; k++) {
            double totalRotations19 = k + r19;
            double turretRotations = totalRotations19 / (k_turretRingTeeth / k_gear19);
            double totalRotations21 = turretRotations * (k_turretRingTeeth / k_gear21);
            
            double expectedR21 = totalRotations21 % 1.0;
            if (expectedR21 < 0) expectedR21 += 1.0;

            double error = Math.abs(r21 - expectedR21);
            if (error > 0.5) error = 1.0 - error; 

            if (error < bestError) {
                bestError = error;
                bestTurretDegrees = turretRotations * 360.0;
            }
        }

        double maxUniqueDeg = k_uniqueRangeRotations * 360.0; 
        if (bestTurretDegrees > (maxUniqueDeg / 2.0)) {
            bestTurretDegrees -= maxUniqueDeg;
        }

        return new double[] {bestTurretDegrees, bestError};
    }

    public void resetToAbsolute() {
        double[] result = calculateAbsAngle();
        double angle = result[0];
        double error = result[1];

        m_lastCrtError = error;

        if (error > 0.05) {
            System.err.println("[Turret] WARNING: High CRT Error: " + error + ". CHECK OFFSETS.");
        }

        m_internalEncoder.setPosition(angle);
        m_targetAngle = angle;
        
        // Reset PID state so it doesn't jump
        m_pidController.reset(angle);
        
        System.out.println("Turret Seeded: " + angle + " deg (Fit Error: " + error + ")");
    }

    public double getCurrentAngle() { return m_internalEncoder.getPosition(); }
    public double getTargetAngle() { return m_targetAngle; }
    public double getRawAbs19() { return m_encoder19.getPosition(); }
    public double getRawAbs21() { return m_encoder21.getPosition(); }
    public boolean atTarget() { return m_pidController.atGoal(); }

    @Override
    public void periodic() {
        double[] crtData = calculateAbsAngle();
        
        SmartDashboard.putNumber("Turret/Angle", getCurrentAngle());
        SmartDashboard.putNumber("Turret/Target", m_targetAngle);
        SmartDashboard.putNumber("Turret/LiveCRTAngle", crtData[0]); 
        SmartDashboard.putNumber("Turret/CRTError", crtData[1]); 
        SmartDashboard.putBoolean("Turret/AtSetpoint", atTarget());
        
        SmartDashboard.putNumber("Turret/Abs 19", getRawAbs19());
        SmartDashboard.putNumber("Turret/Abs 21", getRawAbs21());
    }
}