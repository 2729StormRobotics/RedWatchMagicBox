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
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Turret Subsystem utilizing Chinese Remainder Theorem (CRT) Logic.
 * Updated to use MAXMotion for smooth profiling.
 */
public class Turret extends SubsystemBase {
    private final SparkMax m_motor;
    private final SparkMax m_auxSpark; 
    
    private final SparkAbsoluteEncoder m_encoder19;
    private final SparkAbsoluteEncoder m_encoder21;
    private final RelativeEncoder m_internalEncoder;
    
    private final SparkClosedLoopController m_closedLoopController;

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
    
    // PID & MAXMotion Constants
    // MAXMotion needs aggressive P because the "Target" moves smoothly.
    private final double kP = 0.2; 
    private final double kI = 0.0; // Usually not needed with MAXMotion
    private final double kD = 0.0;
    private final double kFF = 0.015; // Small feedforward helps maintain velocity
    
    // Motion Profile Limits
    private final double kMaxVelocity = 300.0; // Degrees per second
    private final double kMaxAccel = 600.0;    // Degrees per second squared

    private double m_targetAngle = 0.0;

    // Variables for Dashboard Debugging
    private double m_lastCrtError = 0.0;

    public Turret() {
        m_motor = new SparkMax(12, MotorType.kBrushless);
        m_auxSpark = new SparkMax(11, MotorType.kBrushless);

        m_encoder19 = m_motor.getAbsoluteEncoder();
        m_encoder21 = m_auxSpark.getAbsoluteEncoder();
        m_internalEncoder = m_motor.getEncoder();

        m_closedLoopController = m_motor.getClosedLoopController();

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

        // Configure PID + MAXMotion
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder) 
            .p(kP)
            .i(kI)
            .d(kD)
            .velocityFF(kFF)
            .outputRange(-1.0, 1.0);

        motorConfig.closedLoop.maxMotion
            .maxVelocity(kMaxVelocity)
            .maxAcceleration(kMaxAccel)
            .allowedClosedLoopError(1.0); // Allow 1 degree of error

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

        // 3. Calculate Absolute Position
        resetToAbsolute();
    }

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
        
        // Use MAXMotion Control Type
        m_closedLoopController.setReference(m_targetAngle, ControlType.kMAXMotionPositionControl);
        
        System.out.println("Turret Seeded: " + angle + " deg (Fit Error: " + error + ")");
    }

    public void setAngle(double degrees) {
        m_targetAngle = MathUtil.clamp(degrees, k_reverseSoftLimit, k_forwardSoftLimit);
        // CHANGED: Use MAXMotion Position Control
        m_closedLoopController.setReference(m_targetAngle, ControlType.kMAXMotionPositionControl);
    }

    public void setPercentOutput(double speed) {
        m_motor.set(MathUtil.clamp(speed, -1.0, 1.0));
    }

    public double getCurrentAngle() { return m_internalEncoder.getPosition(); }
    public boolean atTarget(double tolerance) { return Math.abs(getCurrentAngle() - m_targetAngle) < tolerance; }
    public void stop() { m_motor.stopMotor(); }
    public double getTargetAngle() { return m_targetAngle; }
    public double getRawAbs19() { return m_encoder19.getPosition(); }
    public double getRawAbs21() { return m_encoder21.getPosition(); }

    @Override
    public void periodic() {
        double[] crtData = calculateAbsAngle();
        
        SmartDashboard.putNumber("Turret/Angle", getCurrentAngle());
        SmartDashboard.putNumber("Turret/Target", m_targetAngle);
        SmartDashboard.putNumber("Turret/LiveCRTAngle", crtData[0]); 
        SmartDashboard.putNumber("Turret/CRTError", crtData[1]); 
        
        SmartDashboard.putNumber("Turret/Abs 19", getRawAbs19());
        SmartDashboard.putNumber("Turret/Abs 21", getRawAbs21());
    }
}