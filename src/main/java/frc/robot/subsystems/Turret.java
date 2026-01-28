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
 * * Hardware:
 * - Motor: NEO on SparkMax ID 12 (19T Pinion)
 * - Sensor 1: Absolute Encoder on 19T Pinion (Main Motor)
 * - Sensor 2: Absolute Encoder on 21T Idler (Aux SparkMax ID 11)
 * - Main Turret Ring: 200T
 * * Strategy:
 * 1. Read absolute encoders on startup (Expecting 0.0 - 1.0 range).
 * 2. Use Brute-Force/Iterative logic to determine absolute Turret angle.
 * 3. Seed the internal NEO relative encoder with this absolute angle.
 * 4. Run PID control on the internal relative encoder (which doesn't wrap).
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
    
    // The pattern repeats every LCM(19, 21) = 399 encoder teeth.
    // 399 encoder teeth / 200 turret teeth = ~1.995 Turret Rotations of unique range.
    // This covers approx +/- 359 degrees.
    private final double k_uniqueRangeRotations = (k_gear19 * k_gear21) / k_turretRingTeeth;

    // Soft Limit Constants (Degrees)
    private final double k_forwardSoftLimit = 350.0;
    private final double k_reverseSoftLimit = -350.0;
    
    // PID Constants
    private final double kP = 0.05;
    private final double kI = 0.0;
    private final double kD = 0.0;

    private double m_targetAngle = 0.0;

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

        // Configure Internal Encoder (Relative) for Control
        // The internal encoder counts rotations of the motor shaft.
        // We convert this to Degrees of the Turret.
        // Gear Ratio = 200 (Turret) / 19 (Pinion) = 10.526...
        // 10.526 motor rotations = 360 degrees turret.
        double positionFactor = 360.0 / (k_turretRingTeeth / k_gear19);
        double velocityFactor = positionFactor / 60.0;

        motorConfig.encoder
            .positionConversionFactor(positionFactor)
            .velocityConversionFactor(velocityFactor);

        // Configure Absolute Encoder for Reading
        // CONFIRMED: We set factor to 1.0, so getPosition() returns 0.0 to 1.0.
        // This represents one full rotation of the 19T gear.
        motorConfig.absoluteEncoder
            .positionConversionFactor(1.0) 
            .velocityConversionFactor(1.0);

        // PID controls the Internal Relative Encoder, NOT the wrapping absolute encoder
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder) 
            .p(kP)
            .i(kI)
            .d(kD)
            .outputRange(-0.5, 0.5); // Safe speed limit for testing

        // Soft Limits
        motorConfig.softLimit
            .forwardSoftLimit(k_forwardSoftLimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(k_reverseSoftLimit)
            .reverseSoftLimitEnabled(true);

        // 2. Configure Aux Motor (21T) - Sensor Only
        // CONFIRMED: We set factor to 1.0, so getPosition() returns 0.0 to 1.0.
        auxConfig.absoluteEncoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);
        
        // Apply configurations
        m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_auxSpark.configure(auxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // 3. Calculate Absolute Position and Seed Internal Encoder
        resetToAbsolute();
    }

    /**
     * Uses an Iterative Search (Best Fit) to find the absolute position.
     * This is more robust than direct formula CRT for noisy sensors.
     */
    public void resetToAbsolute() {
        // 1. Get raw rotations [0.0, 1.0)
        // These are the fractional rotations of each gear.
        double r19 = m_encoder19.getPosition(); 
        double r21 = m_encoder21.getPosition();

        // 2. Iterative Search
        // The 19T gear rotates 21 times before the pattern with the 21T gear repeats perfectly.
        // We check all "k" values from 0 to 20 to see which total rotation count for the 19T gear
        // aligns with the current reading of the 21T gear.
        
        double bestError = Double.MAX_VALUE;
        double bestTurretDegrees = 0.0;

        // Loop through possible full rotations of the 19T gear
        for (int k = 0; k < 21; k++) {
            // Hypothetical total rotations of 19T gear
            // e.g. 5 full rotations + 0.3 partial rotation
            double totalRotations19 = k + r19;

            // Calculate resulting Turret Rotations for this hypothesis
            // (Total 19 Rotations) / (Gear Ratio 200/19) = Turret Rotations
            double turretRotations = totalRotations19 / (k_turretRingTeeth / k_gear19);

            // Calculate Expected 21T Encoder Value for this Turret Angle
            // (Turret Rotations) * (Gear Ratio 200/21) = Total 21 Rotations
            double totalRotations21 = turretRotations * (k_turretRingTeeth / k_gear21);
            
            // Get the fractional part (0.0 - 1.0) expected for the 21T encoder
            double expectedR21 = totalRotations21 % 1.0;
            if (expectedR21 < 0) expectedR21 += 1.0;

            // Calculate Error (cyclic distance between expected and actual r21)
            double error = Math.abs(r21 - expectedR21);
            if (error > 0.5) error = 1.0 - error; // Handle wrap (0.99 vs 0.01 is error 0.02)

            if (error < bestError) {
                bestError = error;
                bestTurretDegrees = turretRotations * 360.0;
            }
        }

        // Safety check for sensor health
        if (bestError > 0.05) { // 5% of a rotation error is quite high (18 degrees on the gear)
            System.err.println("[Turret] WARNING: High CRT Error: " + bestError + ". Sensors may be slipping or offset.");
        }

        // 3. Center the Result
        // The unique range is ~718 degrees (0 to 718).
        // We map this to match our physical setup, e.g., -359 to +359.
        // Assuming 0 is "forward", and the range wraps around back.
        double maxUniqueDeg = k_uniqueRangeRotations * 360.0; // ~718.2 degrees
        
        // If the calculated value is very large (e.g., > 360), it corresponds to negative angles
        if (bestTurretDegrees > (maxUniqueDeg / 2.0)) {
            bestTurretDegrees -= maxUniqueDeg;
        }

        // 4. Seed the internal encoder
        // This makes the PID controller think we are at the correct absolute angle
        // without worrying about the absolute encoders wrapping 0-1 constantly.
        m_internalEncoder.setPosition(bestTurretDegrees);
        m_targetAngle = bestTurretDegrees;
        
        System.out.println("Turret Seeded: " + bestTurretDegrees + " deg (Fit Error: " + bestError + ")");
    }

    /**
     * Sets the target angle for the turret to aim at.
     */
    public void setAngle(double degrees) {
        m_targetAngle = MathUtil.clamp(degrees, k_reverseSoftLimit, k_forwardSoftLimit);
        m_closedLoopController.setSetpoint(m_targetAngle, ControlType.kPosition);
    }

    /**
     * Manual control percentage.
     */
    public void setPercentOutput(double speed) {
        m_motor.set(MathUtil.clamp(speed, -1.0, 1.0));
    }

    public double getCurrentAngle() {
        return m_internalEncoder.getPosition();
    }

    public boolean atTarget(double tolerance) {
        return Math.abs(getCurrentAngle() - m_targetAngle) < tolerance;
    }

    public void stop() {
        m_motor.stopMotor();
    }

    // Debug accessors
    public double getRawAbs19() { return m_encoder19.getPosition(); }
    public double getRawAbs21() { return m_encoder21.getPosition(); }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Angle", getCurrentAngle());
        SmartDashboard.putNumber("Turret/Target", m_targetAngle);
        SmartDashboard.putNumber("Turret/Abs 19", getRawAbs19());
        SmartDashboard.putNumber("Turret/Abs 21", getRawAbs21());
    }
}