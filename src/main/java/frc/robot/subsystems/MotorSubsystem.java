package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * MotorSubsystem for the FRC MagicBox.
 * This subsystem allows the team to test different REV motor types (Vortex, NEO, NEO 550)
 * without redeploying code, by selecting the motor type from the Dashboard.
 */
public class MotorSubsystem extends SubsystemBase {

    // REV Motor Types for MagicBox
    private enum MotorModel {
        NEO_VORTEX,
        NEO_BRUSHLESS,
        NEO_550
    }

    // Hardware references
    private SparkMax m_spark;
    
    // Config object
    private final SparkMaxConfig m_config = new SparkMaxConfig();

    // Dashboard Chooser
    private final SendableChooser<MotorModel> m_modelChooser = new SendableChooser<>();

    // State variables
    private int m_canId = 20; // Default MagicBox CAN ID
    private double m_lastSpeed = 0;

    /**
     * Creates a new MotorSubsystem.
     */
    public MotorSubsystem() {
        // Setup the Dashboard Chooser for REV Motors
        m_modelChooser.setDefaultOption("NEO (Standard)", MotorModel.NEO_BRUSHLESS);
        m_modelChooser.addOption("NEO Vortex", MotorModel.NEO_VORTEX);
        m_modelChooser.addOption("NEO 550", MotorModel.NEO_550);
        SmartDashboard.putData("MagicBox/Motor Model", m_modelChooser);

        // Put default CAN ID on Dashboard
        SmartDashboard.putNumber("MagicBox/Target CAN ID", m_canId);

        // Initialize hardware based on defaults
        reinitializeHardware();
    }

    /**
     * (Re)initializes the SparkMax. Call this if the CAN ID or Motor Model 
     * is changed on the Dashboard.
     */
    public void reinitializeHardware() {
        m_canId = (int) SmartDashboard.getNumber("MagicBox/Target CAN ID", 20);
        MotorModel selectedModel = m_modelChooser.getSelected();

        // 2025 REV API prefers creating a new instance if the ID changes
        m_spark = new SparkMax(m_canId, MotorType.kBrushless);
        
        // Apply safety configurations based on the specific motor model
        switch (selectedModel) {
            case NEO_VORTEX:
                m_config.smartCurrentLimit(60); 
                break;
            case NEO_550:
                m_config.smartCurrentLimit(20); 
                break;
            case NEO_BRUSHLESS:
            default:
                m_config.smartCurrentLimit(40);
                break;
        }

        // Apply configuration and reset parameters to a known state
        m_spark.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        System.out.println("MagicBox: Initialized " + selectedModel + " on ID " + m_canId);
    }

    /**
     * Drives the motor at a specific percentage.
     * @param speed Range -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        m_lastSpeed = speed;
        if (m_spark != null) {
            m_spark.set(speed);
        }
    }

    /**
     * Stops the motor.
     */
    public void stop() {
        setSpeed(0);
    }

    @Override
    public void periodic() {
        if (m_spark != null) {
            SmartDashboard.putNumber("MagicBox/Current (Amps)", m_spark.getOutputCurrent());
            SmartDashboard.putNumber("MagicBox/Temp (C)", m_spark.getMotorTemperature());
            SmartDashboard.putNumber("MagicBox/Encoder Position", m_spark.getEncoder().getPosition());
            SmartDashboard.putNumber("MagicBox/Applied Output", m_spark.getAppliedOutput());
        }

        SmartDashboard.putNumber("MagicBox/Requested Speed", m_lastSpeed);
    }
}