package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

/**
 * SensorHub for the FRC MagicBox.
 * This subsystem automatically monitors all standard RoboRIO ports (DIO and Analog).
 * It is used to quickly verify if sensors (limit switches, beam breaks, ultrasonic, etc.)
 * are wired correctly and functioning without needing custom code for each sensor.
 */
public class SensorHub extends SubsystemBase {

    // Lists to hold all possible ports
    private final List<DigitalInput> m_dioPorts = new ArrayList<>();
    private final List<AnalogInput> m_analogPorts = new ArrayList<>();

    // Number of ports on a standard RoboRIO
    private static final int NUM_DIO_PORTS = 10;
    private static final int NUM_ANALOG_PORTS = 4;

    /**
     * Creates a new SensorHub.
     * Initializes all hardware ports.
     */
    public SensorHub() {
        // Initialize all DIO ports
        for (int i = 0; i < NUM_DIO_PORTS; i++) {
            try {
                m_dioPorts.add(new DigitalInput(i));
            } catch (Exception e) {
                System.out.println("SensorHub: Could not initialize DIO " + i + " (Check if used elsewhere)");
            }
        }

        // Initialize all Analog ports
        for (int i = 0; i < NUM_ANALOG_PORTS; i++) {
            try {
                m_analogPorts.add(new AnalogInput(i));
            } catch (Exception e) {
                System.out.println("SensorHub: Could not initialize Analog " + i + " (Check if used elsewhere)");
            }
        }
    }

    @Override
    public void periodic() {
        // Update Digital Inputs
        // True usually means "Open", False usually means "Tripped" for most FRC sensors
        for (int i = 0; i < m_dioPorts.size(); i++) {
            DigitalInput sensor = m_dioPorts.get(i);
            if (sensor != null) {
                SmartDashboard.putBoolean("MagicBox/Sensors/DIO " + i, sensor.get());
            }
        }

        // Update Analog Inputs
        for (int i = 0; i < m_analogPorts.size(); i++) {
            AnalogInput sensor = m_analogPorts.get(i);
            if (sensor != null) {
                // Post voltage for raw readings (0V to 5V)
                SmartDashboard.putNumber("MagicBox/Sensors/Analog " + i + " (V)", sensor.getVoltage());
                // Post 12-bit value (0 to 4095) for high-res debugging
                SmartDashboard.putNumber("MagicBox/Sensors/Analog " + i + " (Raw)", sensor.getValue());
            }
        }

        // System Health monitoring
        SmartDashboard.putNumber("MagicBox/System/RIO Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putBoolean("MagicBox/System/User Button", RobotController.getUserButton());
    }

    /**
     * Utility method to check if a specific DIO is triggered.
     * @param channel The DIO channel index.
     * @return Current state of the sensor.
     */
    public boolean getDigitalValue(int channel) {
        if (channel >= 0 && channel < m_dioPorts.size()) {
            return m_dioPorts.get(channel).get();
        }
        return false;
    }

    /**
     * Utility method to get an analog voltage.
     * @param channel The Analog channel index.
     * @return Current voltage (0-5V).
     */
    public double getAnalogVoltage(int channel) {
        if (channel >= 0 && channel < m_analogPorts.size()) {
            return m_analogPorts.get(channel).getVoltage();
        }
        return 0.0;
    }
}