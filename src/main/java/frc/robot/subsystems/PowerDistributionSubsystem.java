package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * PowerDistributionSubsystem for the MagicBox.
 * Monitors the PDH to track battery health and individual channel current draw.
 */
public class PowerDistributionSubsystem extends SubsystemBase {
    private final PowerDistribution m_pdh;

    public PowerDistributionSubsystem() {
        // PDH usually has a standard CAN ID of 1
        m_pdh = new PowerDistribution(1, ModuleType.kRev);
    }

    @Override
    public void periodic() {
        // Global Metrics
        SmartDashboard.putNumber("MagicBox/Power/Total Current (A)", m_pdh.getTotalCurrent());
        SmartDashboard.putNumber("MagicBox/Power/Battery Voltage (V)", m_pdh.getVoltage());
        SmartDashboard.putNumber("MagicBox/Power/Total Energy (J)", m_pdh.getTotalEnergy());
        SmartDashboard.putNumber("MagicBox/Power/Temperature (C)", m_pdh.getTemperature());

        // Monitoring specific channels (0-23 on a PDH)
        // You can use these to see if a specific motor is "struggling"
        for (int i = 0; i < 6; i++) { // Monitoring first 6 channels for common test setups
            SmartDashboard.putNumber("MagicBox/Power/Channel " + i + " Amps", m_pdh.getCurrent(i));
        }
    }

    /**
     * Resets the total energy counter.
     */
    public void resetEnergy() {
        m_pdh.resetTotalEnergy();
    }

    /**
     * Clears any sticky faults on the PDH.
     */
    public void clearFaults() {
        m_pdh.clearStickyFaults();
    }
}