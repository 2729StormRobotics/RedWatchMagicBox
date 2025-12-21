package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorSubsystem;

/**
 * PIDTuningCommand for the MagicBox.
 * This command pulls PID constants from the Dashboard every loop,
 * allowing for live tuning of a mechanism.
 */
public class PIDTuningCommand extends Command {
    private final MotorSubsystem m_subsystem;
    
    // Note: This specific implementation assumes you might add 
    // SparkMax PIDController logic to your subsystem later.
    // For now, it logs and prepares the interface.

    public PIDTuningCommand(MotorSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(m_subsystem);

        // Ensure entries exist on the Dashboard
        SmartDashboard.putNumber("MagicBox/PID/kP", 0.0);
        SmartDashboard.putNumber("MagicBox/PID/kI", 0.0);
        SmartDashboard.putNumber("MagicBox/PID/kD", 0.0);
        SmartDashboard.putNumber("MagicBox/PID/Setpoint", 0.0);
    }

    @Override
    public void initialize() {
        System.out.println("MagicBox: Starting PID Tuning Mode...");
    }

    @Override
    public void execute() {
        // Read constants from Dashboard
        double p = SmartDashboard.getNumber("MagicBox/PID/kP", 0.0);
        double i = SmartDashboard.getNumber("MagicBox/PID/kI", 0.0);
        double d = SmartDashboard.getNumber("MagicBox/PID/kD", 0.0);
        double setpoint = SmartDashboard.getNumber("MagicBox/PID/Setpoint", 0.0);

        // If you add a PID method to MotorSubsystem, call it here:
        // m_subsystem.runPID(setpoint, p, i, d);
        
        // For now, we simply log that we are in tuning mode
        SmartDashboard.putBoolean("MagicBox/PID/Active", true);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stop();
        SmartDashboard.putBoolean("MagicBox/PID/Active", false);
    }
}