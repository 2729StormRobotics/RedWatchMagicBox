package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorSubsystem;
import java.util.function.DoubleSupplier;

/**
 * MotorTestCommand allows for manual control of the MagicBox motor.
 * It includes a deadband and a speed multiplier for precision testing.
 */
public class MotorTestCommand extends Command {
    private final MotorSubsystem m_subsystem;
    private final DoubleSupplier m_speedSupplier;
    private final DoubleSupplier m_multiplierSupplier;

    private static final double DEADBAND = 0.1;

    /**
     * @param subsystem The motor subsystem.
     * @param speedSupplier Typically a joystick axis (e.g., controller.getLeftY()).
     * @param multiplierSupplier Typically a trigger or bumper to scale speed (default 1.0).
     */
    public MotorTestCommand(
            MotorSubsystem subsystem, 
            DoubleSupplier speedSupplier, 
            DoubleSupplier multiplierSupplier) {
        m_subsystem = subsystem;
        m_speedSupplier = speedSupplier;
        m_multiplierSupplier = multiplierSupplier;
        addRequirements(m_subsystem);
    }

    @Override
    public void execute() {
        double rawInput = m_speedSupplier.getAsDouble();
        double multiplier = m_multiplierSupplier.getAsDouble();

        // Apply deadband
        double speed = Math.abs(rawInput) < DEADBAND ? 0 : rawInput;
        
        // Scale speed
        m_subsystem.setSpeed(speed * multiplier);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stop();
    }
}