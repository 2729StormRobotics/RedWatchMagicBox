package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.MotorTestCommand;
import frc.robot.commands.PIDTuningCommand;
import frc.robot.subsystems.MotorSubsystem;
import frc.robot.subsystems.SensorHub;

/**
 * RobotContainer is the heart of the MagicBox.
 * This is where we define our subsystems, commands, and button mappings.
 */
public class RobotContainer {
    // 1. Subsystems
    private final MotorSubsystem m_motorSubsystem = new MotorSubsystem();
    private final SensorHub m_sensorHub = new SensorHub();

    // 2. Controllers
    // Standard Xbox Controller on Port 0
    private final XboxController m_driverController = new XboxController(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set Default Command: Manual Motor Control
        // Uses Left Y for speed and Right Trigger as a "Precise multiplier"
        m_motorSubsystem.setDefaultCommand(
            new MotorTestCommand(
                m_motorSubsystem, 
                () -> -m_driverController.getLeftY(), 
                () -> 0.5 + (m_driverController.getRightTriggerAxis() * 0.5)
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        // Add a "Refresh Hardware" button to Dashboard
        // This allows you to update CAN ID or Motor Model without redeploying
        SmartDashboard.putData("MagicBox/SYSTEM/RE-INIT HARDWARE", 
            new InstantCommand(m_motorSubsystem::reinitializeHardware).ignoringDisable(true));
    }

    /**
     * Map buttons to commands.
     */
    private void configureButtonBindings() {
        // Hold 'A' Button to enter PID Tuning Mode
        // This will override the default manual control while held
        new JoystickButton(m_driverController, XboxController.Button.kA.value)
            .whileTrue(new PIDTuningCommand(m_motorSubsystem));

        // Press 'B' Button to emergency stop the motor
        new JoystickButton(m_driverController, XboxController.Button.kB.value)
            .onTrue(new InstantCommand(m_motorSubsystem::stop, m_motorSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // No auto for MagicBox usually, but we could return null or a simple timer test
        return null;
    }
}