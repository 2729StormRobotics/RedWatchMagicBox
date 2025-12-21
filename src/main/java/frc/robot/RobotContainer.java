package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.MotorTestCommand;
import frc.robot.commands.PIDTuningCommand;
import frc.robot.subsystems.MotorSubsystem;
import frc.robot.subsystems.SensorHub;
import frc.robot.subsystems.PowerDistributionSubsystem;

/**
 * RobotContainer is the heart of the MagicBox.
 * This is where we define our subsystems, commands, and button mappings.
 */
public class RobotContainer {
    // 1. Subsystems
    private final MotorSubsystem m_motorSubsystem = new MotorSubsystem();
    private final SensorHub m_sensorHub = new SensorHub();
    private final PowerDistributionSubsystem m_powerSubsystem = new PowerDistributionSubsystem();

    // 2. Controllers
    private final XboxController m_driverController = new XboxController(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Start automatic data logging to USB/Internal Storage
        DataLogManager.start();
        // Record driver station data (joystick inputs, etc.)
        DriverStation.startDataLog(DataLogManager.getLog());

        // Set Default Command: Manual Motor Control
        m_motorSubsystem.setDefaultCommand(
            new MotorTestCommand(
                m_motorSubsystem, 
                () -> -m_driverController.getLeftY(), 
                () -> 0.5 + (m_driverController.getRightTriggerAxis() * 0.5)
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        // Add System buttons to Dashboard
        SmartDashboard.putData("MagicBox/SYSTEM/RE-INIT HARDWARE", 
            new InstantCommand(m_motorSubsystem::reinitializeHardware).ignoringDisable(true));
            
        SmartDashboard.putData("MagicBox/SYSTEM/RESET POWER LOGS", 
            new InstantCommand(m_powerSubsystem::resetEnergy).ignoringDisable(true));
    }

    /**
     * Map buttons to commands.
     */
    private void configureButtonBindings() {
        // Hold 'A' Button to enter PID Tuning Mode
        new JoystickButton(m_driverController, XboxController.Button.kA.value)
            .whileTrue(new PIDTuningCommand(m_motorSubsystem));

        // Press 'B' Button to emergency stop the motor
        new JoystickButton(m_driverController, XboxController.Button.kB.value)
            .onTrue(new InstantCommand(m_motorSubsystem::stop, m_motorSubsystem));

        // Press 'Y' Button to clear PDH faults
        new JoystickButton(m_driverController, XboxController.Button.kY.value)
            .onTrue(new InstantCommand(m_powerSubsystem::clearFaults));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}