package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Turret;

/**
 * RobotContainer is the heart of the MagicBox.
 * This is where we define our subsystems, commands, and button mappings.
 */
public class RobotContainer {
    private final Turret m_turret = new Turret();

    // 2. Controllers
    // Replaced XboxController with CommandJoystick for Logitech Extreme 3D Pro / x30
    // Port 1 based on your snippet.
    private final CommandJoystick m_driverJoystick = new CommandJoystick(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Start automatic data logging to USB/Internal Storage
        DataLogManager.start();
        // Record driver station data (joystick inputs, etc.)
        DriverStation.startDataLog(DataLogManager.getLog());

        // Set Default Command: Manual Turret Control
        // Uses the factory method from Turret.java
        m_turret.setDefaultCommand(
            m_turret.manualControlCommand(() -> m_driverJoystick.getX())
        );

        // Configure the button bindings
        configureButtonBindings();

        // Dashboard buttons
        SmartDashboard.putData("Turret/SYSTEM/RESET TO ABSOLUTE", 
            m_turret.resetToAbsoluteCommand().ignoringDisable(true));
    }

    /**
     * Map buttons to commands.
     */
    private void configureButtonBindings() {
        // ========== TURRET CONTROLS ==========

        // Button 1 (Trigger): Stop Turret / Return to Manual Control
        // This interrupts any running "RunToAngle" command.
        m_driverJoystick.button(1)
            .onTrue(m_turret.stopCommand());

        // Button 2 (Side Thumb Button): Reset to Absolute Position (CRT Reset)
        m_driverJoystick.button(2)
            .onTrue(m_turret.resetToAbsoluteCommand());

        // Button 5 (Top Left): Aim Left (-90)
        m_driverJoystick.button(5)
            .onTrue(m_turret.runToAngleCommand(-90.0));

        // Button 6 (Top Right): Aim Right (90)
        m_driverJoystick.button(6)
            .onTrue(m_turret.runToAngleCommand(90.0));

        // ========== POV (Hat Switch) CONTROLS ==========
        
        // POV Up (0 deg): Aim Forward
        m_driverJoystick.pov(0)
            .onTrue(m_turret.runToAngleCommand(0.0));

        // POV Right (90 deg): Aim Right
        m_driverJoystick.pov(90)
            .onTrue(m_turret.runToAngleCommand(90.0));

        // POV Down (180 deg): Aim Back
        m_driverJoystick.pov(180)
            .onTrue(m_turret.runToAngleCommand(180.0));

        // POV Left (270 deg): Aim Left
        m_driverJoystick.pov(270)
            .onTrue(m_turret.runToAngleCommand(-90.0));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}