package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    // Port 0 is usually the first joystick plugged in.
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
        // Using getX() (Left/Right stick movement) for turret rotation.
        // You could also use m_driverJoystick.getTwist() if you prefer twisting the stick.
        m_turret.setDefaultCommand(
            new RunCommand(
                () -> {
                    double stickInput = m_driverJoystick.getX();
                    // Deadband to prevent drift
                    if (Math.abs(stickInput) < 0.1) {
                        m_turret.stop();
                    } else {
                        // Scale down for fine control (50% speed)
                        m_turret.setPercentOutput(stickInput * 0.5);
                    }
                },
                m_turret
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        // Dashboard buttons
        // Note: m_powerSubsystem was missing from your snippet, so I commented this out to prevent errors.
        // SmartDashboard.putData("MagicBox/SYSTEM/RESET POWER LOGS", 
        //    new InstantCommand(m_powerSubsystem::resetEnergy).ignoringDisable(true));
        
        SmartDashboard.putData("Turret/SYSTEM/RESET TO ABSOLUTE", 
            new InstantCommand(m_turret::resetToAbsolute).ignoringDisable(true));
    }

    /**
     * Map buttons to commands.
     */
    private void configureButtonBindings() {
        // ========== TURRET CONTROLS ==========

        // Button 1 (Trigger): Stop Turret (Safety)
        m_driverJoystick.button(1)
            .onTrue(new InstantCommand(m_turret::stop, m_turret));

        // Button 2 (Side Thumb Button): Reset to Absolute Position (CRT Reset)
        m_driverJoystick.button(2)
            .onTrue(new InstantCommand(m_turret::resetToAbsolute, m_turret));

        // Button 3 (Bottom Left): Decrease target angle by 15 degrees
        m_driverJoystick.button(3)
            .onTrue(new InstantCommand(() -> m_turret.setAngle(m_turret.getCurrentAngle() - 15.0), m_turret));

        // Button 4 (Bottom Right): Increase target angle by 15 degrees
        m_driverJoystick.button(4)
            .onTrue(new InstantCommand(() -> m_turret.setAngle(m_turret.getCurrentAngle() + 15.0), m_turret));

        // Button 5 (Top Left): Aim Left (-90)
        m_driverJoystick.button(5)
            .whileTrue(new InstantCommand(() -> m_turret.setAngle(-90.0), m_turret));

        // Button 6 (Top Right): Aim Right (90)
        m_driverJoystick.button(6)
            .whileTrue(new InstantCommand(() -> m_turret.setAngle(90.0), m_turret));

        // ========== POV (Hat Switch) CONTROLS ==========
        
        // POV Up (0 deg): Aim Forward
        m_driverJoystick.pov(0)
            .whileTrue(new InstantCommand(() -> m_turret.setAngle(0.0), m_turret));

        // POV Right (90 deg): Aim Right
        m_driverJoystick.pov(90)
            .whileTrue(new InstantCommand(() -> m_turret.setAngle(90.0), m_turret));

        // POV Down (180 deg): Aim Back
        m_driverJoystick.pov(180)
            .whileTrue(new InstantCommand(() -> m_turret.setAngle(180.0), m_turret));

        // POV Left (270 deg): Aim Left
        m_driverJoystick.pov(270)
            .whileTrue(new InstantCommand(() -> m_turret.setAngle(-90.0), m_turret));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}