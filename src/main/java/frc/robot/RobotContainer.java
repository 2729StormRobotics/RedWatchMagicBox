package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.MotorTestCommand;
import frc.robot.commands.PIDTuningCommand;
import frc.robot.subsystems.MotorSubsystem;
import frc.robot.subsystems.SensorHub;
import frc.robot.subsystems.PowerDistributionSubsystem;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * RobotContainer is the heart of the MagicBox.
 * This is where we define our subsystems, commands, and button mappings.
 */
public class RobotContainer {
    // 1. Subsystems
    private final MotorSubsystem m_motorSubsystem = new MotorSubsystem();
    private final SensorHub m_sensorHub = new SensorHub();
    private final PowerDistributionSubsystem m_powerSubsystem = new PowerDistributionSubsystem();
    private final Turret m_turret = new Turret();

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
        // m_motorSubsystem.setDefaultCommand(
        //     new MotorTestCommand(
        //         m_motorSubsystem, 
        //         () -> -m_driverController.getLeftY(), 
        //         () -> 0.5 + (m_driverController.getRightTriggerAxis() * 0.5)
        //     )
        // );

        // Set Default Command: Manual Turret Control (Right Stick X for manual rotation)
        m_turret.setDefaultCommand(
            new RunCommand(
                () -> {
                    double rightStickX = m_driverController.getRightX();
                    // Deadband to prevent drift
                    if (Math.abs(rightStickX) < 0.1) {
                        m_turret.stop();
                    } else {
                        // Scale down for fine control
                        m_turret.setPercentOutput(rightStickX * 0.5);
                    }
                },
                m_turret
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        // Add System buttons to Dashboard
        // SmartDashboard.putData("MagicBox/SYSTEM/RE-INIT HARDWARE", 
        //     new InstantCommand(m_motorSubsystem::reinitializeHardware).ignoringDisable(true));
            
        SmartDashboard.putData("MagicBox/SYSTEM/RESET POWER LOGS", 
            new InstantCommand(m_powerSubsystem::resetEnergy).ignoringDisable(true));
        
        SmartDashboard.putData("Turret/SYSTEM/RESET TO ABSOLUTE", 
            new InstantCommand(m_turret::resetToAbsolute).ignoringDisable(true));
    }

    /**
     * Map buttons to commands.
     */
    private void configureButtonBindings() {
        // Hold 'A' Button to enter PID Tuning Mode
        // new JoystickButton(m_driverController, XboxController.Button.kA.value)
        //     .whileTrue(new PIDTuningCommand(m_motorSubsystem));

        // // Press 'B' Button to emergency stop the motor
        // new JoystickButton(m_driverController, XboxController.Button.kB.value)
        //     .onTrue(new InstantCommand(m_motorSubsystem::stop, m_motorSubsystem));

        // // Press 'Y' Button to clear PDH faults
        // new JoystickButton(m_driverController, XboxController.Button.kY.value)
        //     .onTrue(new InstantCommand(m_powerSubsystem::clearFaults));

        // ========== TURRET CONTROLS ==========
        
        // Press 'X' Button to reset turret to absolute position (CRT reset)
        new JoystickButton(m_driverController, XboxController.Button.kX.value)
            .onTrue(new InstantCommand(m_turret::resetToAbsolute, m_turret));

        // Press 'Start' Button to stop turret
        new JoystickButton(m_driverController, XboxController.Button.kStart.value)
            .onTrue(new InstantCommand(m_turret::stop, m_turret));

        // D-Pad Up (0 degrees): Aim turret to 0 degrees (forward)
        new POVButton(m_driverController, 0)
            .onTrue(new InstantCommand(() -> m_turret.setAngle(0.0), m_turret));

        // D-Pad Left (270 degrees): Aim turret to -90 degrees (left)
        new POVButton(m_driverController, 270)
            .onTrue(new InstantCommand(() -> m_turret.setAngle(-90.0), m_turret));

        // D-Pad Right (90 degrees): Aim turret to +90 degrees (right)
        new POVButton(m_driverController, 90)
            .onTrue(new InstantCommand(() -> m_turret.setAngle(90.0), m_turret));

        // D-Pad Down (180 degrees): Aim turret to 180 degrees (back)
        new POVButton(m_driverController, 180)
            .onTrue(new InstantCommand(() -> m_turret.setAngle(180.0), m_turret));

        // Left Bumper: Decrease target angle by 15 degrees
        new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
            .onTrue(new InstantCommand(() -> m_turret.setAngle(m_turret.getTargetAngle() - 15.0), m_turret));

        // Right Bumper: Increase target angle by 15 degrees
        new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
            .onTrue(new InstantCommand(() -> m_turret.setAngle(m_turret.getTargetAngle() + 15.0), m_turret));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}