// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Hood;

/**
 * RobotContainer is the heart of the MagicBox.
 * This is where we define our subsystems, commands, and button mappings.
 */
public class RobotContainer {
    // 1. Subsystems
    private final Flywheel m_flywheel = new Flywheel();
    // private final Turret m_turret = new Turret();
    private final Hood m_Hood = new Hood();

    // 2. Controllers
    // Joystick on Port 1
    private final CommandJoystick m_driverJoystick = new CommandJoystick(1);
    
    // Xbox Controller on Port 2
    // Using CommandXboxController for easier button binding syntax
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        //😉😎
        // Start automatic data logging to USB/Internal Storage
        DataLogManager.start();
        // Record driver station data (joystick inputs, etc.)
        DriverStation.startDataLog(DataLogManager.getLog());

        // Configure the button bindings
        configureButtonBindings();
    }


    /**
     * Map buttons to commands.
     */
    private void configureButtonBindings() {
        // ========== FLYWHEEL CONTROLS ==========

        // When the 'X' button is held, run the flywheel at 50 RPS.
        // When released, the flywheel will stop.
        m_operatorController.x()
            .whileTrue(m_flywheel.runVelocityCommand(350))
            .onFalse(m_flywheel.stopCommand());

        // Press 'B' button to stop the flywheel immediately
        m_operatorController.b()
            .onTrue(m_flywheel.stopCommand());

        m_operatorController.a()
            .onTrue(m_Hood.runPositionCommand(2).andThen(m_Hood.stopCommand()));

        m_operatorController.y()
            .onTrue(m_Hood.runPositionCommand(37).andThen(m_Hood.stopCommand()));

        
        m_operatorController.rightBumper()
            .onTrue(m_Hood.runPositionCommand((19+(m_operatorController.getRightY()*18))));
        m_Hood.setDefaultCommand(m_Hood.runPositionCommandConstant(m_operatorController));
        // SmartDashboard.putNumber("Hood/leftx", (m_operatorController.getLeftX()));
        // SmartDashboard.putNumber("Hood/ly", (m_operatorController.getLeftY()));
        // SmartDashboard.putNumber("Hood/rx", (m_operatorController.getRightX()));
        // SmartDashboard.putNumber("Hood/ry", (m_operatorController.getRightY()));
        // SmartDashboard.putNumber("Hood/rt", (m_operatorController.getRightTriggerAxis()));
            // .onFalse(m_Hood.stop());
        


        
    }
    //     // ========== TURRET CONTROLS (Commented Out) ==========

        
    //     m_driverJoystick.button(1)
    //         .onTrue(m_turret.stopCommand());

    //     m_driverJoystick.pov(90)
    //         .onTrue(m_turret.runToAngleCommand(90));
    //     m_driverJoystick.pov(0)
    //         .onTrue(m_turret.runToAngleCommand(0));
    //     m_driverJoystick.pov(-90)
    //         .onTrue(m_turret.runToAngleCommand(-90));
    //     m_driverJoystick.pov(180)
    //         .onTrue(m_turret.runToAngleCommand(180));     
    // }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}