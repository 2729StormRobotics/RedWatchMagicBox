// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.CommandBase;

// Creates a command for the gripper subsystem to run motors to remove an object
// from the gripper
public class EjectItem extends CommandBase {

  // Prepares a variable for the gripper
  private final Gripper m_chompychompy;

  /** Creates a new EjectItem. */
  public EjectItem(Gripper gripper) {
    // Appoints the gripper parameter (a subsystem) to the gripper variable
    m_chompychompy = gripper;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Runs the gripper motors at eject speed
    m_chompychompy.ejectGripper();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops gripper motors
    m_chompychompy.stopGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
