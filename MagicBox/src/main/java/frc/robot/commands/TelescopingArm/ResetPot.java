// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TelescopingArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopingArm;

public class ResetPot extends CommandBase {
  /** Creates a new ResetPot. */
  private TelescopingArm m_pot;
  public ResetPot(TelescopingArm pot) {
    m_pot = pot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    m_pot.offset = m_pot.pot.get()*50;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    m_pot.offset = m_pot.pot.get()*50;
    return true;
  }
}