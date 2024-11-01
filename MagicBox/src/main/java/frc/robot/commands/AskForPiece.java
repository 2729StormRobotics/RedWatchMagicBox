// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorDetection;
import frc.robot.subsystems.NewLights;

public class AskForPiece extends CommandBase {
  /** Creates a new AskForPiece. */
  public NewLights m_NewLights;
  public ColorDetection m_color;
  int count = 0;
  private boolean cube;
  public AskForPiece(NewLights lights,ColorDetection color, boolean val) {
    m_NewLights = lights;
    m_color = color;
    cube = val;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_NewLights, m_color);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    m_color.detect = false;
    if (cube){
      m_NewLights.askForCube();
    }
    else if(!cube){
      m_NewLights.askForCone();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    count = count+1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_color.detect = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return (count >= 250);
  }
}
