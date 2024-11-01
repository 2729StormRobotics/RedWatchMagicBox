// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorDetection;
import frc.robot.subsystems.NewLights;

public class ColorOverride extends CommandBase {
  public NewLights lights;
  public ColorDetection colorDetect;
  /** Creates a new ColorOverride. */
  public ColorOverride(NewLights light, ColorDetection color) {
    lights = light;
    colorDetect = color;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lights, colorDetect);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lights.resetAnim();
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
    return false;
  }
}
