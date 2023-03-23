// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PivotArm.PivotPID;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;
import static frc.robot.Constants.pinkArmConstants.*;
import static frc.robot.Constants.TelescopingConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetupConeHigh extends SequentialCommandGroup {
  private final TelescopingArm m_TelescopingArm;
  private final PivotArm m_PivotArm;
  /** Creates a new SetupCubeHigh. */
  public SetupConeHigh(TelescopingArm telescopingArm, PivotArm pivotArm) {
    m_TelescopingArm = telescopingArm;
    m_PivotArm = pivotArm;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetupScore(m_PivotArm, m_TelescopingArm, kHighAngleConeIntermediate, HighExtendCone),
      new PivotPID(pivotArm, kHighAngleCone)    
);
  }
}
