// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commandgroups.SetupScore;
import frc.robot.commands.MultiSubsystem.AutoForwardPID;
import frc.robot.Constants.TelescopingConstants;
import frc.robot.Constants.pinkArmConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BackWhileSetupHighCone extends ParallelCommandGroup {
  /** Creates a new BackWhileSetupHighCube. */
  public BackWhileSetupHighCone(Drivetrain drivetrain, PivotArm pivotArm, TelescopingArm telescopingArm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoForwardPID(-3.76, drivetrain),
    new SetupScore(pivotArm, telescopingArm, pinkArmConstants.kHighAngleCone, TelescopingConstants.HighExtendCone)

    );
  }
}
