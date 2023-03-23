// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MultiSubsystem.AutoForwardPID;
import frc.robot.commands.MultiSubsystem.TurnInPlacePID;
import frc.robot.commands.Vision.AprilTagMode;
import frc.robot.commands.Vision.VisionAlign;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class B1_A extends SequentialCommandGroup {
  /** Creates a new B1_A. */
  private final Drivetrain m_Drivetrain;
  private final Vision m_vision;
  // private final PivotArm m_PinkArm;
  // private final TelescopingArm m_arm;
  // private final Gripper m_gripper;

  public B1_A(Drivetrain subsystem, Vision vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_Drivetrain = subsystem;
    m_vision = vision;
    // m_PinkArm = pivot;
    // m_arm = arm;
    // m_gripper = gripper;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      // Score
      // new AutoForwardPID(-50 , m_Drivetrain),
      new AprilTagMode(m_vision),
      new AutoForwardPID(-3.1 , m_Drivetrain),
      new TurnInPlacePID(-180, m_Drivetrain),
      new AutoForwardPID(1.5 , m_Drivetrain),
      // new TurnInPlacePID(-6, m_Drivetrain),
      // new AutoForwardPID(0.2 , m_Drivetrain),
      // new TurnInPlacePID(6, m_Drivetrain),
      // new AutoForwardPID(-0.2 , m_Drivetrain),
      new AutoForwardPID(-0.4 , m_Drivetrain),
      new TurnInPlacePID(177, m_Drivetrain),
      new AutoForwardPID(4 , m_Drivetrain),
      new VisionAlign(m_Drivetrain, m_vision),
      new AutoForwardPID(0.5 , m_Drivetrain)


      ); // 125.37, 114.26, -55.46,-59
      
  }
}
