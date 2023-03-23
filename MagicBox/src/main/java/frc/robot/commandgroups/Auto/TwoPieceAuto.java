// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commandgroups.DriveWhileIntake;
import frc.robot.commandgroups.SetupScore;
import frc.robot.Constants.GripperConstants;
import frc.robot.commands.Gripper.EjectItem;
import frc.robot.commands.Gripper.StopGripper;
import frc.robot.commands.MultSubsystem.AutoForwardPID;
import frc.robot.commands.MultSubsystem.TurnInPlacePID;
import frc.robot.commands.Vision.AprilTagMode;
import frc.robot.commands.Vision.VisionAlign;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;
import frc.robot.subsystems.Vision;

import static frc.robot.Constants.pinkArmConstants;
import static frc.robot.Constants.TelescopingConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceAuto extends SequentialCommandGroup {
  /** Creates a new TwoPieceAuto. */
  public TwoPieceAuto(Drivetrain drivetrain, Gripper gripper, TelescopingArm telescopingArm, PivotArm pivotArm, Vision vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BackwardTime(drivetrain, 0.5, true), // 22.5 INCHES FROM THE NODE!!!!!!
      new SetupScore(pivotArm, telescopingArm, pinkArmConstants.kLowAngleCube, TelescopingConstants.LowExtendCube),
      new DriveWhileIntake(drivetrain, gripper, 4.26),
      new WaitCommand(0.5),
      new StopGripper(gripper),
      new BackWhileSetupHighCube(drivetrain, pivotArm, telescopingArm),
      new TurnInPlacePID(-170, drivetrain),
      new AprilTagMode(vision),
      new VisionAlign(drivetrain, vision),
      new AutoForwardPID(0.5, drivetrain),
      new EjectItem(gripper, GripperConstants.kGripperEjectCubeSpeed),
      new WaitCommand(1.5),
      new StopGripper(gripper)
    );
  }
}
