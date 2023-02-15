// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commandgroups.AutoScore;
import frc.robot.commands.PivotArm.armJoint;
import frc.robot.subsystems.TelescopingArm;
import frc.robot.subsystems.PivotArm;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Drivetrain.curvatureDrive;
import frc.robot.commands.AutoBalancing.AutoBalancePID;
import frc.robot.commands.Lights.CheckObjectColor;
import frc.robot.commands.Lights.ChangeLEDColor;
import frc.robot.commands.TelescopingArm.ArmControl;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lights;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Gripper.*;
import static frc.robot.Constants.pinkArmConstants.*;
import static frc.robot.Constants.TelescopingConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Controller
  private final XboxController m_driver = new XboxController(Constants.DrivetrainConstants.kDriverControllerPort);
  private final XboxController m_weapons = new XboxController(Constants.DrivetrainConstants.kWeaponsControllerPort);

  private SlewRateLimiter m_forwardLimiter = new SlewRateLimiter(1); // controls acceleration of forward speed
  private SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(0.5); // controls acceleration of rotational speed

  // Subsystems
  private final Lights m_lights;
  private final Gripper m_gripper;
  private final Drivetrain m_drivetrain;
  private final PivotArm m_PinkArm;
  private final TelescopingArm m_arm;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems Instantiation
    m_gripper = new Gripper();
    m_lights = new Lights();
    m_drivetrain = new Drivetrain();
    m_PinkArm = new PivotArm();
    m_arm = new TelescopingArm();

    // Setting default commands
    m_arm.setDefaultCommand(
      new ArmControl(() -> m_weapons.getLeftY(), m_arm));

    // Control Panel
    new ControlPanel(m_gripper, m_lights);

    // Setting default commands
    m_arm.setDefaultCommand(
      new ArmControl(() -> m_weapons.getLeftY(), m_arm));

    // Lights
    m_lights.setDefaultCommand(new CheckObjectColor(m_gripper, m_lights));

    // sets the drivetrain default command to curvatureDrive, with the slewratelimiters
    // Left Joystick: forwards/backward, Right Joystick: turn in place left/right
    m_drivetrain.setDefaultCommand(
    new curvatureDrive(
      () -> Math.copySign(Constants.DrivetrainConstants.kS, m_driver.getLeftY())
      + m_forwardLimiter.calculate(m_driver.getLeftY() / Drivetrain.speedLimiter), 
      () -> Math.copySign(Constants.DrivetrainConstants.kS, m_driver.getRightX()) 
      + m_rotationLimiter.calculate(m_driver.getRightX() / Drivetrain.rotationLimiter),
      () -> true, m_drivetrain));
    
    // // Pink Arm
    m_PinkArm.setDefaultCommand(
      new armJoint(() -> m_weapons.getRightY(), m_PinkArm)
    );

    // Configure the button bindings

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {    
    new JoystickButton(m_weapons, Button.kBack.value).onTrue(new IntakeItem(m_gripper));
    new JoystickButton(m_weapons, Button.kStart.value).onTrue(new EjectItem(m_gripper));
    
    new JoystickButton(m_weapons, Button.kLeftStick.value).onTrue(new ChangeLEDColor(m_lights, 255, 255, 0));
    new JoystickButton(m_weapons, Button.kRightStick.value).onTrue(new ChangeLEDColor(m_lights, 101, 15, 140));

    // new JoystickButton(m_weapons, Button.kY.value).onTrue(new ExtendVal( TelescopingConstants.HighExtendCube, m_arm));
    // new JoystickButton(m_weapons, Button.kX.value).onTrue(new ExtendVal( TelescopingConstants.MidExtendCube, m_arm));
    // new JoystickButton(m_weapons, Button.kA.value).onTrue(new ExtendVal( TelescopingConstants.LowStop , m_arm));

    new JoystickButton(m_weapons, Button.kLeftBumper.value).onTrue(new AutoScore(m_PinkArm, m_arm, m_gripper, kHighAngleCone, HighExtendCone));
    new JoystickButton(m_weapons, (int) m_weapons.getLeftTriggerAxis()).onTrue(new AutoScore(m_PinkArm, m_arm, m_gripper, kMidAngleCone, MidExtendCone));

    new JoystickButton(m_weapons, Button.kRightBumper.value).onTrue(new AutoScore(m_PinkArm, m_arm, m_gripper, kHighAngleCube, HighExtendCube));
    new JoystickButton(m_weapons, (int) m_weapons.getRightTriggerAxis()).onTrue(new AutoScore(m_PinkArm, m_arm, m_gripper, kMidAngleCube, MidExtendCube));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new AutoBalancePID(m_drivetrain);
    return new AutoScore(null, null, m_gripper, kAnglePerRevolution, 10);
  }
}