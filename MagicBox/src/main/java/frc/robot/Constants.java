// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Controller Ports
    public static final int kSparkControllerPort = 1;
    public static final int kTalonControllerPort = 2;

    public static final boolean kLeftReversedDefault = true;
    public static final boolean kRightReversedDefault = !kLeftReversedDefault;
    public static final String kShuffleboardTab = "Testing";
    public static final int STALL_LIMIT = 45;
    public static final int kCurrentLimit = 60;

    // Spark Maxes
    public static final int LEFT_SPARK_ID = 2;
    public static final int RIGHT_SPARK_ID = 1;
    public static final String kShuffleboardTabSpark = "Spark Maxes";

    // Talons
    public static final int kLeftTalonPort = 3;
    public static final int kRightTalonPort = 4;
    public static final String kShuffleboardTabTalon = "Talons";

    // Pneumatics
    public static final PneumaticsModuleType PneumaticType = PneumaticsModuleType.CTREPCM;
    public static final int solMotorPort = 4;
}
