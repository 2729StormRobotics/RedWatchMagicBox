// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


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
    public static final String kShuffleboardTab = "Control Panel";
    public static final int STALL_LIMIT = 45;
    public static final int kCurrentLimit = 60;

    // Spark Maxes
    public static final int LEFT_SPARK_ID = 2;
    public static final int RIGHT_SPARK_ID = 1;
    // public static final String kShuffleboardTabSpark = "Spark Maxes";

    // Talons
    public static final int kLeftTalonPort = 3;
    public static final int kRightTalonPort = 4;
    // public static final String kShuffleboardTabTalon = "Talons";

    // Pneumatics
    public static final PneumaticsModuleType PneumaticType = PneumaticsModuleType.CTREPCM;
    public static final int solMotorPort = 4;

    public static class LightConstants {
        public static final int kBlinkinDriverPort = 4; //TODO: Find a port for this
        public static final int kPhoenixDriverPort = 11;
		public static final double kDisabled = 0.0; //TODO: Find what color we want for this and its value
		public static final double kLightsOff = 0.99;
        public static final double kRedBall = 0.67;
        public static final double kBlueBall = 0.87;
        public static final double kPurpleCube = 0.91;
        public static final double kYellowCone = 0.67;


    }

    public static class ControllerConstants {
        public static final int kSparkControllerPort = 1;
        public static final int kTalonControllerPort = 2;
    }

    public static class ControlPanelConstants {
        public static final String kShuffleboardTab = "Magic Panel";
    }

    public static class DriveSpark {
        public static final boolean kLeftReversedDefault = true;
        public static final boolean kRightReversedDefault = !kLeftReversedDefault;
        public static final int STALL_LIMIT = 45;
        public static final int kCurrentLimit = 60;

        // Spark Maxes
        public static final int LEFT_SPARK_ID = 1;
        public static final int RIGHT_SPARK_ID = 7;
        public static final int maxSparkRPM = 5500;
        // public static final String kShuffleboardTabSpark = "Spark Maxes";
    }

    public static class DriveTalon {
        // Talons
        public static final int kLeftTalonPort = 11;
        public static final int kRightTalonPort = 10;
        // public static final String kShuffleboardTabTalon = "Talons";         3
    }

    public static class PneumaticSolenoid {
        // Pneumatics
        public static final PneumaticsModuleType PneumaticType = PneumaticsModuleType.CTREPCM;
        public static final int solMotorPort = 1;
        public static final int solMotorPort2 = 6;
    }

    public static class BeambreakConstants {
        public static final int BeambreakPort = 1;
    }

    public static class GripperConstants {
        // Most likely only be using one motor, but written code for 2 in case.
        public static final int kGripperLeftMotor = 1;
        public static final int kGripperRightMotor = 2;
        // Variable assigned values can change depending on what is needed for the robot.
        public static final double kGripperIntakeMotorSpeed = 0.20;   
        public static final double kGripperEjectMotorSpeed = -0.20;
        
    }

    // Figure Out Constants for Wrist
    public static class WristConstants {
        public static final int WRIST_CAN_ID = 41;
 
        public static final double WRIST_P = 0.015;
        public static final double WRIST_I = 0.0;
        public static final double WRIST_D = 0.0;

        public static final float WRIST_SOFT_LIMIT = 0.0f;

        public static final double WRIST_GEAR_RATIO = 133.33333;

        public static final double WRIST_DEGREES_PER_MOTOR_ROTATION = (360 / WRIST_GEAR_RATIO);

        public static final float WRIST_IN_SOFT_LIMIT = -60f;
        public static final float WRIST_OUT_SOFT_LIMIT = 111f;

        public static final float WRIST_ELEVATOR_OUT_SOFT_LIMIT = 270;

        public static final double WRIST_MAX_TEST_PERCENT_OUTPUT = 0.15;

        public static final double MAX_WRIST_GRAVITY_FF = 0.0475;

        public static final double WRIST_FLOOR_MIN_INCH_DISTANCE = 12.1;
        public static final double WRIST_FLOOR_MAX_INCH_DISTANCE = 20.6;

        public static final boolean WRIST_IS_INVERTED = true;

        public static final double WRIST_SET_POS_CONVERSION_FACTOR = 1.6;
    }

    // Figure Out Constants!!!
    public static class WristAngleConstants {
        public static final double WRIST_CONE_HIGH_ANGLE = 55;
        public static final double WRIST_CONE_MID_ANGLE = 49;

        public static final double WRIST_CUBE_HIGH_ANGLE = 55;
        public static final double WRIST_CUBE_MID_ANGLE = 49;

        public static final double WRIST_START_POSITION = 111;

        public static final double WRIST_SUBSTATION_ANGLE = -53;

        public static final double WRIST_SINGLE_SUBSTATION_ANGLE = 78;

        public static final double WRIST_DRIVE_ANGLE = 108;

        public static final double WRIST_INTAKE_ANGLE = -44.8;

        public static final double WRIST_AUTO_INTAKE_ANGLE = -35;

        public static final double WRIST_HYBRID_ANGLE = 111;
    
        
    }
    public static final class IntakeConstants {
        public static final int kIntakePiston1 = 6;
        public static final Value kIntakeRaiseValue = Value.kForward;
        public static final Value kIntakeLowerValue = Value.kReverse;
    }
    public static class Elevator {
        public static final int kElevatorLeftMotorId = 9;
        public static final int kElevatorRightMotorId = 10;
    
        public static final double kP = 0.15;
        public static final double kI = 0;
        public static final double kD = 0.0;
        public static final double kIZone = 5.0;
        public static final double kG = 0.5;
    
        public static final double kMaxVelocity = 65;
        public static final double kMaxAcceleration = 200;
    
        public static final int kMaxCurrent = 40;
        public static final double kMaxPowerUp = 0.1;
        public static final double kMaxPowerDown = 0.1;
    
        public static final double kStowHeight = 0.0;
        public static final double kL2Height = 9.0;
        public static final double kL3Height = 25.14;
        public static final double kL4Height = 52.0;
        public static final double kMaxHeight = 56.2;
        public static final double kGroundAlgaeHeight = 0.0;
        public static final double kScoreAlgaeHeight = 0.0;
        public static final double kLowAlgaeHeight = 24.8;
        public static final double kHighAlgaeHeight = 42.5;
    }
    public static class Vortex {
        public static final int vortexCanId = 1;
    }
}
