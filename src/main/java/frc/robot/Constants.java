// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Util.Setpoints;
import frc.robot.Util.Setpoints.GameState;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This class should not be
 * used for any other purpose. All constants should be declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class RobotConstants {

        public static final boolean kIsTuningMode = true;
        public static final boolean kIsDriveTuningMode = false;
        public static final boolean kIsArmTuningMode = true;
        public static final boolean kIsIntakeTuningMode = false;
        public static final boolean kIsStageTuningMode = false;
        public static final boolean kIsShooterTuningMode = false;
        public static final boolean kIsAutoAimTuningMode = false;
    
        // Shooter and Arm Setpoints
        public static final Setpoints STOWED =    new Setpoints(1.0, 0.4, 0.0, 0.0, GameState.STOWED);
        public static final Setpoints INTAKE =    new Setpoints(1.0, 2.0, 0.0, 0.0, GameState.INTAKE);
        public static final Setpoints SUBWOOFER = new Setpoints(1.0, 1.0, 2400.0,2100.0,  GameState.SUBWOOFER);
        public static final Setpoints AMP =       new Setpoints(95.0, 0.4, 2400.0,2400.0,  GameState.AMP);
        public static final Setpoints PODIUM =    new Setpoints(23.0, 0.4, 3000.0,3000.0,  GameState.PODIUM);
        public static final Setpoints WING =      new Setpoints(30.0, 0.4, 4200.0,3600.0,  GameState.WING);
        public static final Setpoints PREPCLIMB = new Setpoints(0.0, 0.4, 0.0,0.0,  GameState.PREPCLIMB);
        public static final Setpoints CLIMB =     new Setpoints(88.0, 0.4, 0.0,0.0,  GameState.CLIMB);
        public static final Setpoints TRAP =      new Setpoints(0.0, 0.4, 1200.0,1200.0,  GameState.TRAP);
        public static final Setpoints LOOKUP =    new Setpoints(0.0, 0.4, 1200.0,1500.0,  GameState.LOOKUP);
        public static final Setpoints FEED =      new Setpoints(7.0, 2, 1500,1500.0,  GameState.FEED);

        public static final double autoShootMaxDist = 4.0;
    }
    
    public static final class CanConstants {

        /* PDH Power Ports
        0. Front Left Drive
        1. Front Left Steer
        2. Left Shooter
        3. Back Left Steer
        4. Back Left Drive
        5. Mini Power Module
        6. Left Arm Motor
        7. N/A
        8. Intake
        9. N/A
        10. N/A
        11. Right Arm
        12. Candle
        13. N/A
        14. Back Right Steer
        15. Back Right Drive
        16. Front Right Drive
        17. Front Right Steer
        18. Right Shooter Motor
        19. Stage Motor
        20. Future Limelight
        21. VRM
        22. RIO
        23. N/A
         * 
         * 
         */

        // Drivebase CAN IDs are 1 -> 13
        // See generated/TunerConstants.java

        // Shooter CAN IDs
        public static final int ID_ShooterLeft = 25;
        public static final int ID_ShooterRight = 26;

        // Intake CAN IDs
        public static final int ID_IntakeMotor = 21;

        // Stage CAN IDs
        public static final int ID_StageMotor = 27;

        // Arm CAN IDs
        public static final int ID_ArmLeader = 23;
        public static final int ID_ArmFollower = 24;
        public static final int LED_CANDLE = 14;
    }

    public static final class DIOConstants {
        public static final int kStageBeamBreak = 0;
    }

    public static final class StageConstants {

        public static final double kIntakeSpeed = 0.6;
        public static final double kFeedToShooterSpeed = 1.0;
        public static final double kFeedToAmpSpeed = 0.7;
        public static final double kFeedToTrapSpeed = 0.5;

        public static final double kFeedToShooterTime = 0.5;
        public static final double kFeedToAmpTime = 1.0;
        public static final double kFeedToTrapTime = 5.0;
    }

    public static final class IntakeConstants {

        public static final double kIntakeSpeed = 0.5;
        public static final double kEjectSpeed = -0.3;
    }

    public static final class ShooterConstants {

        // Shooter speeds are set in the individual position Setpoints at the top of this file
        
        public static final double kShooterTolerance = 1.0;
        public static final double kTimeToShoot = .13; //Time it takes before note leave the shooter
        public static final double kShooterIdleSpeed = 15.0;

    }

    public static final class ArmConstants {
/*
 * Calibrating the Arm Angle
 * 
 * - Turn the robot off and push the Arm against its hard stop in the STOWED position <br>
 * - Turn the robot on and connect to the robot (Do not enable) <br>
 * - Open Shuffleboard and find the box with the value for "Arm Angle Uncorrected" <br>
 * - Copy this value into the constant named kARM_STARTING_OFFSET in the "ArmConstants" section of Constants.java <br>
 * - The value should be > 0.0 (i.e. not negative). If it is 0.0 or less, then there is an encoder issue.
 * - The value should be between 30-120 degrees. Anything over 200 likely means the encoder zero point is not in the right spot)<br>
 * - You want to make sure the value you choose is just slightly smaller than the lowest number that appears in "Arm Angle Uncorrected".
 * - Otherwise you may get negative readings for the Arm Current Angle, and error checking may prevent the Arm motors from moving.
 * - Move the Arm to the horizontal position and again check the value in the "Arm Angle Uncorrected" box. <br>
 * - Copy this value into the constant named kARM_HORIZONTAL_OFFSET. (It should be between 90-160 degrees).<br>
 * - Save the file and deploy code to the robot. Make sure the Arm starts in the STOWED position. <br>
 * - If the value for Arm Current Angle is a negative value do not enable, and try to do the offsets again <br>
 * - If it is still negative, then there is an issue with the encoder. <br>
 * 
 * 
 */
        // Observed Arm Offsets
        // Measured against the hardstop when the Arm is in the STOWED position
        public static final double kARM_STARTING_OFFSET = 0;
        // Measured when the Arm is exactly horizontal
        public static final double kARM_HORIZONTAL_OFFSET = 11;

        // Feedforward Gains
        public static final double kS = 0.5;  // The Static Gain, in volts
        public static final double kG = 0;  // The Gravity Gain, in volts //.25
        public static final double kV = 3.56;  // The Velocity Gain, in volt seconds per radian
        public static final double kA = 0;  // The acceleration gain, in volt seconds^2 per radian

        // PID Control Gains
        public static final double kP = 1; // P Gain - Volts
        public static final double kI = 0.00;  // I Gain - Volts
        public static final double kD = 0;  // D Gain - Volts


        // Profiled PID Constants
        public static final double kArm_Cruise = 4.0;           // Radians per second
        public static final double kArm_Acceleration = 10.0;   // Radians per second^2

        public static final double kDuty_Cycle_Min = 1.0/1025.0;
        public static final double kDuty_Cycle_Max = 1024.0/1025.0;

        // Motor Neutral dead-band : Range 0.001 -> 0.25
        public static final double kNeutral_Deadband = 0.005;

	}

    public static class LimelightConstants {
        public static final String kCameraName = "limelight-middle";
    }

    public static class SwerveConstants {
        public static final double kSwerveDriveSupplyCurrentLimit = 30.00;
        public static final boolean kSwerveDriveSupplyCurrentLimitEnable = true;
        public static final double kSwerveDriveSupplyCurrentThreshold = 90.00;
        public static final double kSwerveDriveSupplyTimeThreshold = 0.01;
    }

    public static class ControllerConstants {
        public static final double triggerThreashold = 0.4;

    }

    public static class FieldConstants {
        public static final Pose2d BLUE_SPEAKER = new Pose2d(Units.inchesToMeters(-1.5+12), Units.inchesToMeters(218.42), new Rotation2d(0));
        public static final Pose2d RED_SPEAKER = new Pose2d(Units.inchesToMeters(652.73-12), Units.inchesToMeters(218.42), new Rotation2d(Math.PI));
        public static final double BLUE_AUTO_PENALTY_LINE = 8.6; // X distance from origin to center of the robot almost fully crossing the midline
        public static final double RED_AUTO_PENALTY_LINE = 8; // X distance from origin to center of the robot almost fully crossing the midline

        public static final double NOTE_DIAMETER = 14; // Outer diameter of note

    }

    // Initial max is true top speed
    public static final double maxSpeed = TunerConstants.kSpeedAt12VoltsMps;
    // Reduction in speed from Max Speed, 0.5 = 50%
    public static final double halfSpeed = 0.5;
    // Reduction in speed from Max Speed, 0.25 = 25%
    public static final double quarterSpeed = 0.25;
    // .75 rotation per second max angular velocity. Adjust for max turning rate
    // speed.
    public static final double maxAngularRate = Math.PI * 1.5;
    // .75 rotation per second max angular velocity. Adjust for max turning rate
    // speed.
    public static final double halfAngularRate = Math.PI * 1.0;
    // .75 rotation per second max angular velocity. Adjust for max turning rate
    // speed.
    public static final double quarterAngularRate = Math.PI * 0.5;

    public static final double robotAtAngleTolerance = 2;

}
