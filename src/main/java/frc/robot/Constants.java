package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    
    /**
     * Controller constants
     */
    public static class Controller {

        public static final int AUX_PORT = 0;
    
    }
    
    /**
     * Button constants
     */
    public static class Button {
    
        public static final int CLIMBER_UP = 0;
        public static final int CLIMBER_DOWN = 0;
        
    }
    
    /**
     * Climber subsystem constants
     */
    public static class ClimberConstants {
    
        public static final int LEFT_TALON = 170;
        public static final int RIGHT_TALON = 171;
    
        public static final int LEFT_LIMIT_SWITCH = 0;
        public static final int RIGHT_LIMIT_SWITCH = 1;
    
        public static final int UPPER_LEFT_LIMIT = 1000000;
        public static final int UPPER_RIGHT_LIMIT = 1000000; 
    
        public static final double SPEED = 0.5;
    
    }    

    public static final class ModuleConstants {
        //The diameter of the wheel on your swerve drive
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.94); 
        //The gear ratio for an SDS MK4 module with speed ratio of L2
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        //Ratio for an SDS MK4 turning motor
        public static final double kTurningMotorGearRatio = 1 / 12.80;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        //The P value for the turning PID loop
        public static final double kPTurning = 0.4; //0.2
    }

    public static final class DriveConstants {

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(22.5);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(22.5);


        // Measured from the center of the robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),    /*FL*/
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),   /*FR*/
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),   /*BL*/
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); /*BR*/

        //CAN IDs For the drive motor spark maxes on the swerve modules
        public static final int kFrontLeftDriveMotorPort = 11;
        public static final int kFrontRightDriveMotorPort = 21;
        public static final int kBackLeftDriveMotorPort = 31;
        public static final int kBackRightDriveMotorPort = 41;

        //CAN IDs for the turning motor spark maxes on the swerve modules
        public static final int kFrontLeftTurningMotorPort = 12;
        public static final int kFrontRightTurningMotorPort = 22;
        public static final int kBackLeftTurningMotorPort = 32;
        public static final int kBackRightTurningMotorPort = 42;

        //CTRE Pigeon 2.0
        public static final int kPigeonPort = 1;

        //A boolean to control the inversion of the direction of the motor gievn a positive value
        //Positive values muest result in a counter-clockwise movment
        public static final boolean kFrontLeftTurningMotorReversed = true;
        public static final boolean kFrontRightTurningMotorReversed = true;
        public static final boolean kBackLeftTurningMotorReversed = true;
        public static final boolean kBackRightTurningMotorReversed = true;

        //A boolean to control the inversion of the direction of the motor gievn a positive value
        //Positive values must result in a forward movement
        public static final boolean kFrontLeftDriveMotorReversed = false;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kBackLeftDriveMotorReversed = false;
        public static final boolean kBackRightDriveMotorReversed = false;

        //The CAN id's for the CANcoder's on the swerve modules
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 10;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 20;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 30;
        public static final int kBackRightDriveAbsoluteEncoderPort = 40;

        //Inversion of the direction of the CANcoder
        //Positive values must result in a counter-clockwise movement
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        //Position must be counter-clockwise from the positive YAw
        public static final boolean kGyroInverted = false; // was false

        //The offset of the CANcoder's position from the zero position (Straight forward)
        //Measure this by rotating all the modules to the forward position and reading the CANcoder's value

        // Offsets changed 1/21/24
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(99.404); // -171.035
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(82.969); // 173.408
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(34.893); // 125.508
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(177.891); // -92.285

        // in m/s, based on MK4 L2 speed of 14.5 ft/s
        public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(14.5);  //14.5
        // Robot turning speed
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 5 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 5;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 5;  // Slowed down for testing
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 5; 
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;
        // The P value of the PID controller used in auto for the X and Y directions
        public static final double kPXController = 1.5; //1.5 before change
        public static final double kPYController = 1.5; //1.5 before change
        // The P value of the PID controller used in auto for the theta (rotation) direction
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        // Port for the driver's controller 
        public static final int kDriverControllerPort = 0;
        public static final int kXboxControllerPort = 1;
        // Axis used for the X, Y, Rotation, and Throttle
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverThrottleAxis = 3;
        // Deadband for the controller
        public static final double kDeadband = 0.2;
        // Button used to enable robot orientation driving
        public static final int kDriverFieldOrientedButtonIdx = 5;
        // Button used to enable slow turning
        public static final int kDriverSlowTurnButtonIdx=1;

    }
}
