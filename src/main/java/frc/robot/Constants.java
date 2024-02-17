package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    
    public static class IntakeConstants {
        public static final int SPARK_MAX_ID = 50;
        
        public static final int LIMIT_SWITCH_ID = 1;

        public static final int UPPER_ENCODER_LIMIT = 0;
        public static final int LOWER_ENCODER_LIMIT = -123;

        public static final boolean MOTOR_INVERTED = true;

        public static final double SPEED = .2;
   
    }

    public static final class RollerConstants {
        public static final int VICTOR_ID = 51;

        public static final int DISTANCE_SENSOR_ID = 0;

        public static final boolean MOTOR_INVERTED = true;
        
        public static final double SPEED = 1;
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
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.94); 
        //The gear ratio for an SDS MK4 module with speed ratio of L2
        public static final double DRIVE_MOTOR_GEAR_RATIO = 1 / 6.75;
        //Ratio for an SDS MK4 turning motor
        public static final double TURNING_MOTOR_GEAR_RATIO = 1 / 12.80;
        public static final double DRIVE_ENCODER_ROT_2_METER = WHEEL_DIAMETER_METERS * Math.PI * WHEEL_DIAMETER_METERS;
        public static final double TURNING_ENCODER_ROT_2_RAD = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;
        public static final double DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;
        public static final double TURNING_ENCODER_RPM_2_METER_PER_SEC = TURNING_ENCODER_ROT_2_RAD / 60;
        //The P value for the turning PID loop
        public static final double P_TURNING = .35; //0.2
    }

    public static final class ShooterConstants {

        //CAN IDs For the Shooter motor spark maxes
        public static final int LEFT_SPARK_ID = 52;
        public static final int RIGHT_SPARK_ID = 53;

        public static final double SPEAKER_SPEED = 1; //rate it spins
        public static final double COOLDOWN_TIME = 0.3; //cooldown time in seconds

    }

    public static final class DriveConstants {

        // Distance between right and left wheels
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.5);
        // Distance between front and back wheels
        public static final double WHEEL_BASE = Units.inchesToMeters(22.5);


        // Measured from the center of the robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),    /*FL*/
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),   /*FR*/
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),   /*BL*/
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); /*BR*/

        //CAN IDs For the drive motor spark maxes on the swerve modules
        public static final int FRONT_LEFT_DRIVE_SPARK_ID = 11;
        public static final int FRONT_RIGHT_DRIVE_SPARK_ID = 21;
        public static final int BACK_LEFT_DRIVE_SPARK_ID = 31;
        public static final int BACK_RIGHT_DRIVE_SPARK_ID = 41;

        //CAN IDs for the turning motor spark maxes on the swerve modules
        public static final int FRONT_LEFT_TURNING_SPARK_ID = 12;
        public static final int FRONT_RIGHT_TURNING_SPARK_ID = 22;
        public static final int BACK_LEFT_TURNING_SPARK_ID = 32;
        public static final int BACK_RIGHT_TURNING_SPARK_ID = 42;

        //CTRE Pigeon 2.0
        public static final int PIGEON_ID = 1;

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
        public static final boolean kGyroInverted = false; 

        //The offset of the CANcoder's position from the zero position (Straight forward)
        //Measure this by rotating all the modules to the forward position and reading the CANcoder's value
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(99.404); // 361.143
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(82.969); //1799.209
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(34.893); // 1440.176
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Units.degreesToRadians(177.891); //3240.088

        // in m/s, based on MK4 L2 speed of 14.5 ft/s
        public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(14.5);  //14.5
        // Robot turning speed
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 5 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;  // Slowed down for testing
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2.2; 
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2.2;
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
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int AUX_CONTROLLER_PORT = 1;
        public static final int DRIVER_STATION_PORT = 3;
        // Axis used for the X, Y, Rotation, and Throttle
        public static final int DRIVER_Y_AXIS = 1;
        public static final int DRIVER_X_AXIS = 0;
        public static final int DRIVER_ROT_AXIS = 4;
        public static final int DRIVER_THROTTLE_AXIS = 3;
        // Deadband for the controller
        public static final double DEADBAND = 0.2;
        // Button used to enable robot orientation driving
        public static final int FIELD_ORIENTATED_BUTTON = 6;
        // Button used to enable slow turning
        public static final int SLOW_TURN_BUTTON=1;

    }

    public static final class LimelightConstants {
        /** upward angle of limelight camera [degrees] */
        public static final double LIMELIGHT_ANGLE = 3.0;
        /** distance from limelight lens from floor [inches] */
        public static final double LIMELIGHT_HEIGHT = 18.5;
        /** distance from apriltag to floor(bottom of tag) [inches] */
        public static final double APRILTAG_HEIGHT = 14.25;
        /**
         * distance from apriltag to floor but its the double substation(bottom of tag)
         * [inches]
         */
        public static final double APRILTAG_DOUBLE_SUBSTATION_HEIGHT = 23.375;
    }
}
