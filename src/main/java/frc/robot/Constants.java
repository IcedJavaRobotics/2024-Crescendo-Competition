package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

public final class Constants {
    
    public static class IntakeConstants {
        public static final int SPARK_MAX_ID = 50;
        
        public static final int LIMIT_SWITCH_ID = 1;

        public static final int UPPER_ENCODER_LIMIT = 0;
        public static final int LOWER_ENCODER_LIMIT = -118; //-118
        public static final int START_SPINNING_ENCODER_VALUE = -90;

        public static final boolean MOTOR_INVERTED = true;

        public static final double SPEED = 0.55;

        public static final int FLIPPER_LIMIT = -18;
        
        public static final int EJECT_LIMIT = -50;

        public static final double INTAKE_kP = 0.05;

        
    }

    public static final class RollerConstants {
        public static final int VICTOR_ID = 51;

        public static final int DISTANCE_SENSOR_ID = 0;

        public static final boolean MOTOR_INVERTED = true;
        
        public static final double SHOOTER_SPEED = 1;

        public static final double FLIPPER_SPEED = .35;

        public static final double INTAKE_SPEED = 0.85;

        public static final double MEDIUM_SPEED = 0.4;
        public static final double SLOW_SPEED = 0.25;

        public static final double EJECT_SPEED = 0.6;

    }
    /**
     * Climber subsystem constants
     */
    public static class ClimberConstants {
    
        public static final int LEFT_TALON = 54;
        public static final int RIGHT_TALON = 55;

        public static final boolean LEFT_MOTOR_INVERTED = false;
        public static final boolean RIGHT_MOTOR_INVERTED = true;

        public static final int UPPER_LEFT_LIMIT = 12750;
        public static final int UPPER_RIGHT_LIMIT = 14800; 

        public static final int LOWER_LEFT_LIMIT = 100;
        public static final int LOWER_RIGHT_LIMIT = 10;
    
        public static final double SPEED = 1;
    
    }    

    public static final class ShooterConstants {

        //CAN IDs For the Shooter motor spark maxes
        public static final int LEFT_SPARK_ID = 52;
        public static final int RIGHT_SPARK_ID = 53;

        public static final boolean LEFT_MOTOR_INVERTED = true;
        public static final boolean RIGHT_MOTOR_INVERTED = false;

        public static final double SPEAKER_SPEED = 1; //rate it spins
        public static final double COOLDOWN_TIME = 1; //cooldown time in seconds

    }

    public static final class PneumaticsConstants {
        public static final int COMPRESSOR_ID = 3;
        public static final int PNUEMATIC_HUB_ID = 3;
    }

    public static final class BlinkinConstants {
    
        
    }
    public static final int BLINKIN = 0;
    public static final double RED = .61; 
    public static final double BLUE = .87;
    public static final double RAINBOW = -.89; 
    public static final double GREEN = .77; 
    public static final double OCEAN = -.51; 
    public static final double FOREST = -.71; 

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
        public static final double P_TURNING = .2; //0.2
        
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
        public static final boolean FRONT_LEFT_TURNING_MOTOR_REVERSED = true;
        public static final boolean FRONT_RIGHT_TURNING_MOTOR_REVERSED = true;
        public static final boolean BACK_LEFT_TURNING_MOTOR_REVERSED = true;
        public static final boolean BACK_RIGHT_TURNING_MOTOR_REVERSED = true;

        //A boolean to control the inversion of the direction of the motor gievn a positive value
        //Positive values must result in a forward movement
        public static final boolean FRONT_LEFT_DRIVE_MOTOR_REVERSED = false;
        public static final boolean FRONT_RIGHT_DRIVE_MOTOR_REVERSED = false;
        public static final boolean BACK_LEFT_DRIVE_MOTOR_REVERSED = false;
        public static final boolean BACK_RIGHT_DRIVE_MOTOR_REVERSED = false;

        //The CAN id's for the CANcoder's on the swerve modules
        public static final int FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT = 10;
        public static final int FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT = 20;
        public static final int BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT = 30;
        public static final int BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT = 40;

        //Inversion of the direction of the CANcoder
        //Positive values must result in a counter-clockwise movement
        public static final boolean FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;

        //Position must be counter-clockwise from the positive YAw
        public static final boolean GYRO_INVERTED = false; 

        //The offset of the CANcoder's position from the zero position (Straight forward)
        //Measure this by rotating all the modules to the forward position and reading the CANcoder's value
        public static final double FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = Units.degreesToRadians(-3.252);
        public static final double FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = Units.degreesToRadians(84.990);
        public static final double BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = Units.degreesToRadians(355.693);
        public static final double BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = Units.degreesToRadians(360-81.914);
        // TODO

        // in m/s, based on MK4 L2 speed of 14.5 ft/s
        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(14.5);  //14.5
        // Robot turning speed
        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 5 * Math.PI;

        public static final double TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 1;
        public static final double TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 1;  // Slowed down for testing
        public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 3;
        public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3;
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 2.2; 
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 2.2;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI / 2;
        // The P value of the PID controller used in auto for the X and Y directions
        public static final double PX_CONTROLLER = 1.5; //1.5 before change
        public static final double PY_CONTROLLER = 1.5; //1.5 before change
        // The P value of the PID controller used in auto for the theta (rotation) direction
        public static final double P_THETA_CONTROLLER = 3;

        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
                new TrapezoidProfile.Constraints(
                        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                        MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    }

    public static final class OIConstants {
        // Port for the driver's controller 
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int AUX_CONTROLLER_PORT = 1;
        public static final int DRIVER_STATION_PORT = 2;
        // Axis used for the X, Y, Rotation, and Throttle
        public static final int DRIVER_Y_AXIS = 1;
        public static final int DRIVER_X_AXIS = 0;
        public static final int DRIVER_ROT_AXIS = 4;
        public static final int DRIVER_THROTTLE_AXIS = 3;
        // Deadband for the controller
        public static final double DEADBAND = 0.2;
        // Button used to enable robot orientation driving
        public static final int FIELD_ORIENTATED_BUTTON = 4;
        // Button used to enable slow turning
        public static final int SLOW_TURN_BUTTON = 1;
        public static final int LOCK_ON_BUTTON = XboxController.Axis.kLeftTrigger.value;

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

        public static final double APRILTAG_SPEAKER_ID = 12345; //TODO
        public static final double APRILTAG_SPEAKER_HEIGHT = 24.5; //TODO //in inches

        public static final double APRILTAG_AMP_ID = 12345; //TODO
        public static final double APRILTAG_AMP_HEIGHT = 24.5; //TODO //in inches
    }
}
