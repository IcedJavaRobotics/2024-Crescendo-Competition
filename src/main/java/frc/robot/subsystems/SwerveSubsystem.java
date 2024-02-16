package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveSubsystem extends SubsystemBase {
    public final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveMotorReversed,
            DriveConstants.kFrontLeftTurningMotorReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
            "Front Left");

    public final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveMotorReversed,
            DriveConstants.kFrontRightTurningMotorReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
            "Front Right");

    public final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveMotorReversed,
            DriveConstants.kBackLeftTurningMotorReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
            "Back Left");

    public final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveMotorReversed,
            DriveConstants.kBackRightTurningMotorReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
            "Back Right");

    //Pigeon2 is CTRE gyro module 
    private final Pigeon2 gyro = new Pigeon2(frc.robot.Constants.DriveConstants.kPigeonPort);
    
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), 
            getModulePositions());

            private SwerveModule[] modules;

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        // Configure AutoBuilder last
        modules = new SwerveModule[]{
            frontLeft, frontRight, backLeft, backRight
          };

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(AutoConstants.kPXController, 0, 0.0), // Translation PID constants
                        new PIDConstants(AutoConstants.kPThetaController, 0, 0.0), // Rotation PID constants
                        7.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
        
    }
    //commands required for holometric swerve
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }
    public void resetPose(Pose2d pose) {
        //getRotation2D is replaced by new Rotation2D(gyro.getYaw())
        odometer.resetPosition(new Rotation2d(Math.toRadians(gyro.getYaw())), getPositions(), pose);
    }
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setStates(targetStates);
    }

    public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, AutoConstants.kMaxSpeedMetersPerSecond);

        for (int i = 0; i < modules.length; i++) {
            // modules[i].setTargetState(targetStates[i]);
            modules[i].setDesiredState(targetStates[i]);
        }
    }


    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
          states[i] = modules[i].getState();
        }
        return states;
    }
    /*
     *  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
    }
     */
    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
          positions[i] = modules[i].getPosition();
        }
        return positions;
    }
    
    public void zeroHeading() {
        gyro.setYaw(0);
    }

    public double getYaw() {
        return gyro.getYaw();
    }

    public double getHeading() {
        return ((DriveConstants.kGyroInverted?-1.0:1.0) * Math.IEEEremainder(getYaw(), 360));
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    
    

    public SwerveModulePosition[] getModulePositions() {

        SwerveModulePosition[] positions = {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };

        return positions;

    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(),pose);
        }
    

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("Back Right angle", (Units.radiansToDegrees(backRight.getAbsoluteEncoderRad())));
        SmartDashboard.putNumber("Back Left angle", (Units.radiansToDegrees(backLeft.getAbsoluteEncoderRad())));
        SmartDashboard.putNumber("Front Right angle", (Units.radiansToDegrees(frontRight.getAbsoluteEncoderRad())));
        SmartDashboard.putNumber("Front Left angle", (Units.radiansToDegrees(frontLeft.getAbsoluteEncoderRad())));

        SmartDashboard.putString("Back Right position", (backRight.getPosition().toString()));
        SmartDashboard.putString("Back Left position", (backLeft.getPosition().toString()));
        SmartDashboard.putString("Front Right position", (frontRight.getPosition().toString()));
        SmartDashboard.putString("Front Left position", (frontLeft.getPosition().toString()));

        //Raw encoder values with no offset
        SmartDashboard.putNumber("Raw Back Right angle", (Units.radiansToDegrees(backRight.getRawEncoderValue())));
        SmartDashboard.putNumber("Raw Back Left angle", (Units.radiansToDegrees(backLeft.getRawEncoderValue())));
        SmartDashboard.putNumber("Raw Front Right angle", (Units.radiansToDegrees(frontRight.getRawEncoderValue())));
        SmartDashboard.putNumber("Raw Front Left angle", (Units.radiansToDegrees(frontLeft.getRawEncoderValue())));

        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
        SmartDashboard.putNumber("Pose ROT", getPose().getRotation().getDegrees());
        
        SmartDashboard.putNumber("Front left velocity", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("Front right velocity", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("back left velocity", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("back right velocity", backRight.getDriveVelocity());

    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
        System.out.println("Swerve STOP done.");
        
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
