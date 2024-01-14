package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.util.Units;

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

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
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

    public Pose2d getPose() {
        return odometer.getPoseMeters();
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
