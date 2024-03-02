package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, throttle, lockedOnDouble;
    private final Supplier<Boolean> fieldOrientedFunction, slowTurn;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private double throttleadjusted;
    private LimelightSubsystem limelight;

    public SwerveJoystickCmd(
        SwerveSubsystem swerveSubsystem,
        Supplier<Double> xSpdFunction, 
        Supplier<Double> ySpdFunction, 
        Supplier<Double> turningSpdFunction,
        Supplier<Boolean> fieldOrientedFunction,
        Supplier<Double> throttle,
        Supplier<Boolean> slowTurn,
        Supplier<Double> lockedOn,
        LimelightSubsystem limelight
        ) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.yLimiter = new SlewRateLimiter(DriveConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
        this.throttle = throttle;
        this.slowTurn = slowTurn;
        this.lockedOnDouble = lockedOn;
        this.limelight = limelight;
        addRequirements(swerveSubsystem);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Front Left Spd", swerveSubsystem.frontLeft.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("Front Right Spd", swerveSubsystem.frontRight.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("Back Left Spd", swerveSubsystem.backLeft.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("Back Right Spd", swerveSubsystem.backRight.getState().speedMetersPerSecond);
        
        
        // 1. Get real-time joystick inputs
        throttleadjusted = throttle.get()*-.25+.75;
        double xSpeed = xSpdFunction.get()*throttleadjusted;
        double ySpeed = ySpdFunction.get()*throttleadjusted; 

        double turningSpeed = turningSpdFunction.get();
        
        

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.DEADBAND ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.DEADBAND ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND;

        // System.out.println("xSpeed: " + xSpeed);
        // System.out.println("ySpeed: " + ySpeed);

        // Slow turning speed on true of slowTurn button boolean supplier
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
        if (slowTurn.get()==true){
            turningSpeed=turningSpeed/2;
        }
        
        if(lockedOn()){
            final var rot_limelight = limelight_aim_proportional();
            turningSpeed = rot_limelight;
        }   //IF LOCKED ON CHANGE ROTATION

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean lockedOn(){
        if(lockedOnDouble.get() > 0.2){
            return true;
        }
        return false;
    }
    // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = limelight.getTx() * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= DriveConstants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }
}
