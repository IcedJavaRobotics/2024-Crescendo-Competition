package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.IntakeDownCommand;
import frc.robot.commands.IntakeShootCommand;
import frc.robot.commands.IntakeUpCommand;
import frc.robot.commands.PickupNoteCommand;
import frc.robot.commands.RollerInCommand;
import frc.robot.commands.RollerOutCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import frc.robot.commands.shooter.NoteShoot;
import frc.robot.commands.shooter.ShooterOutCommand;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    private final XboxController xboxController = new XboxController(OIConstants.kXboxControllerPort);
    private final XboxController auxController = new XboxController(OIConstants.AUX_CONTROLLER_PORT);
    private final Joystick driverStation = new Joystick(OIConstants.DRIVER_STATION_PORT);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -xboxController.getRawAxis(OIConstants.kDriverYAxis),
                () -> -xboxController.getRawAxis(OIConstants.kDriverXAxis),
                () -> -xboxController.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !xboxController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
                () -> xboxController.getRawAxis(OIConstants.kDriverThrottleAxis),
                () -> xboxController.getRawButton(OIConstants.kDriverSlowTurnButtonIdx)
                ));
        
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(xboxController, XboxController.Button.kB.value)
                .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
                
        new JoystickButton (driverStation, 1 ) //1
                .whileTrue(new IntakeDownCommand(intakeSubsystem));
        new JoystickButton (driverStation, 6 ) //4
                .whileTrue(new IntakeUpCommand(intakeSubsystem));  

        new JoystickButton (driverStation, 9 ) //2
                .whileTrue(new RollerInCommand(intakeSubsystem));   
        new JoystickButton (driverStation, 8 ) //5
                .whileTrue(new RollerOutCommand(intakeSubsystem));

        new JoystickButton (driverStation, 3 ) //6
                .whileTrue(new IntakeShootCommand(intakeSubsystem));
        new JoystickButton(auxController, XboxController.Axis.kLeftTrigger.value)
                .whileTrue(new PickupNoteCommand(intakeSubsystem));

        new JoystickButton(auxController, XboxController.Axis.kRightTrigger.value)
                .whileTrue(new NoteShoot(shooterSubsystem, intakeSubsystem));
        new JoystickButton(driverStation, 2) //3
                .whileTrue(new ShooterOutCommand(shooterSubsystem));
    }

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        // Note: -y value is to the left (field relative)
        // This sa,ple is an example of a figure 8 auto path
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(.7, -0.7),
                        new Translation2d(1.2, 0),
                        new Translation2d(.7, 0.7),
                        new Translation2d(0, 0),
                        new Translation2d(-.7, -0.7),
                        new Translation2d(-1.2, 0),
                        new Translation2d(-.7, 0.7)
                        ),
                new Pose2d(
               0, 0
                , Rotation2d.fromDegrees(0)),
                trajectoryConfig);      


        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}
