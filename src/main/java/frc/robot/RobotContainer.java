package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.IntakeDownCommand;
import frc.robot.commands.IntakeUpCommand;
import frc.robot.commands.PickupNoteCommand;
import frc.robot.commands.RollerInCommand;
import frc.robot.commands.RollerOutCommand;
import frc.robot.commands.shooter.NoteShootCommand;
import frc.robot.commands.shooter.ShooterOutCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// import static frc.robot.Constants.Controller;
// import static frc.robot.Constants.Button;
import frc.robot.commands.Flasher;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final RollerSubsystem rollerSubsystem = new RollerSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final PneumaticSubsystem pneumaticSubsystem = new PneumaticSubsystem();
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

    private final SendableChooser<Command> autoChooser; 

    private final XboxController xboxController = new XboxController(OIConstants.kXboxControllerPort);
    private final XboxController auxController = new XboxController(OIConstants.AUX_CONTROLLER_PORT);
    private final Joystick driverStation = new Joystick(OIConstants.DRIVER_STATION_PORT);

    public RobotContainer() {
        // swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        //         swerveSubsystem,
        //         () -> -xboxController.getRawAxis(OIConstants.kDriverYAxis),
        //         () -> -xboxController.getRawAxis(OIConstants.kDriverXAxis),
        //         () -> -xboxController.getRawAxis(OIConstants.kDriverRotAxis),
        //         () -> !xboxController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
        //         () -> xboxController.getRawAxis(OIConstants.kDriverThrottleAxis),
        //         () -> xboxController.getRawButton(OIConstants.kDriverSlowTurnButtonIdx)
        //         ));
        
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // new JoystickButton(xboxController, XboxController.Button.kB.value)
        //         .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
                
        new JoystickButton(driverStation, 10 ) //9
                .onTrue(new InstantCommand(() -> intakeSubsystem.zeroIntakeEncoder()));

        new JoystickButton (driverStation, 1 ) //1
                .whileTrue(new IntakeDownCommand(intakeSubsystem));

        new JoystickButton (driverStation, 6 ) //4
                .whileTrue(new IntakeUpCommand(intakeSubsystem));  

        new JoystickButton (driverStation, 9 ) //2
                .whileTrue(new RollerInCommand(rollerSubsystem));  

        new JoystickButton (driverStation, 8 ) //5
                .whileTrue(new RollerOutCommand(rollerSubsystem));

        // new JoystickButton(auxController, XboxController.Axis.kLeftTrigger.value)
        //         .whileTrue(new PickupNoteCommand(intakeSubsystem));

        new JoystickButton(driverStation, 2) //3
                .whileTrue(new ShooterOutCommand(shooterSubsystem));

        new JoystickButton(auxController, XboxController.Button.kRightBumper.value)
                .whileTrue(new NoteShootCommand(shooterSubsystem, rollerSubsystem));
        new JoystickButton(xboxController, 2).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        new JoystickButton(driverJoytick, 2)
                .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        new JoystickButton(auxController, XboxController.Button.kRightBumper.value)
                .whileTrue(new ClimberUpCommand(climberSubsystem, pneumaticSubsystem));
        new JoystickButton(auxController, XboxController.Button.kLeftBumper.value)
                .whileTrue(new ClimberDownCommand(climberSubsystem, pneumaticSubsystem));
    }

//     public Command getAutonomousCommand() {
        // // 1. Create trajectory settings
        // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        //         AutoConstants.kMaxSpeedMetersPerSecond,
        //         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //                 .setKinematics(DriveConstants.kDriveKinematics);

        // // 2. Generate trajectory
        // // Note: -y value is to the left (field relative)
        // // This sa,ple is an example of a figure 8 auto path
        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         List.of(
        //                 new Translation2d(.7, -0.7),
        //                 new Translation2d(1.2, 0),
        //                 new Translation2d(.7, 0.7),
        //                 new Translation2d(0, 0),
        //                 new Translation2d(-.7, -0.7),
        //                 new Translation2d(-1.2, 0),
        //                 new Translation2d(-.7, 0.7)
        //                 ),
        //         new Pose2d(
        //        0, 0
        //         , Rotation2d.fromDegrees(0)),
        //         trajectoryConfig);      


        // // 3. Define PID controllers for tracking trajectory
        // PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        // PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        // ProfiledPIDController thetaController = new ProfiledPIDController(
        //         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

//         // 4. Construct command to follow trajectory
//         SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//                 trajectory,
//                 swerveSubsystem::getPose,
//                 DriveConstants.kDriveKinematics,
//                 xController,
//                 yController,
//                 thetaController,
//                 swerveSubsystem::setModuleStates,
//                 swerveSubsystem);

//         // 5. Add some init and wrap-up, and return everything
//         return new SequentialCommandGroup(
//                 new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
//                 swerveControllerCommand,
//                 new InstantCommand(() -> swerveSubsystem.stopModules()));
//     }
        // NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("AutoMode").getEntry("options").setValue(null);



        configureButtonBindings();
        // Register named commands
        NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        NamedCommands.registerCommand("print hello", Commands.print("hello"));

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("AutoMode", autoChooser);
        


    }
    private void configureButtonBindings() {
        configurePathfinderBindings();
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -xboxController.getRawAxis(OIConstants.kDriverYAxis),
                () -> -xboxController.getRawAxis(OIConstants.kDriverXAxis),
                () -> -xboxController.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !xboxController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverThrottleAxis),
                () -> driverJoytick.getRawButton(OIConstants.kDriverSlowTurnButtonIdx)
                ));
        
        new JoystickButton(xboxController, XboxController.Button.kB.value).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        bindImportantFunctions();
    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    public void bindImportantFunctions(){new JoystickButton(xboxController, XboxController.Button.kStart.value).whileTrue(new Flasher(limelightSubsystem));}
    private void configurePathfinderBindings(){
        
        SmartDashboard.updateValues();
        SmartDashboard.putData("MiddleTwoPieceAuto", new PathPlannerAuto("MiddleTwoPieceAuto"));
        SmartDashboard.updateValues();
        // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
    // SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));
    // // Add a button to run pathfinding commands to SmartDashboard
    // SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
    //   new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
    //   new PathConstraints(
    //     4.0, 4.0, 
    //     Units.degreesToRadians(360), Units.degreesToRadians(540)
    //   ), 
    //   0, 
    //   2.0
    // ));
    // SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
    //   new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
    //   new PathConstraints(
    //     4.0, 4.0, 
    //     Units.degreesToRadians(360), Units.degreesToRadians(540)
    //   ), 
    //   0, 
    //   0
    // ));

    // // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // // This example will simply move the robot 2m in the +X field direction
    // SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
    //   Pose2d currentPose = swerveSubsystem.getPose();
      
    //   // The rotation component in these poses represents the direction of travel
    //   Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
    //   Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

    //   List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
    //   PathPlannerPath path = new PathPlannerPath(
    //     bezierPoints, 
    //     new PathConstraints(
    //       4.0, 4.0, 
    //       Units.degreesToRadians(360), Units.degreesToRadians(540)
    //     ),  
    //     new GoalEndState(0.0, currentPose.getRotation())
    //   );

    //   // Prevent this path from being flipped on the red alliance, since the given positions are already correct
    //   path.preventFlipping = true;

    //   AutoBuilder.followPath(path).schedule();
    // }));
    }

}
