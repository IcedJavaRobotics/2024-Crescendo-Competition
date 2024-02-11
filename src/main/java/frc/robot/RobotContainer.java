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
import frc.robot.commands.Flasher;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

    private final SendableChooser<Command> autoChooser; 

    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final XboxController xboxController = new XboxController(OIConstants.kXboxControllerPort);

    public RobotContainer() {
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
