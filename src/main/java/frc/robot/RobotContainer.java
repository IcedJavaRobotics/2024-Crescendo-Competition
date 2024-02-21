// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.commands.shooter.NoteShootCommand;
import frc.robot.commands.shooter.ShooterOutCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.climber.ClimberDownCommand;
import frc.robot.commands.climber.ClimberUpCommand;
import frc.robot.commands.flipper.AmpScoreCommand;
import frc.robot.commands.flipper.LoadFlipperCommand;
import frc.robot.commands.intake.IntakeDownCommand;
import frc.robot.commands.intake.IntakeUpCommand;
import frc.robot.commands.intake.PickupNoteCommand;
import frc.robot.commands.intake.RollerInCommand;
import frc.robot.commands.intake.RollerOutCommand;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.FlasherCommand;
import frc.robot.subsystems.LimelightSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.subsystems.BlinkinSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final RollerSubsystem rollerSubsystem = new RollerSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    //private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    //private final PneumaticSubsystem pneumaticSubsystem = new PneumaticSubsystem();
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private final BlinkinSubsystem blinkinSubsystem = new BlinkinSubsystem();

    //private final SendableChooser<Command> autoChooser; 

    private final XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    private final XboxController auxController = new XboxController(OIConstants.AUX_CONTROLLER_PORT);
    private final Joystick driverStation = new Joystick(OIConstants.DRIVER_STATION_PORT);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverController.getRawAxis(OIConstants.DRIVER_Y_AXIS),
                () -> -driverController.getRawAxis(OIConstants.DRIVER_X_AXIS),
                () -> -driverController.getRawAxis(OIConstants.DRIVER_ROT_AXIS),
                () -> !driverController.getRawButton(OIConstants.FIELD_ORIENTATED_BUTTON),
                () -> driverController.getRawAxis(OIConstants.DRIVER_THROTTLE_AXIS),
                () -> driverController.getRawButton(OIConstants.SLOW_TURN_BUTTON)
                ));
        
        configureButtonBindings();
         // Register named commands
        // NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        // NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        // NamedCommands.registerCommand("print hello", Commands.print("hello"));

        // autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        // SmartDashboard.putData("AutoMode", autoChooser);

        blinkinSubsystem.autoBlinkin();

    }

    private void configureButtonBindings() {

        new JoystickButton(driverController, XboxController.Button.kB.value)
                .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
                
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

        new JoystickButton(auxController, XboxController.Button.kA.value)
                .whileTrue(new NoteShootCommand(shooterSubsystem, rollerSubsystem));

        // new JoystickButton(auxController, XboxController.Button.kRightBumper.value)
        //         .whileTrue(new ClimberUpCommand(climberSubsystem, pneumaticSubsystem));

        // new JoystickButton(auxController, XboxController.Button.kLeftBumper.value)
        //         .whileTrue(new ClimberDownCommand(climberSubsystem, pneumaticSubsystem));

        // new JoystickButton(driverController, XboxController.Button.kStart.value)
        //         .whileTrue(new FlasherCommand(limelightSubsystem));

        // new JoystickButton(auxController, XboxController.Button.kB.value)
        //         .whileTrue(new AmpScoreCommand(pneumaticSubsystem));
                
        new JoystickButton(auxController, XboxController.Button.kX.value)
                .whileTrue(new LoadFlipperCommand(rollerSubsystem, intakeSubsystem));
                
        new JoystickButton(driverStation, 4) //8
                .onTrue(new InstantCommand(() -> blinkinSubsystem.autoBlinkin()));
        
    }
    
//     public Command getAutonomousCommand() {
//         return autoChooser.getSelected();
//     }

}
