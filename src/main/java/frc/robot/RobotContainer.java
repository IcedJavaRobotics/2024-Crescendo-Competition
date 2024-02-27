// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.shooter.*;
import frc.robot.commands.climber.*;
import frc.robot.commands.flipper.*;
import frc.robot.commands.intake.*;
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
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final PneumaticSubsystem pneumaticSubsystem = new PneumaticSubsystem();
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private final BlinkinSubsystem blinkinSubsystem = new BlinkinSubsystem();

    //private final SendableChooser<Command> autoChooser; 

    private final XboxController driveController2 = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    private final XboxController auxController2 = new XboxController(OIConstants.AUX_CONTROLLER_PORT);
    private final Joystick driveStation2 = new Joystick(OIConstants.DRIVER_STATION_PORT);

    private final CommandXboxController driveController = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController auxController = new CommandXboxController(OIConstants.AUX_CONTROLLER_PORT);
    private final CommandJoystick driverStation = new CommandJoystick(OIConstants.DRIVER_STATION_PORT);

    private Trigger driverRightTrigger;
    private Trigger driverBButton;

    private Trigger auxLeftTrigger;
    private Trigger auxRightTrigger;
    private Trigger auxLeftBumper;
    private Trigger auxRightBumper;
    private Trigger auxAButton;
    private Trigger auxBButton;
    private Trigger auxXButton;

    private Trigger driverStationUpperLeft;
    private Trigger driverStationUpperCenter;
    private Trigger driverStationUpperRight;
    private Trigger driverStationMiddleLeft;
    private Trigger driverStationMiddleCenter;
    private Trigger driverStationMiddleRight;
    private Trigger driverStationLowerLeft;
    private Trigger driverStationLowerCenter;
    private Trigger driverStationLowerRight;

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driveController.getRawAxis(OIConstants.DRIVER_Y_AXIS),
                () -> -driveController.getRawAxis(OIConstants.DRIVER_X_AXIS),
                () -> -driveController.getRawAxis(OIConstants.DRIVER_ROT_AXIS),
                () -> !driveController.button(OIConstants.FIELD_ORIENTATED_BUTTON).getAsBoolean(),
                () -> driveController.getRawAxis(OIConstants.DRIVER_THROTTLE_AXIS),
                () -> driveController.button(OIConstants.SLOW_TURN_BUTTON).getAsBoolean()
                ));
        
        configureButtonBindings();

        // autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        // SmartDashboard.putData("AutoMode", autoChooser);
        blinkinSubsystem.autoBlinkin();

    }

    private void configureButtonBindings() {

        // Default Commands
        intakeSubsystem.setDefaultCommand(new RunCommand(() -> intakeSubsystem.stopIntakeLimitSwitch(), intakeSubsystem);
        shooterSubsystem.setDefaultCommand(new RunCommand(() -> shooterSubsystem.cooldownShooter(), shooterSubsystem);
        rollerSubsystem.setDefaultCommand(new RunCommand(() -> rollerSubsystem.loadShooter(), rollerSubsystem);
      
        // Drive Controls
        driverBButton = driveController.b();
        driverBButton.whileTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

        
        // Combo Controls
        auxRightTrigger = auxController.axisGreaterThan(3, 0.2);
        auxRightTrigger.whileTrue(new NoteShootCommand(shooterSubsystem, rollerSubsystem));

        driverRightTrigger = driveController.axisGreaterThan(3, 0.2);
        driverRightTrigger.whileTrue(new PickupNoteCommand(intakeSubsystem, rollerSubsystem));


        // Intake Controls
        driverStationLowerRight = driverStation.button(10);
        driverStationLowerRight.whileTrue(new InstantCommand(() -> intakeSubsystem.zeroIntakeEncoder()));

        driverStationUpperLeft = driverStation.button(1);
        driverStationUpperLeft.whileTrue(new IntakeOutCommand(intakeSubsystem));
        
        driverStationMiddleLeft = driverStation.button(6);
        driverStationMiddleLeft.whileFalse(new IntakeInCommand(intakeSubsystem));


        // Roller Controls
        driverStationUpperCenter = driverStation.button(9);
        driverStationUpperCenter.whileTrue(new RollerInCommand(rollerSubsystem));

        driverStationMiddleCenter = driverStation.button(8);
        driverStationMiddleCenter.whileTrue(new RollerOutCommand(rollerSubsystem));


        // Shooter Controls 
        driverStationUpperRight = driverStation.button(2);
        driverStationUpperRight.whileTrue(new ShooterOutCommand(shooterSubsystem));


        // Climber Controls
        auxRightBumper = auxController.rightBumper();
        auxRightBumper.whileTrue(new ClimberUpCommand(climberSubsystem, pneumaticSubsystem));

        auxLeftBumper = auxController.leftBumper();
        auxLeftBumper.whileTrue(new ClimberDownCommand(climberSubsystem, pneumaticSubsystem));


        // Flipper Controls
        auxBButton = auxController.b();
        auxBButton.whileTrue(new AmpScoreCommand(pneumaticSubsystem));

        auxXButton = auxController.x();
        auxXButton.whileTrue(new LoadFlipperCommand(rollerSubsystem, intakeSubsystem));


        driverStationLowerCenter = driverStation.Button(4);
        driverStationLowerCenter.whileTrue(new InstantCommand(() -> pneumaticSubsystem.releaseClimber()));

        // driverStationLowerCenter = driverStation.button(4);
        // driverStationLowerCenter.whileTrue(new InstantCommand(() -> blinkinSubsystem.autoBlinkin()));

        // new JoystickButton(driveController2, XboxController.Button.kLeftStick.value)
        //         .whileTrue(new InstantCommand(() -> driveController2.setRumble(RumbleType.kBothRumble, 1)));

        // new JoystickButton(driveController2, XboxController.Button.kStart.value)
        //         .whileTrue(new FlasherCommand(limelightSubsystem));
        
    }
    
//     public Command getAutonomousCommand() {
//         return autoChooser.getSelected();
//     }

}
