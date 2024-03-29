// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.shooter.*;
import frc.robot.commands.auto.*;
import frc.robot.commands.climber.*;
import frc.robot.commands.flipper.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.roller.*;

import frc.robot.subsystems.*;

import com.fasterxml.jackson.core.sym.Name1;
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

    private final SendableChooser<Command> autoChooser; 

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

        intakeSubsystem.setDefaultCommand(new RunCommand(() -> intakeSubsystem.stopIntakeLimitSwitch(), intakeSubsystem));
        shooterSubsystem.setDefaultCommand(new RunCommand(() -> shooterSubsystem.cooldownShooter(), shooterSubsystem));
        rollerSubsystem.setDefaultCommand(new RunCommand(() -> rollerSubsystem.cooldownRoller(), rollerSubsystem));
        pneumaticSubsystem.setDefaultCommand(new RunCommand(() -> pneumaticSubsystem.cooldownFlyswatter(), pneumaticSubsystem));

        registerNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("AutoMode", autoChooser);

        blinkinSubsystem.autoBlinkin();

    }

    private void configureButtonBindings() {

        new JoystickButton(driverController, XboxController.Button.kB.value)
                .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

        new JoystickButton(auxController, XboxController.Button.kStart.value)
                .onTrue(new MediumRollerOutCommand(rollerSubsystem));

        new JoystickButton(auxController, XboxController.Button.kBack.value)
                .onTrue(new SlowRollerInCommand(rollerSubsystem));
                
        new JoystickButton(auxController, XboxController.Button.kY.value)
                .onTrue(new EmergencyEjectCommand(rollerSubsystem, intakeSubsystem));

        new JoystickButton(driverStation, 10 ) //9
                .onTrue(new InstantCommand(() -> intakeSubsystem.zeroIntakeEncoder()));

        new JoystickButton (driverStation, 1 ) //1
                .whileTrue(new IntakeOutCommand(intakeSubsystem));

        new JoystickButton (driverStation, 6 ) //4
                .whileTrue(new IntakeInCommand(intakeSubsystem));  

        new JoystickButton (driverStation, 9 ) //2
                .whileTrue(new RollerInCommand(rollerSubsystem));  

        new JoystickButton (driverStation, 8 ) //5
                .whileTrue(new RollerOutCommand(rollerSubsystem));

        new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
                .whileTrue(new PickupNoteCommand(intakeSubsystem, rollerSubsystem));

        new JoystickButton(driverStation, 2) //3
                .whileTrue(new ShooterOutCommand(shooterSubsystem));

        new JoystickButton(auxController, XboxController.Button.kA.value)
                .whileTrue(new NoteShootCommand(shooterSubsystem, rollerSubsystem));

        new JoystickButton(auxController, XboxController.Button.kRightBumper.value)
                .whileTrue(new ClimberUpCommand(climberSubsystem, pneumaticSubsystem));

        new JoystickButton(auxController, XboxController.Button.kLeftBumper.value)
                .whileTrue(new ClimberDownCommand(climberSubsystem, pneumaticSubsystem));

        new JoystickButton(auxController, XboxController.Button.kB.value)
                .whileTrue(new AmpScoreCommand(pneumaticSubsystem));
                
        new JoystickButton(auxController, XboxController.Button.kX.value)
                .whileTrue(new LoadFlipperCommand(rollerSubsystem, intakeSubsystem));
                
        // new JoystickButton(auxController, XboxController.Button.kStart.value)
        //         .whileTrue(new InstantCommand(() -> shooterSubsystem.setSpeed(0)));
        // new JoystickButton(driverStation, 4) //8
        //         .onTrue(new InstantCommand(() -> blinkinSubsystem.autoBlinkin()));

        // new JoystickButton(driverStation, 4)
        //         .whileTrue(new InstantCommand(() -> pneumaticSubsystem.releaseClimber()));

        // new JoystickButton(driverController, XboxController.Button.kLeftStick.value)
        //         .whileTrue(new InstantCommand(() -> driverController.setRumble(RumbleType.kBothRumble, 1)));
        
    }
    
    public void registerNamedCommands() {
        NamedCommands.registerCommand("RollerOutCommand", new StartRollerOutCommand(rollerSubsystem));
        NamedCommands.registerCommand("ShooterCommand", new StartShooterCommand(shooterSubsystem));
        NamedCommands.registerCommand("StopShootingCommand", new StopShootingCommand(shooterSubsystem, rollerSubsystem));

        NamedCommands.registerCommand("FullShoot", new FullShootCommand(shooterSubsystem, rollerSubsystem));
        NamedCommands.registerCommand("IntakeOut", new MoveIntakeOutCommand(intakeSubsystem));
        NamedCommands.registerCommand("IntakeIn", new MoveIntakeInCommand(intakeSubsystem));
        NamedCommands.registerCommand("RollerIn", new StartRollerInCommand(rollerSubsystem));
    }
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }


}
