// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Auto selector
  private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController mech = new CommandXboxController(1);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Intake s_Intake = new Intake();
  private final Arm s_Arm = new Arm();
  private final Shooter s_Shooter = new Shooter();
  private final Climb s_Climb = new Climb();
  private final LEDs s_LEDs = new LEDs();

  /* Triggers */
  private final Trigger robotCentric = driver.leftBumper();
  private final Trigger intakeSensor = new Trigger(() -> s_Intake.hasNote());
  private final Trigger climbTopSensor = new Trigger(() -> s_Climb.atHigherLimit());
  private final Trigger climbBottomSensor = new Trigger(() -> s_Climb.atLowerLimit());

  /* Commands */
  private final ShootCommand shootCommand = new ShootCommand(s_Shooter, s_Intake, intakeSensor);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> robotCentric.getAsBoolean()));

    // Add different auto programs
    autonChooser.setDefaultOption("Wall Drive Out 10ft No Shot", new MoveBackToNoteAuto(s_Swerve, 120));
    autonChooser.addOption("Wall Drive Out 19ft No Shot", new MoveBackToNoteAuto(s_Swerve, 120));
    autonChooser.addOption("Subwoofer 1 Shot No Drive", new ShootAuto(s_Arm, s_Intake, s_Shooter, intakeSensor, 32));
    autonChooser.addOption("Subwoofer Center 2 Shot",
        new ShootAuto(s_Arm, s_Intake, s_Shooter, intakeSensor, 32)
            .andThen(s_Arm.moveArmCommand(-4.5))
            .andThen(s_Intake.intakeCommand().until(intakeSensor).withTimeout(6)
            .alongWith(new MoveBackToNoteAuto(s_Swerve, 48)))
            .andThen(new ShootAuto(s_Arm, s_Intake, s_Shooter, intakeSensor, 48)));
    autonChooser.addOption("Subwoofer Left Wide Turn 1 Shot",
      new ShootAuto(s_Arm, s_Intake, s_Shooter, intakeSensor, 32)
          .andThen(s_Arm.moveArmCommand(48))
          .andThen(new ForwardDrive(s_Swerve, 206, -70))
          .andThen(new ForwardDrive(s_Swerve, 120, 0)));
    autonChooser.addOption("Subwoofer Left 2 Shot", 
      new ShootAuto(s_Arm, s_Intake, s_Shooter, intakeSensor, 32)
          .andThen(s_Arm.moveArmCommand(-4.5))
          .andThen(new ForwardDrive(s_Swerve, 13, -70))
          .andThen(new ForwardDrive(s_Swerve, 74, 0))
          .andThen(s_Intake.intakeCommand().until(intakeSensor).withTimeout(5)
          .andThen(new ForwardDrive(s_Swerve, 9, 25))
          .andThen(new ShootAuto(s_Arm, s_Intake, s_Shooter, intakeSensor, 52))));
    autonChooser.addOption("Subwoofer Right Wide Turn 1 Shot",
      new ShootAuto(s_Arm, s_Intake, s_Shooter, intakeSensor, 32)
          .andThen(s_Arm.moveArmCommand(48))
          .andThen(new ForwardDrive(s_Swerve, 206, 70))
          .andThen(new ForwardDrive(s_Swerve, 120, 0)));
          autonChooser.addOption("Subwoofer Right 2 Shot", 
      new ShootAuto(s_Arm, s_Intake, s_Shooter, intakeSensor, 32)
          .andThen(s_Arm.moveArmCommand(-4.5))
          .andThen(new ForwardDrive(s_Swerve, 13, 70))
          .andThen(new ForwardDrive(s_Swerve, 74, 0))
          .andThen(s_Intake.intakeCommand().until(intakeSensor).withTimeout(5)
          .andThen(new ForwardDrive(s_Swerve, 9, -25))
          .andThen(new ShootAuto(s_Arm, s_Intake, s_Shooter, intakeSensor, 52))));
    /*autonChooser.addOption("Subwoofer Center 3 Piece Right to Center", 
      new ShootAuto(s_Arm, s_Intake, s_Shooter, intakeSensor, 32)
          .andThen(s_Arm.moveArmCommand(-3.2))
          .andThen(new ForwardDrive(s_Swerve, 3, 0))
          .andThen(new SideDrive(s_Swerve, 57, 0))
          .andThen(s_Intake.intakeCommand().until(intakeSensor).withTimeout(5))
          .alongWith(new ForwardDrive(s_Swerve, 45, 0))
          .andThen(new ForwardDrive(s_Swerve, 2, -25))
          .andThen(new ShootAuto(s_Arm, s_Intake, s_Shooter, intakeSensor, 52))
          .andThen(new ForwardDrive(s_Swerve, -2, 25))
          .andThen(new ForwardDrive(s_Swerve, -12, 0))
          .andThen(new SideDrive(s_Swerve, -57, 0))
          .alongWith(s_Arm.moveArmCommand(-3.2))
          .andThen(new ForwardDrive(s_Swerve, 12, 0))
          .alongWith(s_Intake.intakeCommand().until(intakeSensor).withTimeout(5))
          .andThen(s_Arm.moveArmCommand(48))
          .andThen(new ShootAuto(s_Arm, s_Intake, s_Shooter, intakeSensor, 48)));
    autonChooser.addOption("Subwoofer Center 4 Piece Right to Left", 
      new ShootAuto(s_Arm, s_Intake, s_Shooter, intakeSensor, 32)
          .andThen(s_Arm.moveArmCommand(-3.2))
          .andThen(new ForwardDrive(s_Swerve, 3, 0))
          .andThen(new SideDrive(s_Swerve, 57, 0))
          .andThen(s_Intake.intakeCommand().until(intakeSensor).withTimeout(5))
          .alongWith(new ForwardDrive(s_Swerve, 45, 0))
          .andThen(new ForwardDrive(s_Swerve, 2, -25))
          .andThen(new ShootAuto(s_Arm, s_Intake, s_Shooter, intakeSensor, 52))
          .andThen(new ForwardDrive(s_Swerve, -2, 25))
          .andThen(new ForwardDrive(s_Swerve, -12, 0))
          .andThen(new SideDrive(s_Swerve, -57, 0))
          .alongWith(s_Arm.moveArmCommand(-3.2))
          .andThen(new ForwardDrive(s_Swerve, 12, 0))
          .alongWith(s_Intake.intakeCommand().until(intakeSensor).withTimeout(5))
          .andThen(s_Arm.moveArmCommand(48))
          .andThen(new ShootAuto(s_Arm, s_Intake, s_Shooter, intakeSensor, 48))
          .andThen(new ForwardDrive(s_Swerve, 0, -25))
          .andThen(new ForwardDrive(s_Swerve, -12, 0))
          .andThen(new SideDrive(s_Swerve, -57, 0))
          .alongWith(s_Arm.moveArmCommand(-3.2))
          .andThen(new ForwardDrive(s_Swerve, 12, 0))
          .alongWith(s_Intake.intakeCommand().until(intakeSensor).withTimeout(5))
          .andThen(s_Arm.moveArmCommand(50))
          .andThen(new ShootAuto(s_Arm, s_Intake, s_Shooter, intakeSensor, 50)));*/
    // Configure the controller button bindings
    configureButtonBindings();

    // auto triggers
    intakeSensor.onTrue(s_LEDs.turnOnCommand()).onFalse(s_LEDs.turnOffCommand());
    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * 
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    // dpad
    driver.povDown().and(climbBottomSensor.negate()).whileTrue(s_Climb.hooksDown());
    driver.povUp().and(climbTopSensor.negate()).whileTrue(s_Climb.hooksUp());

    /* Operator buttons */
    // bumpers
    mech.rightBumper()
        .onTrue(shootCommand.withTimeout(1.2));
    mech.leftBumper().onTrue(s_Arm.moveArmCommand(50));

    // axyb
    mech.a()
        .and(intakeSensor.negate())
        .whileTrue(s_Intake.intakeCommand());
    mech.x().whileTrue(s_Intake.runAmp());
    mech.y().onTrue(s_Arm.moveArmCommand(100));
    mech.b().onTrue(s_Arm.moveArmCommand(110));

    // dpad
    mech.povUp().onTrue(s_Arm.moveArmCommand(90));
    mech.povLeft().onTrue(s_Arm.moveArmCommand(32));
    mech.povRight().onTrue(s_Arm.moveArmCommand(48));
    mech.povDown().onTrue(s_Arm.moveArmCommand(-4.5));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autonChooser.getSelected();
  }
}
