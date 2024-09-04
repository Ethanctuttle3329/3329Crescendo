package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  //Auto selector
  private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  //Controllers
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController mech = new CommandXboxController(1);

  //Subsystems
  private final Swerve s_Swerve = new Swerve();
  private final Intake s_Intake = new Intake();
  private final Arm s_Arm = new Arm();
  private final Shooter s_Shooter = new Shooter();
  private final Climb s_Climb = new Climb();
  private final LEDs s_LEDs = new LEDs();

  //Triggers
  private final Trigger robotCentric = driver.leftBumper();
  private final Trigger intakeSensor = new Trigger(() -> s_Intake.hasNote());
  private final Trigger climbBottomSensor = new Trigger(() -> s_Climb.atLowerLimit());

  //Commands
  private final ShootCommand shootCommand = new ShootCommand(s_Shooter, s_Intake, intakeSensor);

  public RobotContainer() {
    //drive command
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> robotCentric.getAsBoolean()));

    // Add different auto programs
    autonChooser.setDefaultOption("Wall Drive Out 10ft No Shot", new ForwardDrive(s_Swerve, 120, 0));
    autonChooser.addOption("Wall Drive Out 19ft No Shot", new ForwardDrive(s_Swerve, 228, 0));
    autonChooser.addOption("Subwoofer 1 Shot No Drive", new ShootAuto(s_Arm, s_Intake, s_Shooter, intakeSensor, 32));
    autonChooser.addOption("Subwoofer Center 2 Shot",
        new ShootAuto(s_Arm, s_Intake, s_Shooter, intakeSensor, 32)
            .andThen(s_Arm.moveArmCommand(-4.5))
            .andThen(s_Intake.intakeCommand().until(intakeSensor).withTimeout(6)
            .alongWith(new ForwardDrive(s_Swerve, 48, 0)))
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

    configureButtonBindings();

    //auto triggers
    intakeSensor.onTrue(s_LEDs.turnOnCommand()).onFalse(s_LEDs.turnOffCommand());
    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  private void configureButtonBindings() {
    //Driver Buttons
    //dpad
    driver.povDown().and(climbBottomSensor.negate()).whileTrue(s_Climb.hooksDown()); //lower hooks
    driver.povUp().whileTrue(s_Climb.hooksUp()); //raise hooks
    //Operator buttons
    //bumpers
    mech.rightBumper().onTrue(shootCommand.withTimeout(1.2)); //fires note
    mech.leftBumper().onTrue(s_Arm.moveArmCommand(50)); //moves to amp/podium arm position

    //axyb
    mech.a().and(intakeSensor.negate()).whileTrue(s_Intake.intakeCommand()); //runs intake
    mech.x().whileTrue(s_Intake.runAmp()); //reverse intake

    //dpad
    mech.povUp().onTrue(s_Arm.moveArmCommand(90)); //moves to climb/defense arm position
    mech.povLeft().onTrue(s_Arm.moveArmCommand(32)); //moves to close shot arm position
    mech.povRight().onTrue(s_Arm.moveArmCommand(48)); //moves to mid range shot arm position
    mech.povDown().onTrue(s_Arm.moveArmCommand(-4.5)); //moves to intake/stable arm position
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
