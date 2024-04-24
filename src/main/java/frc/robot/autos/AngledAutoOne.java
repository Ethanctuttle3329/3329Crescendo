// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class AngledAutoOne extends SequentialCommandGroup {
  /** Creates a new AngledAutoOne. */
  public AngledAutoOne(Swerve s_Swerve, Arm s_Arm, Shooter s_Shooter, Intake s_Intake, Trigger noteDetectedTrigger) {
    TrajectoryConfig config = new TrajectoryConfig(
        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.Swerve.swerveKinematics);

    Trajectory path1 = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), Rotation2d.fromDegrees(60)),
            new Pose2d(Units.inchesToMeters(28), Units.inchesToMeters(13), Rotation2d.fromDegrees(0)),
            new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(13), Rotation2d.fromDegrees(0))),
        config);

    Trajectory path2 = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(13), Rotation2d.fromDegrees(0)),
            new Pose2d(Units.inchesToMeters(85), Units.inchesToMeters(13), Rotation2d.fromDegrees(45))),
        config);

    var thetaController = new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController,
        0,
        0,
        Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand run1 = new SwerveControllerCommand(
        path1,
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        s_Swerve::setModuleStates,
        s_Swerve);

    SwerveControllerCommand run2 = new SwerveControllerCommand(
        path2,
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        s_Swerve::setModuleStates,
        s_Swerve);

    addCommands(
        // set robot inital pose to first pose start
        new InstantCommand(() -> s_Swerve.resetOdometry(path1.getInitialPose())),
        // shoot first shot
        new ShootAuto(s_Arm, s_Intake, s_Shooter, noteDetectedTrigger, 41.52),
        // go to note while running the intake
        run1.deadlineWith(s_Intake.intakeCommand().until(noteDetectedTrigger.negate()).withTimeout(4)),
        new InstantCommand(() -> s_Swerve.stop()),
        run2,
        new InstantCommand(() -> s_Swerve.stop()),
        new ShootAuto(s_Arm, s_Intake, s_Shooter, noteDetectedTrigger, 60));
  }
}
