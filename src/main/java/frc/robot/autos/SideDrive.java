package frc.robot.autos;

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
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.List;

public class SideDrive extends SequentialCommandGroup {
        public SideDrive(Swerve s_Swerve, double distance, double angle) {
                TrajectoryConfig config = new TrajectoryConfig(
                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(Constants.Swerve.swerveKinematics);

                // An example trajectory to follow. All units in meters.
                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                
                                List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                                                new Pose2d(0, Units.inchesToMeters(distance), Rotation2d.fromDegrees(angle))),
                                config);

                var thetaController = new ProfiledPIDController(
                                Constants.AutoConstants.kPThetaController,
                                0,
                                0,
                                Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                exampleTrajectory,
                                s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                s_Swerve::setModuleStates,
                                s_Swerve);

                addCommands(
                                new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
                                swerveControllerCommand,
                                new InstantCommand(() -> s_Swerve.stop()));
        }
}
