package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private final Pigeon2 gyro;

    private SwerveDriveOdometry swerveOdometry;
    private frc.robot.subsystems.SwerveModule[] mSwerveMods;

    private Field2d field;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        zeroGyro();

        mSwerveMods = new frc.robot.subsystems.SwerveModule[] {
                new frc.robot.subsystems.SwerveModule(0, Constants.Swerve.Mod0.constants),
                new frc.robot.subsystems.SwerveModule(1, Constants.Swerve.Mod1.constants),
                new frc.robot.subsystems.SwerveModule(2, Constants.Swerve.Mod2.constants),
                new frc.robot.subsystems.SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        field = new Field2d();
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Gyro", gyro);

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getStates());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), translation.getY(), rotation, getYaw())
                        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (frc.robot.subsystems.SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (frc.robot.subsystems.SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void stop() {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics
                .toSwerveModuleStates(ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, getYaw()));
        for (frc.robot.subsystems.SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getStates(), pose);
    }

    /**
     * 
     * @return
     */
    public SwerveModulePosition[] getStates() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (frc.robot.subsystems.SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getState();
        }
        return positions;
    }

    /**
     * Sets the gyro to zero.
     */
    public void zeroGyro() {
        gyro.setYaw(0);
    }

    /**
     * Sets the gyro to a particular value. Useful for if the autonomus is starting
     * at an angle such that the robot is enabled at an angle.
     * 
     * @param value value to set to. Value is in degrees.
     */
    public void setGyro(double value) {
        gyro.setYaw(value);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro)
                ? Rotation2d.fromDegrees(360 - (gyro.getYaw().getValue()))
                : Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getStates());
        field.setRobotPose(getPose());

        for (frc.robot.subsystems.SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " NEO Position", mod.getAngle().getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Velocity", mod.getState().distanceMeters);
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Module Angle", mod.getState().angle.getDegrees());
        }
    }
}