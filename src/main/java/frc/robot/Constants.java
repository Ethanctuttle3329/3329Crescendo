package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    public static final int pigeonID = 19;
    public static final boolean invertGyro = false;

    //Drivetrain Constants
    public static final double trackWidth = Units.inchesToMeters(24);
    public static final double wheelBase = Units.inchesToMeters(24);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (150.0 / 7.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    //Swerve Voltage Compensation
    public static final double voltageComp = 12.0;

    //Swerve Current Limiting
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    //Angle Motor PID Values
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    //Drive Motor PID Values
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    //Drive Motor Characterization Value
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    //Drive Motor Conversion Factors
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    //Swerve Profiling Values
    public static final double maxSpeed = 4.5 / 6; // meters per second
    public static final double maxAngularVelocity = 90 * Math.PI / 180.0; // radians per second

    //Neutral Modes
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    //Motor Inverts
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = true;

    //Angle Encoder Invert
    public static final boolean canCoderInvert = false;

    //Module Specific Constants
    //Front Left Module - Module 0
    public static final class Mod0 {
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 17;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(24.3457);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    //Front Right Module - Module 1
    public static final class Mod1 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 18;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(110.6543);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    //Back Left Module - Module 2
    public static final class Mod2 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 16;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(125.77148);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    //Back Right Module - Module 3
    public static final class Mod3 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 15;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(255.5859);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final class IntakeConstants {
    public static final int intakeID = 11;
    public static final boolean inverted = true;
    public static final double speed = 0.5;
    public static final int beamID = 0;
  }

  public static final class ClimbConstants {
    public static final int climbID = 14;
    public static final boolean inverted = true;
    public static final double speed = 1;
    public static final int bumpID = 3;
  }

  public static final class ArmConstants {
    public static final int leftID = 13;
    public static final int rightID = 12;
    public static final boolean inverted = false;
    public static final double speed = 0.2;
    public static final double kP = 50;
    public static final double kD = 0;
    public static final double kMaxVelocityRadPerSec = 3;
    public static final double kMaxAccelerationRadPerSecSquared = 3;
    public static final double kS = 0;
    public static final double kG = 1.7;
    public static final double kV = 0;

    public static final double kArmDown = -2.8;
    public static final double kArmFullUp = 90;
  }

  public static final class ShooterConstants {
    public static final int topID = 9;
    public static final int bottomID = 10;
    public static final double speed = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 6;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 4;

    public static final double kPXController = 16;
    public static final double kPYController = 16;
    public static final double kPThetaController = 8;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
