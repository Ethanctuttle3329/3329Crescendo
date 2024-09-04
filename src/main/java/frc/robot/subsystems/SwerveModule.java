package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.OnboardModuleState;
import frc.robot.Constants;

public class SwerveModule {
  public int moduleNumber; //swerve module number
  private Rotation2d lastAngle; //previous angle
  private Rotation2d angleOffset; //angle offset

  private CANSparkMax angleMotor; //motor that controls the direction the wheel is facing
  private CANSparkMax driveMotor; //motor that controls rotation of the wheel

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder; //cancoder that keeps track of rotation for the module

  private final SparkPIDController driveController; //PID for drive motor
  private final SparkPIDController angleController; //PID for angle motor

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      Constants.Swerve.driveKS, Constants.Swerve.driveKV);

  public SwerveModule(int moduleNumber, frc.lib.config.SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber; //sets module number
    angleOffset = moduleConstants.angleOffset; //sets angle offset

    angleEncoder = new CANcoder(moduleConstants.cancoderID); //cancoder with ID dependent on module number (set in constants file)
    CANcoderConfigurator cfg = angleEncoder.getConfigurator(); //creates cancoder configurator (allows adjusting cancoder)
    cfg.apply(new CANcoderConfiguration()); //applys basic cancoder configuration
    MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
    cfg.refresh(magnetSensorConfigs); //refreshes configuration
    cfg.apply(magnetSensorConfigs
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1) //sets sensor range
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)); //sets counterclockwise to positive direction for rotation

    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, CANSparkMax.MotorType.kBrushless); //sparkmax with ID dependent on module number (set in constants file)
    integratedAngleEncoder = angleMotor.getEncoder(); //gets the integrated encoder in the angle motor
    angleController = angleMotor.getPIDController(); //creates angle motor PID controller
    configAngleMotor(); //configures angle motor based on method below

    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, CANSparkMax.MotorType.kBrushless); //sparkmax with ID dependent on module number (set in constants file)
    driveEncoder = driveMotor.getEncoder(); //gets the integrated encoder in the drive motor
    driveController = driveMotor.getPIDController(); //creates drive motor PID controller
    configDriveMotor(); //configures drive motor based on method below
    lastAngle = getState().angle; //sets last angle to current angle
  }

  //sets desired angle
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  //resets angle motor to absolute (might not work)
  private void resetNEOSToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults(); //sets angle motor to default
    angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit); //sets current limit
    angleMotor.setInverted(Constants.Swerve.angleInvert); //inverts angle motor
    angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode); //sets brake mode
    integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    angleController.setP(Constants.Swerve.angleKP);
    angleController.setI(Constants.Swerve.angleKI);
    angleController.setD(Constants.Swerve.angleKD);
    angleController.setFF(Constants.Swerve.angleKFF);
    angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    angleMotor.burnFlash(); //saves configuration

    resetNEOSToAbsolute(); //resets motor to absolute (might not work)
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults(); //sets drive motor to default
    driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit); //sets current limit
    driveMotor.setInverted(Constants.Swerve.driveInvert);
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode); //sets brake mode
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    driveController.setP(Constants.Swerve.angleKP);
    driveController.setI(Constants.Swerve.angleKI);
    driveController.setD(Constants.Swerve.angleKD);
    driveController.setFF(Constants.Swerve.angleKFF);
    driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    driveMotor.burnFlash(); //saves configuration
    driveEncoder.setPosition(0.0); //sets wheel to initial position
  }

  //sets speed of drive motor based on PID controller
  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  //rotates the module to the desired angle based on PID
  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  //returns current angle from integrated angle encoder
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  //returns current angle from cancoder
  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getPosition().getValueAsDouble() * 360 - angleOffset.getDegrees());
  }

  //returns rotation of the wheel
  public SwerveModulePosition getState() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }
}
