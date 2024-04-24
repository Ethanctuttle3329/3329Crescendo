package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private CANSparkMax left, right;
    private DutyCycleEncoder encoder;
    private ProfiledPIDController pid;
    private ArmFeedforward armFeedforward;
    private double target;

    public Arm() {
        pid = new ProfiledPIDController(
                ArmConstants.kP,
                0.0,
                ArmConstants.kD,
                new Constraints(ArmConstants.kMaxVelocityRadPerSec,
                        ArmConstants.kMaxAccelerationRadPerSecSquared));

        left = new CANSparkMax(ArmConstants.leftID, MotorType.kBrushed);
        right = new CANSparkMax(ArmConstants.rightID, MotorType.kBrushed);
        left.setInverted(ArmConstants.inverted);
        right.setInverted(!ArmConstants.inverted);
        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);

        pid.setTolerance(Units.degreesToRadians(2));

        setTargetPosition(90);

        encoder = new DutyCycleEncoder(2);
        encoder.reset();

        armFeedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);
    }

    /**
     * Sets the speed of the arm motors
     * 
     * @param power the motor speed, [-1, 1]
     */
    public void setSpeed(double power) {
        left.set(power);
        right.set(power);
    }

    /**
     * Sets output for the motor in voltage.
     * 
     * @param volts volts for each motor.
     */
    public void setVolts(double volts) {
        left.setVoltage(volts);
        right.setVoltage(volts);
    }

    /**
     * Whether the arm is at the specified PID commanded setpoint.
     * 
     * @return True if the arm is at the setpoint, false if it is not.
     */
    public boolean isAtPosition() {
        return pid.atGoal();
    }

    /**
     * Command to move the arm to a specific position.
     *
     * @param degrees The position to move to
     * @return A command that moves the arm to a specific position
     */
    public Command moveArmCommand(double degrees) {
        return Commands.runOnce(() -> this.setTargetPosition(degrees)).andThen(Commands.waitSeconds(1.2))
                .until(() -> isAtPosition());
    }

    /**
     * Gets the arm angle.
     * 
     * @return The arm encoder value in degrees.
     */
    public double getPositionEncoder() {
        // 0 -> 0.774
        // 90 -> 0.527
        return encoderToDegrees(encoder.getAbsolutePosition());
    }

    /**
     * Turns the encoder value into degrees.
     * <p>
     * 0 degrees is parallel to horizontal.
     * 
     * @param encoder The arm encoder value.
     * @return The degrees coresponding to the encoder value.
     */
    private double encoderToDegrees(double encoder) {
        return -364.372 * encoder + 228 - 67.2 + 3.8 + 2.8 + 5.6 - 12.9;
    }

    /**
     * Turns degrees into the encoder value.
     * <p>
     * 0 degrees is parallel to horizontal.
     * 
     * @param degrees The degrees the arm should be at.
     * @return The encoder value coresponding to the input degrees.
     */
    private double degreesToEncoder(double degrees) {
        return 0.7739349 - 0.00274445 * degrees;
    }

    /**
     * Sets the arm target position.
     * 
     * @param degrees target position in degrees where 0 is parallel to the ground
     */
    public void setTargetPosition(double degrees) {
        // degrees = degreesToEncoder(degrees);
        target = degrees;
        pid.setGoal(Units.degreesToRadians(degrees));
    }

    /**
     * Gets the arm's target position.
     * 
     * @return the arm target position.
     */
    public double getTargetPosition() {
        return target;
    }

    @Override
    public void periodic() {
        useOutput(target);
        SmartDashboard.putNumber("Encoder", getPositionEncoder());
        SmartDashboard.putNumber("Encoder Degrees", getPositionEncoder());
        SmartDashboard.putNumber("Target", target);
        SmartDashboard.putBoolean("At positon", isAtPosition());
    }

    protected void useOutput(double degree) {
        double ff = armFeedforward.calculate(Units.degreesToRadians(degree), 0);
        double outputVoltage = pid.calculate(Units.degreesToRadians(getPositionEncoder())) + ff;
        SmartDashboard.putNumber("Output Voltage", outputVoltage);
        setVolts(outputVoltage);
    }
}
