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
    private CANSparkMax left, right; //motors that control the lifting of the arm
    private DutyCycleEncoder encoder; //encoder that keeps track of position of the arm
    private ProfiledPIDController pid;
    private ArmFeedforward armFeedforward;
    private double target; //target position for the arm

    public Arm() {
        //used for adujusting arm speed to get to target position
        pid = new ProfiledPIDController(
                ArmConstants.kP,
                0.0,
                ArmConstants.kD,
                new Constraints(ArmConstants.kMaxVelocityRadPerSec,
                        ArmConstants.kMaxAccelerationRadPerSecSquared));

        left = new CANSparkMax(ArmConstants.leftID, MotorType.kBrushed); //sparkmax with ID 13
        right = new CANSparkMax(ArmConstants.rightID, MotorType.kBrushed); //sparkmax with ID 12
        left.setInverted(ArmConstants.inverted);
        right.setInverted(!ArmConstants.inverted); //inverts right side motor
        left.setIdleMode(IdleMode.kBrake); //sets motor to brake mode
        right.setIdleMode(IdleMode.kBrake); //sets motor to brake mode

        pid.setTolerance(Units.degreesToRadians(2));

        setTargetPosition(90); //sets initial position for arm at 90 degrees

        encoder = new DutyCycleEncoder(2); //creates an encoder with id 2
        encoder.reset(); //resets encoder position on power on
        //used for adujusting arm speed to get to target position
        armFeedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);
    }

    //sets the speed of the arm motors (-1 - 1)
    public void setSpeed(double power) {
        left.set(power);
        right.set(power);
    }

    //sets the voltage output of the motors
    public void setVolts(double volts) {
        left.setVoltage(volts);
        right.setVoltage(volts);
    }

    //returns true if arm is at the target position, false otherwise
    public boolean isAtPosition() {
        return pid.atGoal();
    }

    // command to move are to a particular position in degrees
    public Command moveArmCommand(double degrees) {
        return Commands.runOnce(() -> this.setTargetPosition(degrees)).andThen(Commands.waitSeconds(1.2))
                .until(() -> isAtPosition());
    }

    //returns the arm position in degrees
    public double getPositionEncoder() {
        return encoderToDegrees(encoder.getAbsolutePosition());
    }

    //converts encoder values to degrees
    private double encoderToDegrees(double encoder) {
        return -364.372 * encoder + 160.1;
    }

    //sets the target position
    public void setTargetPosition(double degrees) {
        target = degrees;
        pid.setGoal(Units.degreesToRadians(degrees));
    }

    //returns the target position
    public double getTargetPosition() {
        return target;
    }

    //updates information of SmartDashboard
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
