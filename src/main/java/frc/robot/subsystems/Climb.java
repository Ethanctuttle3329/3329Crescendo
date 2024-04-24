package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private DigitalInput limit;
    private final double zeroPoint;

    public Climb() {
        motor = new CANSparkMax(Constants.ClimbConstants.climbID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        limit = new DigitalInput(Constants.ClimbConstants.bumpID);

        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(60);
        motor.setInverted(Constants.ClimbConstants.inverted);

        zeroPoint = encoder.getPosition();
    }

    /**
     * Makes the hooks go up.
     */
    public void up() {
        motor.set(Constants.ClimbConstants.speed);
    }

    /**
     * Makes the hooks go down.
     */
    public void down() {
        motor.set(-Constants.ClimbConstants.speed);
    }

    /**
     * Stops the climb from climbing.
     */
    public void stop() {
        motor.set(0);
    }

    /**
     * Gets whether the climb has hit the lower limit.
     * 
     * @return true if it is at the lower limit.
     */
    public boolean atLowerLimit() {
        return limit.get();
    }

    /**
     * Gets whehter the climb has hit the higher limit.
     *
     * @return true if the climb is at the higher limit.
     */
    public boolean atHigherLimit() {
        return encoder.getPosition() >= zeroPoint;
    }

    /**
     * Command to make the hooks go up. Runs infinitly until interrupted and stops
     * the motor when interrupted.
     * 
     * @return the command to make the hooks go up.
     */
    public Command hooksUp() {
        return this.runEnd(() -> this.up(), () -> this.stop());
    }

    /**
     * Command to make the hooks go Down. Runs infinitly until interrupted and stops
     * the motor when interrupted.
     * 
     * @return the command to make the hooks go down.
     */
    public Command hooksDown() {
        return this.runEnd(() -> this.down(), () -> this.stop());
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Lower Limit Hit", atLowerLimit());
        SmartDashboard.putBoolean("Higher Limit Hit", atHigherLimit());
    }
}
