package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private VictorSPX motor;
    private DigitalInput beam;

    public Intake() {
        motor = new VictorSPX(Constants.IntakeConstants.intakeID);
        motor.setInverted(Constants.IntakeConstants.inverted);
        beam = new DigitalInput(Constants.IntakeConstants.beamID);

        stop();
    }

    /**
     * Runs the intake to intake.
     */
    public void pickup() {
        motor.set(ControlMode.PercentOutput, Constants.IntakeConstants.speed);
    }

    /**
     * Stops the intake.
     */
    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Runs at a less speed and in reverse for an amp shot.
     */
    public void amp() {
        motor.set(ControlMode.PercentOutput, -Constants.IntakeConstants.speed);
    }

    /**
     * Runs ata higher speed to feed the motor.
     */
    public void feedShooter() {
        motor.set(ControlMode.PercentOutput, Constants.IntakeConstants.speed * 1.2);
    }

    /**
     * Runs the intake continuously and stops the intake when interrupted.
     * 
     * @return A command which intakes.
     */
    public Command intakeCommand() {
        return this.runEnd(() -> this.pickup(), () -> this.stop());
    }

    /**
     * Runs the intake in reverse and at a less speed. Runs untill interrupted and
     * stops the intake when interrupted.
     * 
     * @return the command to run the amp.
     */
    public Command runAmp() {
        return this.runEnd(() -> this.amp(), () -> this.stop());
    }

    /**
     * Gets whether the intake has a note or not.
     * 
     * @return true if the intake detects it has a note
     */
    public boolean hasNote() {
        return !beam.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Beam Break", hasNote());
    }
}
