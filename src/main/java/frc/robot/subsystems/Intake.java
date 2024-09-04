package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private VictorSPX motor; //motor that controls intake wheels
    private DigitalInput beam; //beam break sensor that detects note

    public Intake() {
        motor = new VictorSPX(Constants.IntakeConstants.intakeID); //victor with ID 11
        motor.setInverted(Constants.IntakeConstants.inverted); //inverts motor
        beam = new DigitalInput(Constants.IntakeConstants.beamID); //beam break sensor with ID 0
        stop(); //stops the motor
    }

    //runs intake forward
    public void pickup() {
        motor.set(ControlMode.PercentOutput, Constants.IntakeConstants.speed);
    }

    //stops intake
    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    //runs intake in reverse
    public void amp() {
        motor.set(ControlMode.PercentOutput, -Constants.IntakeConstants.speed);
    }

    //runs intake at higher speed
    public void feedShooter() {
        motor.set(ControlMode.PercentOutput, Constants.IntakeConstants.speed * 1.2);
    }

    //command to run intake until interrupted
    public Command intakeCommand() {
        return this.runEnd(() -> this.pickup(), () -> this.stop());
    }

    //command to run inverse in reverse
    public Command runAmp() {
        return this.runEnd(() -> this.amp(), () -> this.stop());
    }

    //returns true if beam is broken by note
    public boolean hasNote() {
        return !beam.get();
    }

    //updates information on SmartDashboard
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Beam Break", hasNote());
    }
}
