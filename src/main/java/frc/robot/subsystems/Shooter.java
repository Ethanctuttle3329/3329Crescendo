package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private VictorSPX top, bottom;

    public Shooter() {
        top = new VictorSPX(Constants.ShooterConstants.topID);
        bottom = new VictorSPX(Constants.ShooterConstants.bottomID);

        stop();
    }

    /**
     * Fires the shooter.
     */
    public void fire() {
        top.set(ControlMode.PercentOutput, Constants.ShooterConstants.speed);
        bottom.set(ControlMode.PercentOutput, Constants.ShooterConstants.speed);
    }

    /**
     * Stops the shooter.
     */
    public void stop() {
        top.set(ControlMode.PercentOutput, 0);
        bottom.set(ControlMode.PercentOutput, 0);
    }
}
