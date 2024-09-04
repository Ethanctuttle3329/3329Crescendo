package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private VictorSPX top, bottom; //shooter motors

    public Shooter() {
        top = new VictorSPX(Constants.ShooterConstants.topID); //victor with ID 9
        bottom = new VictorSPX(Constants.ShooterConstants.bottomID); //victor with ID 10

        stop(); //stops shoooter motors
    }

    //runs motors forward
    public void fire() {
        top.set(ControlMode.PercentOutput, Constants.ShooterConstants.speed);
        bottom.set(ControlMode.PercentOutput, Constants.ShooterConstants.speed);
    }

    //stops motors
    public void stop() {
        top.set(ControlMode.PercentOutput, 0);
        bottom.set(ControlMode.PercentOutput, 0);
    }
}
