package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    private CANSparkMax motor; //motor that moves the hooks
    private DigitalInput limit; //limit switch at bottom of left hook

    public Climb() {
        motor = new CANSparkMax(Constants.ClimbConstants.climbID, MotorType.kBrushless); //sparkmax with ID 14
        limit = new DigitalInput(Constants.ClimbConstants.bumpID); //limit switch with ID 3
        motor.restoreFactoryDefaults(); //restores motor to factory default
        motor.setIdleMode(IdleMode.kBrake); //sets motor to brake mode
        motor.setSmartCurrentLimit(60); //limits current to motor
        motor.setInverted(Constants.ClimbConstants.inverted); //inverts motor
    }

    //raises hooks
    public void up() {
        motor.set(Constants.ClimbConstants.speed);
    }

    //lowers hooks
    public void down() {
        motor.set(-Constants.ClimbConstants.speed);
    }

    //stops hooks
    public void stop() {
        motor.set(0);
    }

    //returns true if the hook is pressing the limit switch
    public boolean atLowerLimit() {
        return limit.get();
    }

    //command to raise hooks
    public Command hooksUp() {
        return this.runEnd(() -> this.up(), () -> this.stop());
    }

    //command to lower hooks
    public Command hooksDown() {
        return this.runEnd(() -> this.down(), () -> this.stop());
    }

    //updates information on SmartDashboard
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Lower Limit Hit", atLowerLimit());
    }
}
