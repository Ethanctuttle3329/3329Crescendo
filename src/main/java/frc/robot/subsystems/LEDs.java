// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private final PowerDistribution pdh;

  public LEDs() {
    pdh = new PowerDistribution(); //power control for LEDs
    pdh.clearStickyFaults(); //resets LEDs

    off(); //turns LEDs off
  }

  //turns LEDs on
  public void on() {
    pdh.setSwitchableChannel(true);
  }

  //turns LEDs off
  public void off() {
    pdh.setSwitchableChannel(false);
  }

  //command to turn LEDs on
  public Command turnOnCommand() {
    return this.runOnce(() -> this.on());
  }

  //command to turn LEDs off
  public Command turnOffCommand() {
    return this.runOnce(() -> this.off());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
