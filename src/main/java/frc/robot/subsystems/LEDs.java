// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private final PowerDistribution pdh;

  /**
   * Interface for turnin the LEDs on and off by
   * using the power distribution hub.
   */
  public LEDs() {
    pdh = new PowerDistribution();
    pdh.clearStickyFaults();

    off();
  }

  /**
   * Turn the leds on.
   */
  public void on() {
    pdh.setSwitchableChannel(true);
  }

  /**
   * Turn the leds off.
   */
  public void off() {
    pdh.setSwitchableChannel(false);
  }

  /**
   * Turns the leds on.
   * 
   * @return A command that turns the leds on.
   */
  public Command turnOnCommand() {
    return this.runOnce(() -> this.on());
  }

  /**
   * Turns the leds off
   * 
   * @return A command that turns the leds off.
   */
  public Command turnOffCommand() {
    return this.runOnce(() -> this.off());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
