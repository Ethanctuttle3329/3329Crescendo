// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends ParallelRaceGroup {
  //spins up shooter motor, waits 0.75 seconds, and then runs intake wheels in reverse to fire
  public ShootCommand(Shooter shooter, Intake intake, Trigger noteDetectedTrigger) {
    addCommands(
        Commands.runEnd(() -> shooter.fire(), () -> shooter.stop(), shooter),
        Commands.waitSeconds(0.75)
            .andThen(Commands.runEnd(() -> intake.feedShooter(), () -> intake.stop(), intake)),
        Commands.waitUntil(noteDetectedTrigger.debounce(0.1, DebounceType.kBoth).negate()));
  }
}
