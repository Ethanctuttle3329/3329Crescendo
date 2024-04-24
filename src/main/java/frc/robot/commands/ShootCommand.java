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
  /** Creates a new Fire2. */
  public ShootCommand(Shooter shooter, Intake intake, Trigger noteDetectedTrigger) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runEnd(() -> shooter.fire(), () -> shooter.stop(), shooter),
        Commands.waitSeconds(0.75)
            .andThen(Commands.runEnd(() -> intake.feedShooter(), () -> intake.stop(), intake)),
        Commands.waitUntil(noteDetectedTrigger.debounce(0.1, DebounceType.kBoth).negate()));
  }
}
