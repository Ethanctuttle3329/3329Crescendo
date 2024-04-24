package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootAuto extends SequentialCommandGroup {

  public ShootAuto(Arm s_Arm, Intake s_Intake, Shooter s_Shooter, Trigger noteDetectedTrigger, double armAngle) {
    addCommands(
        s_Arm.moveArmCommand(armAngle),
        // Commands.waitSeconds(2),
        // new RunShooter(s_Shooter, s_Intake),
        new ShootCommand(s_Shooter, s_Intake, noteDetectedTrigger).withTimeout(2));
    // Commands.waitSeconds(2));
  }
}