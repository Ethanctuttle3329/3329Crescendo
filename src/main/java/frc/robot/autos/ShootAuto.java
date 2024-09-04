package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootAuto extends SequentialCommandGroup {
  //moves arm to specified angle then fires
  public ShootAuto(Arm s_Arm, Intake s_Intake, Shooter s_Shooter, Trigger noteDetectedTrigger, double armAngle) {
    addCommands(
        s_Arm.moveArmCommand(armAngle),
        new ShootCommand(s_Shooter, s_Intake, noteDetectedTrigger).withTimeout(2));
  }
}