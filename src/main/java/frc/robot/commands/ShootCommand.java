package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCommand extends ParallelCommandGroup {

  public ShootCommand(Shooter shooter, Kicker kicker) {
    addCommands(new RunShooter(shooter), new RunKicker(kicker));
  }
}
