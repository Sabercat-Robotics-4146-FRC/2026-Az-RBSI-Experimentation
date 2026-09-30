package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.kicker.Kicker;

public class RunKicker extends Command {

  private final Kicker kicker;

  public RunKicker(Kicker kicker) {
    this.kicker = kicker;
    addRequirements(kicker);
  }

  @Override
  public void execute() {
    kicker.runVolts(5);
  }

  @Override
  public void end(boolean interrupted) {
    kicker.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
