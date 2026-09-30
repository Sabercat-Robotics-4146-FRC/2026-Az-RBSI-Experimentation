package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.extension.Extension;

public class ExtendCommand extends Command {

  private final Extension extension;

  public ExtendCommand(Extension extension) {
    this.extension = extension;
    addRequirements(extension);
  }

  @Override
  public void execute() {
    extension.extend();
  }

  @Override
  public void end(boolean interrupted) {
    extension.stop();
  }

  @Override
  public boolean isFinished() {
    return extension.isLimitReached();
  }
}
