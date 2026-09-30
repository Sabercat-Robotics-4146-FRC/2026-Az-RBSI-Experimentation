package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.intake.Intake;

public class MainCommand extends SequentialCommandGroup {

  public MainCommand(RobotContainer container, Extension extension, Intake intake) {
    addCommands(new ExtendCommand(extension), new RunIntake(intake));
  }
}
