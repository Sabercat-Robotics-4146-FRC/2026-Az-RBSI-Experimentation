/*package frc.robot.commands;


public class MainCommand extends SequentialCommandGroup {

  public MainCommand(RobotContainer container, Kicker kicker, Intake intake){
    addCommands(
      new RunIntake(intake),
      Commands.runOnce(
            () -> {
              kicker.runVoltage(1.8);
            },
            kicker)

    )
  }


}
*/
