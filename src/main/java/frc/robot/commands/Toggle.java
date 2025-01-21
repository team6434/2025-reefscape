package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

public class Toggle extends Command {
  public boolean toggle = true;

  public Toggle() {
    System.out.println("Toggle: CONS");
  }

  @Override
  public void initialize() {
    System.out.println("Toggle: INIT : " + toggle);
    toggle = !toggle;  
  }

  @Override
  public void execute() {
    System.out.println("Toggle: EXEC");
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Toggle: END");
  }

  @Override
  public boolean isFinished() {
    System.out.println("Toggle: FINI : " + toggle);
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
