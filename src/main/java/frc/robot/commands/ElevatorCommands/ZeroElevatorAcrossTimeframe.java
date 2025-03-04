package frc.robot.commands.ElevatorCommands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroElevatorAcrossTimeframe extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private int steps;
  private double sumOverWindow;
  private int tossedOut;

  public ZeroElevatorAcrossTimeframe(ElevatorSubsystem subsystem) {
    elevatorSubsystem = subsystem;
    steps = 0;
    sumOverWindow = 0;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    steps = 0;
    tossedOut = 0;
    sumOverWindow = 0;
  }

  @Override
  public void execute() {
    DogLog.log("auto/tofisconnected", elevatorSubsystem.tofIsConnected());
    if (elevatorSubsystem.tofIsConnected()) {
      DogLog.log("auto/gettofdistance", elevatorSubsystem.getToFDistance());
      if ((elevatorSubsystem.getToFDistance() <= 0.04)
          && (elevatorSubsystem.getToFDistance() >= -0.01)) {
        sumOverWindow += elevatorSubsystem.getToFDistance();
        steps++;
      } else {
        tossedOut++;
      }
      DogLog.log("auto/steps", steps);
      DogLog.log("auto/tossedOut", tossedOut);
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.resetPosition(sumOverWindow / (double) steps);
  }

  @Override
  public boolean isFinished() {
    if (steps >= 30) {
      return true;
    } else return false;
  }
}
