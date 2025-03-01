package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroElevatorAcrossTimeframe extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private int steps;
  private double sumOverWindow;

  public ZeroElevatorAcrossTimeframe(ElevatorSubsystem subsystem) {
    elevatorSubsystem = subsystem;
    steps = 0;
    sumOverWindow = 0;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    steps = 0;
    sumOverWindow = 0;
  }

  @Override
  public void execute() {
    if (elevatorSubsystem.tofIsConnected()) {
      if ((elevatorSubsystem.getToFDistance() <= 0.03) && (elevatorSubsystem.getToFDistance() >= 0.01)) {
        sumOverWindow += elevatorSubsystem.getToFDistance();
        steps++;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.resetPosition(sumOverWindow / (double) steps);
  }

  @Override
  public boolean isFinished() {
    if (steps >= 40) {
      return true;
    } else return false;
  }
}
