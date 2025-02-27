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
  public void initialize() {}

  @Override
  public void execute() {
    if (elevatorSubsystem.tofIsConnected()) {
      sumOverWindow += elevatorSubsystem.getToFDistance();
      steps++;
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.resetPosition(sumOverWindow/(double)steps);
  }

  @Override
  public boolean isFinished() {
    if (steps >= 20) {
      return true;
    }
    else return false;
  }
}
