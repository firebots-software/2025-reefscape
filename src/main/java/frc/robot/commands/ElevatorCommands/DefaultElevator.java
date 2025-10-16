package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.ElevatorSubsystemMD2;

public class DefaultElevator extends Command {
  private final ElevatorSubsystemMD2 elevatorSubsystem;

  // private WindowAverage windowAverage = new WindowAverage();
  // private double currentAverage = 0.0;

  public DefaultElevator(ElevatorSubsystemMD2 subsystem) {
    elevatorSubsystem = subsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // if (elevatorSubsystem.isAtPosition()
    //     && elevatorSubsystem.getLevel().equals(ElevatorPositions.Intake)) {
    //   windowAverage.addValue(elevatorSubsystem.getToFDistance());
    //   if (currentAverage != windowAverage.getAverage()) {
    //     currentAverage = windowAverage.getAverage();
    //     elevatorSubsystem.resetPosition(windowAverage.getAverage());
    //   }
    //   DogLog.log("mattcs/average", windowAverage.getAverage());
    // } else if (!elevatorSubsystem.getLevel().equals(ElevatorPositions.Intake)
    //     && !CoralPosition.isCoralInFunnel()
    //     && !FunnelSubsystem.getInstance().isCoralCheckedIn()
    //     && !FunnelSubsystem.getInstance().isCoralCheckedOut()) {
    //   DogLog.log("mattcs/level", elevatorSubsystem.getLevel().equals(ElevatorPositions.Intake));
    //   DogLog.log("mattcs/isCoralInFunnel", CoralPosition.isCoralInFunnel());
    //   DogLog.log("mattcs/isCoralCheckedIn", FunnelSubsystem.getInstance().isCoralCheckedIn());
    //   DogLog.log("mattcs/isCoralCheckedOut", FunnelSubsystem.getInstance().isCoralCheckedOut());
    //   windowAverage.clearWindow();
    // }

    if (!CoralPosition.isCoralInTootsieSlide()) {
      elevatorSubsystem.setHeight(ElevatorPositions.Intake.height);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
