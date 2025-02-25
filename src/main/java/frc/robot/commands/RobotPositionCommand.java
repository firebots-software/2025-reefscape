package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.Landmarks.BranchSide;
import frc.robot.util.RobotPosition;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RobotPositionCommand extends Command {

  RobotPosition position;
  ElevatorPositions elevatorPosition;
  BranchSide branchSide;
  public RobotPositionCommand(ElevatorPositions pos,BranchSide side) {
    this.elevatorPosition = pos;
    this.branchSide = side;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotPosition.Instance.elevator = elevatorPosition;
    RobotPosition.Instance.side = branchSide;
    RobotPosition.Instance.time = Timer.getFPGATimestamp();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
