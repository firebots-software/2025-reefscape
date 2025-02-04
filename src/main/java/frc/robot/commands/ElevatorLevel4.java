package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorLevel4 extends Command {
  private final ElevatorSubsystem m_subsystem;

  public ElevatorLevel4(ElevatorSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.elevate(ElevatorPositions.L4);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_subsystem.atTargetPosition(ElevatorPositions.L1);
  }
}
