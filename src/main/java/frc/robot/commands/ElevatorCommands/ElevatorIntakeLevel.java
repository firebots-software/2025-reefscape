// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;

/** An example command that uses an example subsystem. */
public class ElevatorIntakeLevel extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem elevatorSubsystem;

  private final FunnelSubsystem funnelSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorIntakeLevel(ElevatorSubsystem subsystem, FunnelSubsystem funnel) {
    elevatorSubsystem = subsystem;
    this.funnelSubsystem = funnel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  public void execute() {
    if (!funnelSubsystem.isCoralCheckedOut()) {
      elevatorSubsystem.elevateTo(ElevatorPositions.Intake);
    } else {
      elevatorSubsystem.elevateTo(ElevatorPositions.L1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorSubsystem.atTargetPosition();
  }
}
