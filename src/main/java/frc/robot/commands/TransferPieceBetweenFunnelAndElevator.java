// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TransferPieceBetweenFunnelAndElevator extends Command {

  private ElevatorSubsystem elevatorSubsystem;
  private FunnelSubsystem funnelSubsystem;
  private TootsieSlideSubsystem tootsieSlideSubsystem;
  public TransferPieceBetweenFunnelAndElevator(
      ElevatorSubsystem elevatorSubsystem, FunnelSubsystem funnelSubsystem, TootsieSlideSubsystem tootsieSlideSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.funnelSubsystem = funnelSubsystem;
    this.tootsieSlideSubsystem=tootsieSlideSubsystem;

    addRequirements(elevatorSubsystem,funnelSubsystem,tootsieSlideSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.elevateTo(ElevatorPositions.Intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevatorSubsystem.isAtPosition()) {
      funnelSubsystem.spinFunnel();
      tootsieSlideSubsystem.intakeCoral();
    } else {
      funnelSubsystem.maintainCurrentPosition();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    funnelSubsystem.setCoralInFunnel(false);
    funnelSubsystem.stopFunnel();
    tootsieSlideSubsystem.stopTootsie();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return funnelSubsystem.drakeTripped();
  }
}
