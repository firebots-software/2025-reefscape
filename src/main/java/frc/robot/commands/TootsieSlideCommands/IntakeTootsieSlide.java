// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TootsieSlideCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeTootsieSlide extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private TootsieSlideSubsystem tootsieSlideSubsystem;
  private FunnelSubsystem funnelSubsystem;

  public IntakeTootsieSlide(TootsieSlideSubsystem subsystem, FunnelSubsystem funnelSubsystem) {
    tootsieSlideSubsystem = subsystem;
    this.funnelSubsystem = funnelSubsystem;
    addRequirements(tootsieSlideSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tootsieSlideSubsystem.intakeCoral();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CoralPosition.setCoralInTootsieSlide(true);
    tootsieSlideSubsystem.stopTootsie();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return funnelSubsystem.drakeTripped();
  }
}
