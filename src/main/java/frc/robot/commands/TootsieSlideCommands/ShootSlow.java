// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TootsieSlideCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.TootsieSlideSubsystem;

/** An example command that uses an example subsystem. */
public class ShootSlow extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private TootsieSlideSubsystem tootsieSlideSubsystem;

  public ShootSlow(TootsieSlideSubsystem subsystem) {
    tootsieSlideSubsystem = subsystem;
    addRequirements(tootsieSlideSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tootsieSlideSubsystem.shootTootsieSlow();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CoralPosition.setCoralInTootsieSlide(false);
    tootsieSlideSubsystem.stopTootsie();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
// adjust intake posotion
// try to reproduce the issue
// go into auto and then teleop
