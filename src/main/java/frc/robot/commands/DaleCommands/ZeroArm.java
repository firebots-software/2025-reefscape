// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DaleCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroArm extends Command {
  /** Creates a new ZeroArm. */
  ArmSubsystem arm;

  int timesExceededCurrent;

  public ZeroArm(ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    timesExceededCurrent = 0;
    addRequirements(this.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.moveMuyNegative(); // this works no, it just hits dale against the hardstop
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      arm.zeroSensor();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (arm.checkCurrent()) {
      timesExceededCurrent++;
    } else {
      timesExceededCurrent = 0;
    }

    return timesExceededCurrent >= 10;
  }
}
