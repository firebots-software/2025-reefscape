// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.SetElevatorLevel;
import frc.robot.commands.TootsieSlideShooting;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class ElevatorAndShooting extends SequentialCommandGroup {
  TootsieSlideShooting tootsieSlideShooting;

  public ElevatorAndShooting(
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      ElevatorPositions level) {
    addCommands(new SetElevatorLevel(elevatorSubsystem, level));
    addCommands(new TootsieSlideShooting(tootsieSlideSubsystem));
  }
}
