// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.ElevatorCommands.ElevatorIntakeLevel;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.FunnelCommands.RunFunnelUntilDetection;
import frc.robot.commands.TransferPieceBetweenFunnelAndElevator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LoadAndPutUp extends SequentialCommandGroup {

  public LoadAndPutUp(
      ElevatorSubsystem elevator,
      FunnelSubsystem funnel,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      ElevatorPositions level) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ElevatorIntakeLevel(elevator, funnel).andThen(new RunFunnelUntilDetection(funnel)));
    addCommands(new TransferPieceBetweenFunnelAndElevator(elevator, funnel, tootsieSlideSubsystem));
    addCommands(new SetElevatorLevel(elevator, level));
  }
}
