// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RunFunnelUntilDetection;
import frc.robot.commands.TransferPieceBetweenFunnelAndElevator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.commands.RunElevatorUpWithCoral;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LoadAndPutUp extends SequentialCommandGroup {
  RunElevatorUpWithCoral tootsieSlide;
  RunFunnelUntilDetection funnel;
  TransferPieceBetweenFunnelAndElevator transferer;
  public LoadAndPutUp(FunnelSubsystem funnelSubsystem, ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    
    addCommands(new RunFunnelUntilDetection(funnelSubsystem));
    addCommands(new TransferPieceBetweenFunnelAndElevator(elevatorSubsystem, funnelSubsystem));
    addCommands(new RunElevatorUpWithCoral(elevatorSubsystem, funnelSubsystem));
  }
}
