package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FunnelCommands.RampUpFunnel;
import frc.robot.commands.FunnelCommands.RunFunnelUntilDetectionSafe;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;

public class RunFunnelUntilDetectionSafeSmooth extends SequentialCommandGroup {
  public RunFunnelUntilDetectionSafeSmooth(
      ElevatorSubsystem elevatorSubsystem, FunnelSubsystem funnelSubsystem) {
    addCommands(
        new WaitCommand(0.2),
        new RampUpFunnel(funnelSubsystem, elevatorSubsystem).withTimeout(0.2),
        new RunFunnelUntilDetectionSafe(funnelSubsystem, elevatorSubsystem));
  }
}
