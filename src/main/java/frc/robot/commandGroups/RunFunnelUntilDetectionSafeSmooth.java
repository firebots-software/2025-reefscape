package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FunnelCommands.RampUpFunnel;
import frc.robot.commands.FunnelCommands.RunFunnelUntilDetectionSafe;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.LedSubsystem;

import java.util.function.BooleanSupplier;

public class RunFunnelUntilDetectionSafeSmooth extends SequentialCommandGroup {
  public RunFunnelUntilDetectionSafeSmooth(
      ElevatorSubsystem elevatorSubsystem, FunnelSubsystem funnelSubsystem, LedSubsystem leds) {

    addCommands(
        // new WaitCommand(0.1),
        leds.updateLedsCommand(LedSubsystem.LedState.INTAKE_FLASH),
        new RampUpFunnel(funnelSubsystem, elevatorSubsystem)
            .until(
                (BooleanSupplier)
                    () -> {
                      return !funnelSubsystem.isCoralCheckedIn();
                    }),
        new WaitCommand(0.1),
        new RunFunnelUntilDetectionSafe(funnelSubsystem, elevatorSubsystem));
  }
}
