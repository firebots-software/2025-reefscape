package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommands.ZeroElevator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

public class EjectCoralThenZero extends SequentialCommandGroup {
  public EjectCoralThenZero(ElevatorSubsystem elevatorSubsystem, TootsieSlideSubsystem tootsieSlideSubsystem, FunnelSubsystem funnelSubsystem) {
    Command a = null;
    if (funnelSubsystem.drakeTripped()) { 
      a = new EjectCoralFR(elevatorSubsystem, tootsieSlideSubsystem);  
    }
    addCommands(
      a,                          
      new ZeroElevator(elevatorSubsystem)  
    );
  }
}
