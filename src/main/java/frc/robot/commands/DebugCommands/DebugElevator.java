package frc.robot.commands.DebugCommands;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class DebugElevator extends Command{

  private final ElevatorSubsystem elevatorSubsystem;

    public DebugElevator(ElevatorSubsystem subsystem){
      this.elevatorSubsystem = subsystem;
      addRequirements(elevatorSubsystem);
    }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevatorSubsystem.testElevator(50);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.atTargetPosition();
  }
}
