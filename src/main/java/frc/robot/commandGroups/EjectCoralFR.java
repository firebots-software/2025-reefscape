package frc.robot.commandGroups;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

public class EjectCoralFR extends SequentialCommandGroup{
    public EjectCoralFR(
        ElevatorSubsystem elevatorSubsystem,
        TootsieSlideSubsystem tootsieSlideSubsystem
    ){
        addCommands(
        new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L1).andThen(new ShootTootsieSlide(tootsieSlideSubsystem).withTimeout(0.5))
        )
        ;
    }
    
}
