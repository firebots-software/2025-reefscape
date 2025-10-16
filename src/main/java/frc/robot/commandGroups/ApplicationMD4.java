package frc.robot.commandGroups;

import java.util.List;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LandmarkPose;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.FunnelCommands.RunFunnelOutCommand;
import frc.robot.commands.FunnelCommands.isCoralInTootsieSlide;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

/*
 * 
 * 
 * Task Card:
 * You will be creating a CommandGroup that does the following:
 * 1. Intakes the Coral
 * 2. Elevator goes to L4
 * 3. Elevator goes to L2
 * 4. Elevator goes to L3
 * 5. Elevator goes to L2
 * 6. Elevator Shoots AND Spins the Funnel at the same time. 
 *    When the Shooting ENDS stop the Funnel
 * 7. Elevator goes to L4
 * 8. AS Elevator lowers down to L2 spin both the Funnel and Shooter
 * 
 * 
 * NOTES:
 * * Auto-intaking has been disabled 
 * * Auto-eject has been disabled
 * --> This means you have to be careful with trying to feed a coral
 * --> when there is already another coral in the robot
 * 
 * * INVESTIGATE ALL OF THE RELEVANT COMMANDS AND THEIR END BEHAVIORS
 * * BEFORE YOU BEGIN CREATING THE COMMAND GROUP
 * 
 * Relevant Commands:
 * SetElevatorLevel <-- takes care of Elevator raising to certain hegiht
 * ShootTootsieSlide <-- Shoots the coral (spins the shooter)
 * RunFunnelOutCommand <-- spins the funnel
 * 
 * 
 * * STARTING HINT: This class must extend either SequentialCommandGroup or
 * * ParallelCommandGroup
 * 
 * Good luck and refer to the MD.4 Resources, me or your Software
 * mentors if you have any questions or difficulties!
 * 
 * After you make your CommandGroup, check RobotContainer.java
 * Lines 359-360 and you will bind your CommandGroup to the 
 * custom controller board left L1 button and test that way
 * 
 */

public class ApplicationMD4 extends SequentialCommandGroup{
    public ApplicationMD4 (SwerveSubsystem driveTrain,
      TootsieSlideSubsystem shooter,
      ElevatorSubsystem elevator,
      FunnelSubsystem funnel,
      ArmSubsystem arm,
      List<LandmarkPose> autoInformation,
      LedSubsystem leds) {

        addCommands(
            new SequentialCommandGroup(
                new Intake(elevator, funnel, shooter, leds),
                new SetElevatorLevel(elevator, ElevatorConstants.ElevatorPositions.L4, new isCoralInTootsieSlide().isFinished()),
                new SetElevatorLevel(elevator, ElevatorConstants.ElevatorPositions.L2, new isCoralInTootsieSlide().isFinished()),
                new SetElevatorLevel(elevator, ElevatorConstants.ElevatorPositions.L3, new isCoralInTootsieSlide().isFinished()),
                new SetElevatorLevel(elevator, ElevatorConstants.ElevatorPositions.L2, new isCoralInTootsieSlide().isFinished()),

                new ParallelDeadlineGroup(new ShootTootsieSlide(shooter), new RunFunnelOutCommand(funnel, () -> false)),
                
                new SetElevatorLevel(elevator, ElevatorConstants.ElevatorPositions.L4, new isCoralInTootsieSlide().isFinished()),

                new ParallelCommandGroup(
                    new SetElevatorLevel(elevator, ElevatorConstants.ElevatorPositions.L2, new isCoralInTootsieSlide().isFinished()),
                    new ParallelCommandGroup(
                        new ShootTootsieSlide(shooter), new RunFunnelOutCommand(funnel, () -> false)
                    )
                )
            )
        );
    }
}
