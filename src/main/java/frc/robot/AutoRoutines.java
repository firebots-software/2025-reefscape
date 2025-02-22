package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commandGroups.AutoLiftAndShoot;
import frc.robot.commandGroups.LoadAndPutUp;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.FunnelCommands.RunFunnelUntilCheckedIn;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;
import java.util.function.BooleanSupplier;

public class AutoRoutines {
  private final AutoFactory autoFactory;
  private final SwerveSubsystem driveTrain;
  private final ElevatorSubsystem elevatorSubsystem;
  private final TootsieSlideSubsystem tootsieSlideSubsystem;
  private final FunnelSubsystem funnelSubsystem;
  private final BooleanSupplier redside;

  private final AutoRoutine routine;
  private final AutoTrajectory[] topTraj;
  private final AutoTrajectory[] middleTraj;
  private final AutoTrajectory[] bottomTraj;

  public AutoRoutines(
      AutoFactory factory,
      SwerveSubsystem driveTrain,
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      FunnelSubsystem funnelSubsystem,
      BooleanSupplier redside) {
    autoFactory = factory;
    this.driveTrain = driveTrain;
    this.elevatorSubsystem = elevatorSubsystem;
    this.tootsieSlideSubsystem = tootsieSlideSubsystem;
    this.funnelSubsystem = funnelSubsystem;
    this.redside = redside;

    

  


  }

  public AutoRoutine autoRoutine(String chosenPath) {
    AutoRoutine routine = autoFactory.newRoutine("routine");
    switch (chosenPath) {
      case "top":
        routine
            .active()
            .onTrue(
                routine
                    .trajectory("TSTART-4R")
                    .resetOdometry()
                    .andThen(autoSubCommand(routine, "TSTART-4R"))
                    .andThen(autoSubCommand(routine, "4R-THPS"))
                    .andThen(autoSubCommand(routine, "THPS-5L"))
                    .andThen(autoSubCommand(routine, "5L-THPS"))
                    .andThen(autoSubCommand(routine, "THPS-5R"))
                    .andThen(autoSubCommand(routine, "5R-THPS"))
                    .andThen(autoSubCommand(routine, "THPS-0L")));
        break;

      case "middle":
        routine
            .active()
            .onTrue(
                routine
                    .trajectory("MSTART-3R")
                    .resetOdometry()
                    .andThen(autoSubCommand(routine, "MSTART-3R")));
        break;

      case "bottom":
        routine
            .active()
            .onTrue(
                routine
                    .trajectory("BSTART-2L")
                    .resetOdometry()
                    .andThen(autoSubCommand(routine, "BSTART-2L"))
                    .andThen(autoSubCommand(routine, "2L-BHPS"))
                    .andThen(autoSubCommand(routine, "BHPS-1R"))
                    .andThen(autoSubCommand(routine, "1R-BHPS"))
                    .andThen(autoSubCommand(routine, "BHPS-1L"))
                    .andThen(autoSubCommand(routine, "1L-BHPS"))
                    .andThen(autoSubCommand(routine, "BHPS-0R")));
        break;
    }

    return routine;
  }

  /**
   * @param routine
   * @param baseCommandName
   * @return
   */
  public Command autoSubCommand(AutoRoutine routine, String baseCommandName) { // for actual robot
    BooleanSupplier pathGoesToHPS =
        () -> !(baseCommandName.contains("HPS-") || baseCommandName.contains("START-"));
    // if it has HPS- or START- the path ends at the reef and thus we will want to raise elevator
    // and shoot, else lower elevator and intake
    return Commands.parallel(
            routine.trajectory(baseCommandName).cmd(),
            Commands.sequence(
                new LoadAndPutUp(
                        elevatorSubsystem,
                        funnelSubsystem,
                        tootsieSlideSubsystem,
                        ElevatorPositions.Intake) // TEST TEST TEST
                    .onlyIf(() -> !pathGoesToHPS.getAsBoolean()),
                ((pathGoesToHPS.getAsBoolean())
                    ? new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake) // TEST THIS
                    : new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L4)))) // TEST THIS
        .andThen(
            (pathGoesToHPS.getAsBoolean())
                ? new RunFunnelUntilCheckedIn(funnelSubsystem) // TEST
                : new AutoLiftAndShoot(elevatorSubsystem, tootsieSlideSubsystem)); // TEST!!
  }
}
// new SetElevatorLevel // Intake
// new RunFunnelUntilDetectionSafe
// new TransferPieceBetweenFunnelAndElevator
// new SetElevatorLevel // L4
// new ShootTootsieSlide
// new SetElevatorLevel // Intake
