package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commandGroups.AutoLiftAndShoot;
import frc.robot.commandGroups.LoadAndPutUp;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.FunnelCommands.RunFunnelUntilCheckedIn;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class AutoRoutines {
  private final AutoFactory autoFactory;
  private final SwerveSubsystem driveTrain;
  private final ElevatorSubsystem elevatorSubsystem;
  private final TootsieSlideSubsystem tootsieSlideSubsystem;
  private final FunnelSubsystem funnelSubsystem;
  private final BooleanSupplier redside;

  private AutoRoutine routine;
  private ArrayList<String> topNames;
  private ArrayList<String> middleNames;
  private ArrayList<String> bottomNames;
  private ArrayList<AutoTrajectory> topTraj;
  private ArrayList<AutoTrajectory> middleTraj;
  private ArrayList<AutoTrajectory> bottomTraj;

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

    this.routine = autoFactory.newRoutine("routine");

    /*
    Field Diagram

              BLUE                        RED
    ___________________________________________________
    |THPS                 TSTART                  THPS|
    |                       | |                       |
    |        5    4         | |         4    5        |
    |     0          3    MSTART     3          0     |
    |        1    2         | |         2    1        |
    |                       | |                       |
    |BHPS_________________BSTART__________________BHPS|

    L and R branches are on the left and right of the robot when it is at the reef in between the two branches

    a path name consists of a start and a destination
    a start and a destinantion can be on any one of those marked areas
    the format for a path is start-destination
    ex. 2L-BHPS
    this path starts from the left branch on the second part of the reef (2L), and goes to the bottom human player station (BHPS)
    */

    this.topNames = new ArrayList<>();
    this.topNames.add("TSTART-4R");
    this.topNames.add("4R-THPS");
    this.topNames.add("THPS-5L");
    this.topNames.add("5L-THPS");
    this.topNames.add("THPS-5R");
    this.topNames.add("5R-THPS");
    this.topNames.add("THPS-0L");

    this.middleNames = new ArrayList<>();
    this.middleNames.add("MSTART-3R");

    this.bottomNames = new ArrayList<>();
    this.bottomNames.add("BSTART-2L");
    this.bottomNames.add("2L-BHPS");
    this.bottomNames.add("BHPS-1R");
    this.bottomNames.add("1R-BHPS");
    this.bottomNames.add("BHPS-1L");
    this.bottomNames.add("1L-BHPS");
    this.bottomNames.add("BHPS-0R");

    this.topTraj = new ArrayList<>();
    this.middleTraj = new ArrayList<>();
    this.bottomTraj = new ArrayList<>();

    for(String n : topNames){
      topTraj.add(routine.trajectory(n));
    }
    for(String n : middleNames){
      middleTraj.add(routine.trajectory(n));
    }
    for(String n : bottomNames){
      bottomTraj.add(routine.trajectory(n));
    }

  }

  public AutoRoutine autoRoutine(String chosenAuto) {
    SequentialCommandGroup autoCommandGroup = new SequentialCommandGroup();
    int numPaths; // Number of trajectories (segments) in the chosen Auto routine

    // Set the first command in the AutoCommandGroup to reset the Odometry to the start Pose of the first trajectory in Choreo
    switch (chosenAuto) {
      case "top":
        autoCommandGroup.addCommands(topTraj.get(0).resetOdometry());
        numPaths = topNames.size();
        break;

      case "middle":
        autoCommandGroup.addCommands(middleTraj.get(0).resetOdometry());
        numPaths = middleNames.size();
        break;

      case "bottom":
        autoCommandGroup.addCommands(bottomTraj.get(0).resetOdometry());
        numPaths = bottomNames.size();
        break;
      default:
        throw new Error("AUTO ERROR: The SmartDashboard SendableChooser for Auto (top/middle/bottom) was incorrect in autoRoutine()");
    }

    // Add all the auto segments as commands
    for(int i = 0; i < numPaths; i++) {
      autoCommandGroup.addCommands(autoSubCommand(routine, chosenAuto, i));
    }

    // Bind the Auto SequentialCommandGroup to run when the routine is activated
    routine.active().onTrue(autoCommandGroup);

    return routine;
  }

  /**
   * @param routine
   * @param baseCommandName
   * @return
   */
  public Command autoSubCommand(AutoRoutine routine, String chosenAuto, int index) {
    /*
    AutoSubCommand creates a ParallelCommandGroup, which is a combination of the robot's swerve motion and necessary mechanism action.
     
    Structure:

    Sequential(
      Parallel (
        Follow Trajectory,
        
      ),
      Wait-For-Coral / Shoot-Coral
    )
    

    */
    String trajName;
    AutoTrajectory trajectory;

    switch (chosenAuto) {
      case "top":
        trajName = topNames.get(index);
        trajectory = topTraj.get(index);
        break;
        
      case "middle":
        trajName = middleNames.get(index);
        trajectory = middleTraj.get(index);
        break;

      case "bottom":
        trajName = bottomNames.get(index);
        trajectory = bottomTraj.get(index);
        break;
    }

    boolean pathIsStart = trajName.contains("START-");
    boolean pathGoesToHPS = !(trajName.contains("HPS-") || trajName.contains("START-"));

    return Commands.parallel(
            routine.trajectory(baseCommandName).cmd(),
            Commands.sequence(
                new LoadAndPutUp(
                        elevatorSubsystem,
                        funnelSubsystem,
                        tootsieSlideSubsystem,
                        ElevatorPositions.Intake)
                    .onlyIf(() -> !pathGoesToHPS.getAsBoolean()), // LoadAndPutUp(Intake) only if the path does NOT go to the HPS. Should run on Start path
                ((pathGoesToHPS.getAsBoolean())
                    ? new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake)
                    : new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L4))))
        .andThen(
            (pathGoesToHPS.getAsBoolean())
                ? new RunFunnelUntilCheckedIn(funnelSubsystem)
                : new AutoLiftAndShoot(elevatorSubsystem, tootsieSlideSubsystem));
  }
}
// new SetElevatorLevel // Intake
// new RunFunnelUntilDetectionSafe
// new TransferPieceBetweenFunnelAndElevator
// new SetElevatorLevel // L4
// new ShootTootsieSlide
// new SetElevatorLevel // Intake
