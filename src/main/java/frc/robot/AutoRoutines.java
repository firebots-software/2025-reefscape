package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commandGroups.Intake;
import frc.robot.commandGroups.JamesHardenScore;
import frc.robot.commands.AutoCommands.SetIsAutoRunningToFalse;
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

  private static boolean isAutoRunning = true;

  public AutoRoutines(
      AutoFactory factory,
      SwerveSubsystem driveTrain,
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      FunnelSubsystem funnelSubsystem,
      BooleanSupplier redside) {

    this.autoFactory = factory;
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

    for (String n : topNames) {
      topTraj.add(routine.trajectory(n));
    }
    for (String n : middleNames) {
      middleTraj.add(routine.trajectory(n));
    }
    for (String n : bottomNames) {
      bottomTraj.add(routine.trajectory(n));
    }
  }

  public AutoRoutine simpleTest() {
    SequentialCommandGroup autoCommandGroup = new SequentialCommandGroup();

    autoCommandGroup.addCommands(bottomTraj.get(0).resetOdometry());

    // Add all the auto segments as commands
    for (int i = 0; i < bottomNames.size(); i++) {
      autoCommandGroup.addCommands(bottomTraj.get(i).cmd());
    }

    // Set isAutoRunning to false when auto routine finishes
    autoCommandGroup.addCommands(new SetIsAutoRunningToFalse());

    // Bind the Auto SequentialCommandGroup to run when the routine is activated
    routine.active().onTrue(autoCommandGroup);

    DogLog.log("Auto/Simple-Test-Constructor", "Ran");
    // DogLog.log("Auto/Returning-Num-Paths", numPaths);

    return routine;
  }

  public AutoRoutine autoRoutine(String chosenAuto) {
    SequentialCommandGroup autoCommandGroup = new SequentialCommandGroup();
    int numPaths; // Number of trajectories (segments) in the chosen Auto routine

    // Set the first command in the AutoCommandGroup to reset the Odometry to the start Pose of the
    // first trajectory in Choreo
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
        throw new Error(
            "AUTO ERROR: The SmartDashboard SendableChooser for Auto (top/middle/bottom) was incorrect in autoRoutine()");
    }

    // Add all the auto segments as commands
    for (int i = 0; i < numPaths; i++) {
      autoCommandGroup.addCommands(autoSubCommand(chosenAuto, i));
    }

    // Set isAutoRunning to false when auto routine finishes
    autoCommandGroup.addCommands(new SetIsAutoRunningToFalse());

    // Bind the Auto SequentialCommandGroup to run when the routine is activated
    routine.active().onTrue(autoCommandGroup);

    DogLog.log("Auto/Returning-Auto-Routine", chosenAuto);
    DogLog.log("Auto/Returning-Num-Paths", numPaths);

    return routine;
  }

  /**
   * AutoSubCommand creates a Command Group, which is a combination of the robot's swerve motion and
   * necessary mechanism action.
   *
   * @param chosenAuto - A String representing whether the top, middle, or bottom auto routine was
   *     selected. Passed on as a parameter inside autoRoutine().
   * @param index - An int representing which trajectory/segment of the auto routine to create a
   *     command for. Corresponds to the ArrayLists of trajectory names and AutoTrajectories created
   *     in the constructor.
   */
  public SequentialCommandGroup autoSubCommand(String chosenAuto, int index) {
    /*
    AutoSubCommand creates a Command Group, which is a combination of the robot's swerve motion and necessary mechanism action.

    ----- Old Structure (not using configureBindings()): -----

    Sequential (
      Parallel (
        Follow Trajectory,
        Sequential (
          Intake-To-Tootsie (if Start or leaving HPS)
          Elevator: Intake (if going to HPS) or L4 (if Start or leaving HPS)
        )
      ),
      Wait-For-Coral / Shoot-Coral
    )

    ----- New Structure 1 (using configureBindings()): -----

    (actually i don't think making an Auto that relies on the default bindings is ideal or even possible;
    we would have to alter the triggers and commands in configureBindings() a lot in order to get it to work properly)

    ----- New Structure 2 (not using configureBindings()): -----

    Sequential (
      Intake-To-Tootsie (if Start or leaving HPS),
      Elevator: Safe Position, // we only want to move with the elevator down at the Safe position
      Follow Trajectory,
      if going to HPS (
        Parallel (
          Elevator: Intake // start moving Elevator to Intake to get ready for incoming Coral at HPS
          Wait-For-Coral-Checkin // waiting for a Coral to hit Checkin sensor at the HPS
        )
      )
      else if Start or leaving HPS (
        Sequential (
          Elevator: L4
          Shoot-Coral
        )
      )
    )

    */

    String trajName; // Name of the .traj file that corresponds to chosenAuto and index
    AutoTrajectory trajectory; // Corresponding AutoTrajectory from the ArrayList initialized in the
    // constructor

    // Set trajName and trajectory based on chosenAuto and index
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

      default:
        throw new Error(
            "AUTO ERROR: The SmartDashboard SendableChooser for Auto (top/middle/bottom) was incorrect in autoSubCommand()");
    }

    // boolean pathIsStart = trajName.contains("START-");
    BooleanSupplier pathGoesToHPS =
        () -> !(trajName.contains("HPS-") || trajName.contains("START-"));
    BooleanSupplier startOrLeavingHPS = () -> !pathGoesToHPS.getAsBoolean();
    boolean goRightBranch = trajName.substring(trajName.length() - 1).equals("R");

    DogLog.log("Auto/trajName", trajName);
    DogLog.log("Auto/pathGoesToHPS", pathGoesToHPS.getAsBoolean());

    // See Structure description comment above for a sort-of better explanation
    SequentialCommandGroup newStructure2 =
        new SequentialCommandGroup(
            new Intake(elevatorSubsystem, funnelSubsystem, tootsieSlideSubsystem)
                .onlyIf(startOrLeavingHPS),
            new SetElevatorLevel(
                elevatorSubsystem,
                ElevatorPositions
                    .L1), // using L1 as the Safe Position because not sure if the "pos" value in
            // the Constants Enum should be 0 or 1
            trajectory.cmd(), // actual robot movement
            (pathGoesToHPS.getAsBoolean()
                ? new ParallelCommandGroup(
                    new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake),
                    new RunFunnelUntilCheckedIn(funnelSubsystem))
                : new JamesHardenScore(
                    elevatorSubsystem,
                    tootsieSlideSubsystem,
                    driveTrain,
                    ElevatorPositions.L4,
                    redside,
                    goRightBranch)));

    // Command oldStructure =  Commands.parallel(
    //         trajectory.cmd(),
    //         Commands.sequence(
    //             new LoadAndPutUp(
    //                     elevatorSubsystem,
    //                     funnelSubsystem,
    //                     tootsieSlideSubsystem,
    //                     ElevatorPositions.Intake)
    //                 .onlyIf(() -> !pathGoesToHPS.getAsBoolean()), // LoadAndPutUp(Intake) only if
    // the path does NOT go to the HPS. Should run on Start path
    //             ((pathGoesToHPS.getAsBoolean())
    //                 ? new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake)
    //                 : new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L4))))
    //     .andThen(
    //         (pathGoesToHPS.getAsBoolean())
    //             ? new RunFunnelUntilCheckedIn(funnelSubsystem)
    //             : new AutoLiftAndShoot(elevatorSubsystem, tootsieSlideSubsystem));

    DogLog.log("Auto/AutoSubCommand-ran", true);

    return newStructure2;
  }

  public static void setIsAutoRunning(boolean running) {
    isAutoRunning = running;
  }

  public static boolean getIsAutoRunning() {
    return isAutoRunning;
  }
}

// Commands we should use for Auto:
// new SetElevatorLevel // Intake
// new RunFunnelUntilDetectionSafe
// new TransferPieceBetweenFunnelAndElevator
// new SetElevatorLevel // L4
// new ShootTootsieSlide
// new SetElevatorLevel // Intake
