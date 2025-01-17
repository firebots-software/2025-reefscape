// package frc.robot.commands;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.IdealStartingState;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.Waypoint;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.util.MiscUtils;
// import java.util.List;
// import java.util.function.Supplier;

// public class MoveToTarget extends Command {
//   private Pose2d[] pointDirs;
//   private SwerveSubsystem swerve;
//   private Supplier<Boolean> reflected;
//   private Rotation2d goalEndRot, startDir;
//   private double goalEndVelo;
//   PathPlannerPath constructedPath;
//   Command pathCommand;

//   private MoveToTarget(
//       SwerveSubsystem swerve,
//       Pose2d[] pointDirs,
//       Supplier<Boolean> reflected,
//       double goalEndVelo,
//       Rotation2d goalEndRot,
//       Rotation2d startDir) {
//     this.pointDirs = pointDirs;
//     this.swerve = swerve;
//     this.reflected = reflected;
//     this.goalEndVelo = goalEndVelo;
//     this.goalEndRot =
//         (reflected.get()) ? new Rotation2d(Math.PI - goalEndRot.getRadians()) : goalEndRot;
//     if (startDir != null) {
//       this.startDir =
//           (reflected.get()) ? new Rotation2d(Math.PI - startDir.getRadians()) : startDir;
//     } else {
//       this.startDir = null;
//     }
//     addRequirements(swerve);
//   }

//   private MoveToTarget(
//       SwerveSubsystem swerve,
//       Pose2d pointDirs,
//       Supplier<Boolean> reflected,
//       double goalEndVelo,
//       Rotation2d goalEndRot,
//       Rotation2d startDir) {
//     this(swerve, new Pose2d[] {pointDirs}, reflected, goalEndVelo, goalEndRot, startDir);
//   }

//   @Override
//   public void initialize() {
//     // constructing the list of path points using absolute coordinates on the field

//     Pose2d[] poseArray = new Pose2d[pointDirs.length + 1];
//     poseArray[0] =
//         new Pose2d(
//             swerve.getState().Pose.getTranslation(),
//             (startDir == null) ? swerve.getState().Pose.getRotation() : startDir);
//     for (int i = 1; i < poseArray.length; i++) {
//       poseArray[i] =
//           (reflected.get()
//               ? MiscUtils.reflectAcrossMidline(this.pointDirs[i - 1])
//               : this.pointDirs[i - 1]);
//     }

//     List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(poseArray);

//     // create the path using the path points and constraints (also providing the final robot
//     // heading)
//     constructedPath =
//         new PathPlannerPath(
//             bezierPoints,
//             Constants.Swerve.PPConstants.PATH_PLANNER_CONSTRAINTS,
//             new IdealStartingState(0, new Rotation2d()),
//             new GoalEndState(goalEndVelo, goalEndRot) // goal end velocity and heading
//             );

//     // prevent automatic path flipping by AutoBuilder (we want to execute absolute path)
//     constructedPath.preventFlipping = true;

//     // Command of the built auto path
//     pathCommand = AutoBuilder.followPath(constructedPath);
//     pathCommand.initialize();
//   }

//   public void execute() {
//     if (pathCommand != null) {
//       pathCommand.execute();
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     if (pathCommand != null) {
//       pathCommand.end(interrupted);
//     }
//     SmartDashboard.putString("ended", "yes");
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return pathCommand != null ? pathCommand.isFinished() : false;
//   }

//   // Factory pattern (two separate constructors that invoke the mirror / non mirror)
//   public static Command withAbsolute(
//       SwerveSubsystem swerve, Rotation2d goalEndRot, Rotation2d startDir, Pose2d... absolutePose) {
//     return new MoveToTarget(swerve, absolutePose, () -> false, 0, goalEndRot, startDir);
//   }

//   public static Command withAbsolute(
//       SwerveSubsystem swerve,
//       Rotation2d goalEndRot,
//       Rotation2d startDir,
//       double goalEndVelo,
//       Pose2d... absolutePose) {
//     return new MoveToTarget(swerve, absolutePose, () -> false, goalEndVelo, goalEndRot, startDir);
//   }

//   public static Command withMirror(
//       SwerveSubsystem swerve,
//       Supplier<Boolean> mirror,
//       Rotation2d goalEndRot,
//       Rotation2d startDir,
//       Pose2d... absolutePose) {
//     return new MoveToTarget(swerve, absolutePose, mirror, 0, goalEndRot, startDir);
//   }

//   public static Command withMirror(
//       SwerveSubsystem swerve,
//       Supplier<Boolean> mirror,
//       Rotation2d goalEndRot,
//       Rotation2d startDir,
//       double goalEndVelo,
//       Pose2d... absolutePose) {
//     return new MoveToTarget(swerve, absolutePose, mirror, goalEndVelo, goalEndRot, startDir);
//   }
// }
