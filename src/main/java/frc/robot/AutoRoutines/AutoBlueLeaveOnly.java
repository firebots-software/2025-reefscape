package frc.robot.AutoRoutines;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DaleCommands.ZeroArm;
import frc.robot.commands.ElevatorCommands.ZeroElevatorAcrossTimeframe;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBlueLeaveOnly extends SequentialCommandGroup {
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  public AutoBlueLeaveOnly(
      SwerveSubsystem driveTrain, ElevatorSubsystem elevator, ArmSubsystem arm) {
    addCommands(
        new ZeroElevatorAcrossTimeframe(elevator),
        new InstantCommand(() -> driveTrain.resetPose(Constants.Landmarks.blueMidAutoStart))
            .alongWith((new ZeroArm(arm))),
        new JamesHardenMovement(
            driveTrain,
            new Pose2d(new Translation2d(6.11305570602417, 4.050836891), new Rotation2d(Math.PI)),
            false));
  }
}
