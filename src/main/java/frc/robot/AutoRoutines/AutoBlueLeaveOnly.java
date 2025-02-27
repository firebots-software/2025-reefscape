package frc.robot.AutoRoutines;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBlueLeaveOnly extends SequentialCommandGroup {
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  public AutoBlueLeaveOnly(SwerveSubsystem driveTrain) {
    addCommands(
        new InstantCommand(() -> driveTrain.resetPose(Constants.Landmarks.blueMidAutoStart)),
        new JamesHardenMovement(
            driveTrain,
            new Pose2d(new Translation2d(6.11305570602417, 4.050836891), new Rotation2d(Math.PI)),
            false));
  }
}
