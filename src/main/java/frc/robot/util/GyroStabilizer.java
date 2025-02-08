package frc.robot.util;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class GyroStabilizer extends Command {
  public static final double TIP_THRESHOLD = .5;  

  private SwerveSubsystem swerveSubsystem;
  private Pigeon2 pigeon;

  private final SwerveRequest.RobotCentric robotCentricDrive =
    new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

  public GyroStabilizer(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    pigeon = swerveSubsystem.getPigeon2();
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transform2d currentTipVectorXY = getTipVectorXY(pigeon);
    Transform2d currentTipVectorRP = getTipVectorRP(pigeon);
    SwerveRequest drive = robotCentricDrive.withVelocityX(currentTipVectorRP.getX() > 0 ? -currentTipVectorXY.getX() : currentTipVectorXY.getX())
                                           .withVelocityY(currentTipVectorRP.getY() > 0 ? -currentTipVectorXY.getY() : currentTipVectorXY.getY());
    swerveSubsystem.setControl(drive);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return magnitudeTipVector(getTipVectorXY(pigeon)) < TIP_THRESHOLD;
  }

  public static Transform2d getTipVectorRP(Pigeon2 pigeon) {
    double roll = pigeon.getRotation3d().getX();
    double pitch = pigeon.getRotation3d().getY();

    DogLog.log("gyroStabilizer/roll", roll);
    DogLog.log("gyroStabilizer/pitch", pitch);

    return new Transform2d(roll, pitch, Rotation2d.kZero);
  }

  public static Transform2d getTipVectorXY(Pigeon2 pigeon) {
    double roll = pigeon.getRotation3d().getX();
    double pitch = pigeon.getRotation3d().getY();

    DogLog.log("gyroStabilizer/xComponent", Math.sin(roll));
    DogLog.log("gyroStabilizer/yComponent", Math.sin(pitch));

    return new Transform2d(Math.sin(roll), Math.sin(pitch), Rotation2d.kZero);
  }

  public static double magnitudeTipVector(Transform2d tipVector) {
    return Math.sqrt(tipVector.getX() * tipVector.getX() + tipVector.getY() * tipVector.getY());
  }
}
