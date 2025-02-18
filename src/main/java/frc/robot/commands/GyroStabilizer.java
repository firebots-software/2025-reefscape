package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class GyroStabilizer extends Command {
  public static final double TIP_THRESHOLD = 5 * (Math.PI / 180); // TIP_THRESHOLD is in radians :)

  private SwerveSubsystem swerveSubsystem;
  private static Pigeon2 pigeon;

  private static PIDController pidController = new PIDController(1, 0.0, 0.0); // TODO: tune PID

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
    Transform2d currentTipVectorRP = getTipVectorRP(pigeon);

    double dirToGoX = currentTipVectorRP.getY(); // TODO: change for comp robot
    double dirToGoY = -currentTipVectorRP.getX();

    double xSpeed = pidController.calculate(dirToGoX, 0);
    double ySpeed = pidController.calculate(dirToGoY, 0);

    SwerveRequest drive = robotCentricDrive.withVelocityX(xSpeed).withVelocityY(ySpeed);

    DogLog.log("gyroStabilizer/xSpeed", xSpeed);
    DogLog.log("gyroStabilizer/ySpeed", ySpeed);

    swerveSubsystem.setControl(drive);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !tipping(swerveSubsystem);
  }

  public static Transform2d getTipVectorRP(Pigeon2 pigeon) {
    double roll = pigeon.getRotation3d().getX();
    double pitch = pigeon.getRotation3d().getY();

    DogLog.log("gyroStabilizer/roll", roll);
    DogLog.log("gyroStabilizer/pitch", pitch);

    return new Transform2d(roll, pitch, Rotation2d.kZero);
  }

  public static double magnitudeTipVector(Transform2d tipVector) {
    return Math.sqrt(tipVector.getX() * tipVector.getX() + tipVector.getY() * tipVector.getY());
  }

  public static boolean tipping(SwerveSubsystem driveTrain) {
    // Old version checks if the magnitude of the vector is greater than a TIP_THRESHOLD of 0.1.
    // return
    // GyroStabilizer.magnitudeTipVector(GyroStabilizer.getTipVectorRP(driveTrain.getPigeon2()))
    //     > TIP_THRESHOLD;

    return GyroStabilizer.getTipVectorRP(pigeon).getX() > TIP_THRESHOLD
        || GyroStabilizer.getTipVectorRP(pigeon).getY() > TIP_THRESHOLD;
  }
}
