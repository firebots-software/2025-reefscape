package frc.robot.util;

import com.ctre.phoenix6.hardware.Pigeon2;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.subsystems.SwerveSubsystem;

public class GyroStabilizer {
  private static SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();
  private static Pigeon2 pigeon = driveTrain.getPigeon2();

  //  getRotation3d() returns a Rotation3d object, which contains the roll, pitch, and yaw of the
  // robot
  /*  Rotation3d important methods
     double    getX():
         Returns the counterclockwise rotation angle around the X axis (roll) in radians.
     double    getY():
         Returns the counterclockwise rotation angle around the Y axis (pitch) in radians.
     double    getZ():
         Returns the counterclockwise rotation angle around the Z axis (yaw) in radians.
  */

  /* ideas
   *   private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
   *
   *   and
   *
   *   robotCentricDrive.withVelocityX(x).withVelocityY(y).withRotationalRate(turn);
   */

  /*
   * For Auto, I don't think we can use the RobotCentric Swerve Request to move the robot, since Choreo is already using the
   * followTrajectory() method in SwerveSubsystem to apply a ChassisSpeeds to the robot. That's why we need to use the
   * "tip vector" calculated in this class to adjust the ChassisSpeeds in followTrajectory() in  order to implement tip prevention
   * during Auto.
   */

  public static Transform2d getTipVector() {
    double roll = pigeon.getRotation3d().getX();
    double pitch = pigeon.getRotation3d().getY();

    DogLog.log("gyroStabilizer/roll", roll);
    DogLog.log("gyroStabilizer/pitch", pitch);

    return new Transform2d();
  }
}
