package frc.robot.util;

import com.ctre.phoenix6.hardware.Pigeon2;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveSubsystem;


public class GyroStabilizer {
  private static SwerveSubsystem driveTrain;
  private static Pigeon2 pigeon;
  private double rollOffset; // The number of radians the roll reads when the robot is flat. Used to zero the gyro
  private double pitchOffset; // The number of radians the pitch reads when the robot is flat. Used to zero the gyro

  public GyroStabilizer(){
    driveTrain = SwerveSubsystem.getInstance();
    pigeon = driveTrain.getPigeon2();
    
    rollOffset = pigeon.getRotation3d().getX();
    pitchOffset = pigeon.getRotation3d().getY();
  }
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

  public Translation2d getTipVector() {
    double roll = pigeon.getRotation3d().getX() - rollOffset;
    double pitch = pigeon.getRotation3d().getY() - pitchOffset;

    double xComponent = Math.sin(roll);
    double yComponent = Math.sin(pitch);

    Translation2d tipVector = new Translation2d(xComponent, yComponent);

    DogLog.log("gyroStabilizer/roll", roll);
    DogLog.log("gyroStabilizer/pitch", pitch);
    DogLog.log("gyroStabilizer/xComponent", tipVector.getX());
    DogLog.log("gyroStabilizer/yComponent", tipVector.getY());

    return tipVector;
  }
}
