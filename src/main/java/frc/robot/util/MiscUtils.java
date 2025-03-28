package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class MiscUtils {
  /**
   * @param point The Pose2d that needs to be reflected across the midline
   * @return The reflected Pose2d (Midline is at x = 8.27 field-based)
   */
  public static Pose2d reflectAcrossMidline(Pose2d point) {
    return new Pose2d(
        point.getX() - (2.0 * (point.getX() - Constants.Landmarks.MIDLINE_X)),
        point.getY(),
        new Rotation2d(Math.PI - point.getRotation().getRadians()));
  }

  public static Translation2d reflectAcrossMidline(Translation2d point) {
    return new Translation2d(
        point.getX() - (2.0 * (point.getX() - Constants.Landmarks.MIDLINE_X)), point.getY());
  }

  public static Pose2d plus(Pose2d a, Transform2d b) {
    return new Pose2d(
        a.getX() + b.getX(),
        a.getY() + b.getY(),
        new Rotation2d(a.getRotation().getRadians() + b.getRotation().getRadians()));
  }

  public static Pose2d plus(Pose2d a, Translation2d b) {
    return new Pose2d(
        a.getX() + b.getX(), a.getY() + b.getY(), new Rotation2d(a.getRotation().getRadians()));
  }

  // public static double lerp(double t, double a, double b, double c) {
  //   if (t > 1) return c;
  //   t = Math.max(0, Math.min(1, t));

  //   return a + (t * (b - a));
  // }

  public static double lerp(double t, double a, double b) {
    t = Math.max(0, Math.min(1, t));

    return a + (t * (b - a));
  }
}
