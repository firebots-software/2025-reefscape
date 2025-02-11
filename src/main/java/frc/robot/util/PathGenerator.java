package frc.robot.util;

import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import java.util.ArrayList;
import java.util.List;

public class PathGenerator {
  public static List<Waypoint> generatePath(
      Pose2d start, Pose2d end, Pose2d circleOfDoom, float doomSize) {
    float resolution = .1f;
    Transform2d pathSnippet = makeMagnitude(end.minus(start), resolution);
    ArrayList<Pose2d> path = new ArrayList<Pose2d>();
    path.add(start);

    int segmentAmt = (int) (magnitudeOfTransform2D(end.minus(start)) / resolution);
    for (int i = 0; i < segmentAmt; i++) {
      path.add(path.get(path.size() - 1).plus(pathSnippet));
    }
    path.add(end);

    for (Pose2d point : path) {
      if (!point.equals(start) && !point.equals(end)) {
        if (magnitudeOfTransform2D(point.minus(circleOfDoom)) < doomSize) {
          point = circleOfDoom.plus(makeMagnitude(point.minus(circleOfDoom), doomSize));
        }
      }
    }

    // add rotations properly
    // make into waypoint list

    return new ArrayList<Waypoint>();
  }

  private static double magnitudeOfTransform2D(Transform2d transform) {
    return Math.sqrt(transform.getX() * transform.getX() + transform.getY() * transform.getY());
  }

  private static Transform2d makeMagnitude(Transform2d transform, double magnitude) {
    return new Transform2d(
        transform.getX() * magnitude / magnitudeOfTransform2D(transform),
        transform.getY() * magnitude / magnitudeOfTransform2D(transform),
        transform.getRotation());
  }
}
