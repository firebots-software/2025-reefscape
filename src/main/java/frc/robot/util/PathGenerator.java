package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;

public class PathGenerator {
    public static List<Waypoint> generatePath(Pose2d start, Pose2d end, Pose2d circleOfDoom, float doomSize) {
        float resolution = .1f;
        end.minus(start);
        return new ArrayList<Waypoint>();
    }
}
