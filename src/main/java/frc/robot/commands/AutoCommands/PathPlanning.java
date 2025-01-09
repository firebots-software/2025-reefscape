package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj2.command.Command;

public class PathPlanning {
    // Load the path we want to pathfind to and follow
PathPlannerPath path = PathPlannerPath.fromPathFile("reef");

// Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

// Since AutoBuilder is configured, we can use it to build pathfinding commands
Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
        path,
        constraints);
    
}
