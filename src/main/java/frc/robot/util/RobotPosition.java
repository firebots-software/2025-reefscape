package frc.robot.util;

import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.Landmarks.BranchSide;

public class RobotPosition {
    public static RobotPosition Instance;

    public ElevatorPositions elevator;
    public BranchSide side;
    public double time;

    public RobotPosition(ElevatorPositions elevator, BranchSide side, double time){
        this.elevator = elevator;
        this.side=side;
        this.time = time;
    }
}