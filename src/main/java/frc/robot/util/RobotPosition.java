package frc.robot.util;

import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.Landmarks.BranchSide;

public class RobotPosition {
    public static RobotPosition Instance;

    public ElevatorPositions elevator;
    public BranchSide side;
    public double dTime;

    public RobotPosition(ElevatorPositions elevator, BranchSide side, double dTime){
        this.elevator = elevator;
        this.side=side;
        this.dTime = dTime;
    }
}