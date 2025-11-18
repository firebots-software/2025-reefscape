package frc.robot;

import java.util.ArrayList;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutineUtils {

    private final AutoFactory autoFactory;

    private BinaryPathNode pathHead;
    private AutoRoutine routine;

    public AutoRoutineUtils(BinaryPathNode pathHead, AutoSubCommandHolder autoSubCommandHolder, AutoFactory factory) {
        autoFactory = factory;
        this.pathHead = pathHead;

        this.routine = autoFactory.newRoutine("routine");
    }
}
