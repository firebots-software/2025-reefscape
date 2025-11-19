package frc.robot;

import java.util.ArrayList;
import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoRoutineUtils {

    private final AutoFactory autoFactory;
    private AutoRoutine routine;

    public AutoRoutineUtils(AutoFactory factory) {
        autoFactory = factory;

        this.routine = autoFactory.newRoutine("routine");
    }

    public AutoRoutine getRoutine() {
        return routine;
    }

    public Command commandFromHead(Supplier<BinaryPathNode> head) {
      if (head.get().nextPath() == null) return new InstantCommand();
      return head.get().command().andThen(commandFromHead(() -> head.get().nextPath()));
    }
}
