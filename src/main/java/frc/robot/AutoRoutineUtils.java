package frc.robot;

import java.util.ArrayList;
import java.util.function.BinaryOperator;
import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DebugCommands.DogLogCmd;

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

    public Command commandFromHead(BinaryPathNode head) {
      return head.command().andThen(new ConditionalCommand(
        head.trueChild() == null ? new InstantCommand() : commandFromHead(head.trueChild()), 
        head.falseChild() == null ? new InstantCommand() : commandFromHead(head.falseChild()), 
        head.condition()));
    }
}
