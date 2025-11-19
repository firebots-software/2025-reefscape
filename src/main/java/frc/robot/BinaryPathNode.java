package frc.robot;

import java.util.function.BooleanSupplier;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;

public class BinaryPathNode {
    private String name;
    private Command command;
    private BinaryPathNode trueChild;
    private BinaryPathNode falseChild;
    private BooleanSupplier conditionToCheck;

    public BinaryPathNode(String name, Command command) {
        this.name = name;
        this.command = command;
        
        trueChild = null;
        falseChild = null;
        conditionToCheck = () -> true;
    }

    public BinaryPathNode withChildren(BinaryPathNode trueChild) {
        this.trueChild = trueChild;
        this.falseChild = null;
        conditionToCheck = () -> true;
        return this;
    }

    public BinaryPathNode withChildren(BooleanSupplier condition, BinaryPathNode trueChild, BinaryPathNode falseChild) {
        this.trueChild = trueChild;
        this.falseChild = falseChild;
        conditionToCheck = condition;
        return this;
    }

    public BinaryPathNode nextPath() {
        return conditionToCheck.getAsBoolean() ? trueChild : falseChild;
    }

    public String getName() {
        return name;
    }

    public Command command() {
        return command;
    }
}
