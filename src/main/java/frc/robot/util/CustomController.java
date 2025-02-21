package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CustomController extends GenericHID {
    Trigger LeftL1, LeftL2, LeftL3, LeftL4;
    Trigger RightL1, RightL2, RightL3, RightL4;
    Trigger Eject;
    Trigger In, Out;

    public CustomController(int port){
        super(port);
        
        LeftL1 = new Trigger(() -> this.getRawButton(0));
        LeftL2 = new Trigger(() -> this.getRawButton(1));
        LeftL3 = new Trigger(() -> this.getRawButton(2));
        LeftL4 = new Trigger(() -> this.getRawButton(3));
        RightL1 = new Trigger(() -> this.getRawButton(7));
        RightL2 = new Trigger(() -> this.getRawButton(8));
        RightL3 = new Trigger(() -> this.getRawButton(9));
        RightL4 = new Trigger(() -> this.getRawButton(10));
        Eject = new Trigger(() -> this.getRawButton(4));
        In = new Trigger(() -> this.getRawButton(5));
        Out = new Trigger(() -> this.getRawButton(6));

    }

    public Trigger LeftL1() {
        return LeftL1;
    }

    public Trigger LeftL2() {
        return LeftL2;
    }

    public Trigger LeftL3() {
        return LeftL3;
    }

    public Trigger LeftL4(){
        return LeftL4;
    }

    public Trigger RightL1() {
        return RightL1;
    }

    public Trigger RightL2() {
        return RightL2;
    }

    public Trigger RightL3() {
        return RightL3;
    }

    public Trigger RightL4() {
        return RightL4;
    }

    public Trigger Eject() {
        return Eject;
    }

    public Trigger In() {
        return In;
    }

    public Trigger Out() {
        return Out;
    }
}
