package frc.robot.commands.AbstractedPID;

import java.lang.reflect.Array;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class IncreasePArm extends Command {

  private boolean prev = false;

  private boolean pidChanged = false;

  private boolean mechChanged = false;

  private int[] broom = {1, 2, 3, 4, 5};
  private int index = 0;
  private boolean mechStuffNoCap = false;

  BooleanSupplier increaseFunction;
  BooleanSupplier decreaseFunction;
  BooleanSupplier pidChange;
  BooleanSupplier mechChange;

  private CommandXboxController joystick;
  
  public IncreasePArm(BooleanSupplier increaseFunction, BooleanSupplier decreaseFunction, BooleanSupplier pidChange, BooleanSupplier mechChange) {
      this.joystick = joystick;
      this.increaseFunction = increaseFunction;
      this.decreaseFunction = decreaseFunction;
      this.pidChange = pidChange;
      this.mechChange = mechChange;
      prev = increaseFunction.getAsBoolean();
        
  }

  @Override
  public void initialize() {

    if(prev != increaseFunction.getAsBoolean() && increaseFunction.getAsBoolean() == true){
      IndexStuffNoCap(broom);
    }
    prev = increaseFunction.getAsBoolean();
  }

  public void IndexStuffNoCap(int[] broom2){
      index = broom2[index];

      if(index == 1){
        Constants.Arm.S0C_KP += 0.1;
      }
      else if (index == 2) {
        Constants.Arm.S0C_KI += 0.1;
      }

      else if (index == 3) {
        Constants.Arm.S0C_KD +=0.1;
      }

      else if (index == 4) {
        //static stuff changes
      }

      else if (index == 5) {
        //gravity?
      }
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
