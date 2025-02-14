package frc.robot.commands.AbstractedPID;

import java.lang.reflect.Array;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class IncreasePArm extends Command {

  private boolean prevInc = false;

  private boolean pidChanged = false;

  private boolean mechChanged = false;

  private int[] broom = {1, 2, 3, 4, 5};
  private boolean prevPID = false;
  private boolean prevMech = false;
  private boolean prevDec = false;
  private boolean prev4 = false;
  private int index = 0;
  private int[] mechbroom = {1,2,3};
  private int mechIndex = 0;

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
      prevInc = increaseFunction.getAsBoolean();
      prevDec = decreaseFunction.getAsBoolean();
      prevPID = pidChange.getAsBoolean();
      prevMech = mechChange.getAsBoolean();  
  }
  @Override
  public void initialize() {

    if(prevPID != pidChange.getAsBoolean() && pidChange.getAsBoolean() == true) {
      IndexPID();
          }
        prevPID = pidChange.getAsBoolean();
    
    if(prevMech != mechChange.getAsBoolean() && mechChange.getAsBoolean() == true) {
      IndexMech();
    }
         prevMech = mechChange.getAsBoolean(); 
      
    if(prevInc != increaseFunction.getAsBoolean() && increaseFunction.getAsBoolean() == true) {
            IndexStuffNoCap(broom, mechbroom);
          }
    prevInc = increaseFunction.getAsBoolean();
      
    if(prevDec != decreaseFunction.getAsBoolean() && decreaseFunction.getAsBoolean() == true) {
            IndexStuffCap(broom, mechbroom);
          }
          prevDec = increaseFunction.getAsBoolean();
    
}
      
    private void IndexPID() {
        index++;
        if(index > broom.length-1){
          index = 0;
        }
    }

      private void IndexMech() {
        mechIndex++;
        if(mechIndex > mechbroom.length-1){
          mechIndex = 0;
        }
      }
      
      public void IndexStuffNoCap(int[] broom, int[] mechbroom){
      int pid = broom[index];
      int mech = mechbroom[mechIndex];

      PIDIcreaseArm(pid, mech);
      PIDIncreaseElevator(pid, mech);
      PIDIncreaseTootsie(pid, mech);
  }

  public int getPID(){
    return broom[index];
  }

  public int getMech(){
    return mechbroom[mechIndex];
  }

public void PIDIcreaseArm(int pid2, int mech2){
if(pid2 == 1 && mech2 == 1){
        Constants.Arm.S0C_KP += 0.1;
      }
      else if (pid2  == 2 && mech2 == 1) {
        Constants.Arm.S0C_KI += 0.1;
      }

      else if (pid2 == 3 && mech2 == 1) {
        Constants.Arm.S0C_KD +=0.1;
      }

      else if (pid2  == 4 && mech2 == 1) {
        Constants.Arm.S0C_KS += 0.1;
      }

      else if (pid2 == 5 && mech2 == 1) {
        Constants.Arm.S0C_KG += 0.1;
      }
}

public void PIDDecreaseArm(int pid2, int mech2){
if(pid2 == 1 && mech2 == 1){
        Constants.Arm.S0C_KP -= 0.1;
      }
      else if (pid2  == 2 && mech2 == 1) {
        Constants.Arm.S0C_KI -= 0.1;
      }

      else if (pid2 == 3 && mech2 == 1) {
        Constants.Arm.S0C_KD -=0.1;
      }

      else if (pid2  == 4 && mech2 == 1) {
        Constants.Arm.S0C_KS -= 0.1;
      }

      else if (pid2 == 5 && mech2 == 1) {
        Constants.Arm.S0C_KG -= 0.1;
      }
}

  public void PIDIncreaseElevator(int pid2, int mech2){
if(pid2 == 1 && mech2 == 2){
        Constants.ElevatorConstants.S0C_KP += 0.1;
      }
      else if (pid2  == 2 && mech2 == 2) {
        Constants.ElevatorConstants.S0C_KI += 0.1;
      }

      else if (pid2 == 3 && mech2 == 2) {
        Constants.ElevatorConstants.S0C_KD +=0.1;
      }

      else if (pid2  == 4 && mech2 == 2) {
        Constants.ElevatorConstants.S0C_KS += 0.1;
      }

      else if (pid2 == 5 && mech2 == 2) {
        Constants.ElevatorConstants.S0C_KG += 0.1;
      }
  }

   public void PIDDecreaseElevator(int pid2, int mech2){
if(pid2 == 1 && mech2 == 2){
        Constants.ElevatorConstants.S0C_KP -= 0.1;
      }
      else if (pid2  == 2 && mech2 == 2) {
        Constants.ElevatorConstants.S0C_KI -= 0.1;
      }

      else if (pid2 == 3 && mech2 == 2) {
        Constants.ElevatorConstants.S0C_KD -=0.1;
      }

      else if (pid2  == 4 && mech2 == 2) {
        Constants.ElevatorConstants.S0C_KS -= 0.1;
      }

      else if (pid2 == 5 && mech2 == 2) {
        Constants.ElevatorConstants.S0C_KG -= 0.1;
      }
  }
  public void PIDIncreaseTootsie (int pid2, int mech2){
    
      if(pid2 == 1 && mech2 == 3){
        Constants.TootsieSlide.S0C_KP += 0.1;
      }
      else if (pid2  == 2 && mech2 == 3) {
        Constants.TootsieSlide.S0C_KI += 0.1;
      }

      else if (pid2 == 3 && mech2 == 3) {
        Constants.TootsieSlide.S0C_KD +=0.1;
      }

      else if (pid2  == 4 && mech2 == 3) {
        Constants.TootsieSlide.S0C_KS += 0.1;
      }

      else if (pid2 == 5 && mech2 == 3) {
        Constants.TootsieSlide.S0C_KG += 0.1;
      }
  }

  public void PIDDecreaseTootsie (int pid2, int mech2){
    
      if(pid2 == 1 && mech2 == 3){
        Constants.TootsieSlide.S0C_KP -= 0.1;
      }
      else if (pid2  == 2 && mech2 == 3) {
        Constants.TootsieSlide.S0C_KI -= 0.1;
      }

      else if (pid2 == 3 && mech2 == 3) {
        Constants.TootsieSlide.S0C_KD -=0.1;
      }

      else if (pid2  == 4 && mech2 == 3) {
        Constants.TootsieSlide.S0C_KS -= 0.1;
      }

      else if (pid2 == 5 && mech2 == 3) {
        Constants.TootsieSlide.S0C_KG -= 0.1;
      }
  }


  public void IndexStuffCap(int[] broom, int[] mechbroom){
      int pid = broom[index];
      int mech = mechbroom[mechIndex];

    PIDDecreaseArm(pid, mech);
    PIDDecreaseElevator(pid,mech);
    PIDDecreaseTootsie(pid, mech);
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
