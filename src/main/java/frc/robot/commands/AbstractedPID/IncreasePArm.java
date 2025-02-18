package frc.robot.commands.AbstractedPID;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;
import java.util.function.BooleanSupplier;

public class IncreasePArm extends Command {

  private boolean prevInc = false;

  private boolean pidChanged = false;

  private boolean mechChanged = false;

  private static int[] broom = {1, 2, 3, 4, 5};
  private boolean prevPID = false;
  private boolean prevMech = false;
  private boolean prevDec = false;
  private boolean prev4 = false;
  public static int index = 0;
  private static int[] mechbroom = {1, 2, 3};
  private static int mechIndex = 0;

  BooleanSupplier increaseFunction;
  BooleanSupplier decreaseFunction;
  BooleanSupplier pidChange;
  BooleanSupplier mechChange;

  private CommandXboxController joystick;

  public IncreasePArm(
      BooleanSupplier increaseFunction,
      BooleanSupplier decreaseFunction,
      BooleanSupplier pidChange,
      BooleanSupplier mechChange) {
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

    if (prevPID != pidChange.getAsBoolean() && pidChange.getAsBoolean() == true) {
      IndexPID();
    }
    prevPID = pidChange.getAsBoolean();

    if (prevMech != mechChange.getAsBoolean() && mechChange.getAsBoolean() == true) {
      IndexMech();
    }
    prevMech = mechChange.getAsBoolean();

    if (prevInc != increaseFunction.getAsBoolean() && increaseFunction.getAsBoolean() == true) {
      IndexStuffNoCap(broom, mechbroom);
    }
    prevInc = increaseFunction.getAsBoolean();

    if (prevDec != decreaseFunction.getAsBoolean() && decreaseFunction.getAsBoolean() == true) {
      IndexStuffCap(broom, mechbroom);
    }
    prevDec = increaseFunction.getAsBoolean();

    resetPIDController();
  }

  private void resetPIDController() {
    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(ElevatorConstants.S0C_KP)
            .withKI(ElevatorConstants.S0C_KI)
            .withKD(ElevatorConstants.S0C_KD)
            .withKS(0);
    TalonFXConfigurator masterConfig =
        ElevatorSubsystem.getInstance().getMasterMotor().getConfigurator();
    masterConfig.apply(s0c);

    Slot0Configs s0cArm =
        new Slot0Configs().withKP(Constants.Arm.S0C_KP * 0.75).withKI(0).withKD(0);
    TalonFXConfigurator armConfig = ArmSubsystem.getInstance().armMotor.getConfigurator();
    armConfig.apply(s0cArm);

    Slot0Configs s0cFlywheel =
        new Slot0Configs().withKP(Constants.Flywheel.FLYWHEEL_S0C_KP).withKI(0).withKD(0);
    TalonFXConfigurator flywhConfigurator =
        ArmSubsystem.getInstance().flywheelMotor.getConfigurator();
    flywhConfigurator.apply(s0cFlywheel);

    Slot0Configs s0cSlide =
        new Slot0Configs()
            .withKP(Constants.TootsieSlide.S0C_KP)
            .withKI(Constants.TootsieSlide.S0C_KI)
            .withKD(Constants.TootsieSlide.S0C_KD);
    TalonFXConfigurator slideConfig = TootsieSlideSubsystem.getInstance().master.getConfigurator();
    slideConfig.apply(s0cSlide);
  }

  private void IndexPID() {
    index++;
    if (index > broom.length - 1) {
      index = 0;
    }
  }

  private void IndexMech() {
    mechIndex++;
    if (mechIndex > mechbroom.length - 1) {
      mechIndex = 0;
    }
  }

  public void IndexStuffNoCap(int[] broom, int[] mechbroom) {
    int pid = broom[index];
    int mech = mechbroom[mechIndex];

    PIDIcreaseArm(pid, mech);
    PIDIncreaseElevator(pid, mech);
    PIDIncreaseTootsie(pid, mech);
  }

  public int getPID() {
    return broom[index];
  }

  public int getMech() {
    return mechbroom[mechIndex];
  }

  public void PIDIcreaseArm(int pid2, int mech2) {
    if (pid2 == 1 && mech2 == 1) {
      Constants.Arm.S0C_KP += 0.1;
    } else if (pid2 == 2 && mech2 == 1) {
      Constants.Arm.S0C_KI += 0.1;
    } else if (pid2 == 3 && mech2 == 1) {
      Constants.Arm.S0C_KD += 0.1;
    } else if (pid2 == 4 && mech2 == 1) {
      Constants.Arm.S0C_KS += 0.1;
    } else if (pid2 == 5 && mech2 == 1) {
      Constants.Arm.S0C_KG += 0.1;
    }
  }

  public void PIDDecreaseArm(int pid2, int mech2) {
    if (pid2 == 1 && mech2 == 1) {
      Constants.Arm.S0C_KP -= 0.1;
    } else if (pid2 == 2 && mech2 == 1) {
      Constants.Arm.S0C_KI -= 0.1;
    } else if (pid2 == 3 && mech2 == 1) {
      Constants.Arm.S0C_KD -= 0.1;
    } else if (pid2 == 4 && mech2 == 1) {
      Constants.Arm.S0C_KS -= 0.1;
    } else if (pid2 == 5 && mech2 == 1) {
      Constants.Arm.S0C_KG -= 0.1;
    }
  }

  public void PIDIncreaseElevator(int pid2, int mech2) {
    if (pid2 == 1 && mech2 == 2) {
      Constants.ElevatorConstants.S0C_KP += 0.1;
    } else if (pid2 == 2 && mech2 == 2) {
      Constants.ElevatorConstants.S0C_KI += 0.1;
    } else if (pid2 == 3 && mech2 == 2) {
      Constants.ElevatorConstants.S0C_KD += 0.1;
    } else if (pid2 == 4 && mech2 == 2) {
      Constants.ElevatorConstants.S0C_KS += 0.1;
    } else if (pid2 == 5 && mech2 == 2) {
      Constants.ElevatorConstants.S0C_KG += 0.1;
    }
  }

  public void PIDDecreaseElevator(int pid2, int mech2) {
    if (pid2 == 1 && mech2 == 2) {
      Constants.ElevatorConstants.S0C_KP -= 0.1;
    } else if (pid2 == 2 && mech2 == 2) {
      Constants.ElevatorConstants.S0C_KI -= 0.1;
    } else if (pid2 == 3 && mech2 == 2) {
      Constants.ElevatorConstants.S0C_KD -= 0.1;
    } else if (pid2 == 4 && mech2 == 2) {
      Constants.ElevatorConstants.S0C_KS -= 0.1;
    } else if (pid2 == 5 && mech2 == 2) {
      Constants.ElevatorConstants.S0C_KG -= 0.1;
    }
  }

  public void PIDIncreaseTootsie(int pid2, int mech2) {

    if (pid2 == 1 && mech2 == 3) {
      Constants.TootsieSlide.S0C_KP += 0.1;
    } else if (pid2 == 2 && mech2 == 3) {
      Constants.TootsieSlide.S0C_KI += 0.1;
    } else if (pid2 == 3 && mech2 == 3) {
      Constants.TootsieSlide.S0C_KD += 0.1;
    } else if (pid2 == 4 && mech2 == 3) {
      Constants.TootsieSlide.S0C_KS += 0.1;
    } else if (pid2 == 5 && mech2 == 3) {
      Constants.TootsieSlide.S0C_KG += 0.1;
    }
  }

  public void PIDDecreaseTootsie(int pid2, int mech2) {

    if (pid2 == 1 && mech2 == 3) {
      Constants.TootsieSlide.S0C_KP -= 0.1;
    } else if (pid2 == 2 && mech2 == 3) {
      Constants.TootsieSlide.S0C_KI -= 0.1;
    } else if (pid2 == 3 && mech2 == 3) {
      Constants.TootsieSlide.S0C_KD -= 0.1;
    } else if (pid2 == 4 && mech2 == 3) {
      Constants.TootsieSlide.S0C_KS -= 0.1;
    } else if (pid2 == 5 && mech2 == 3) {
      Constants.TootsieSlide.S0C_KG -= 0.1;
    }
  }

  public void PIDIncreaseArmWheel(int pid2, int mech2) {

    if (pid2 == 1 && mech2 == 4) {
      Constants.Flywheel.FLYWHEEL_S0C_KP += 0.1;
    } else if (pid2 == 2 && mech2 == 4) {
      Constants.Flywheel.FLYWHEEL_S0C_KI += 0.1;
    } else if (pid2 == 3 && mech2 == 4) {
      Constants.Flywheel.FLYWHEEL_S0C_KD += 0.1;
    } else if (pid2 == 4 && mech2 == 4) {
      Constants.Flywheel.FLYWHEEL_S0C_KS += 0.1;
    } else if (pid2 == 5 && mech2 == 4) {
      Constants.Flywheel.FLYWHEEL_S0C_KG += 0.1;
    }
  }

  public void PIDDecreaseArmWheel(int pid2, int mech2) {

    if (pid2 == 1 && mech2 == 4) {
      Constants.Flywheel.FLYWHEEL_S0C_KP -= 0.1;
    } else if (pid2 == 2 && mech2 == 4) {
      Constants.Flywheel.FLYWHEEL_S0C_KI -= 0.1;
    } else if (pid2 == 3 && mech2 == 4) {
      Constants.Flywheel.FLYWHEEL_S0C_KD -= 0.1;
    } else if (pid2 == 4 && mech2 == 4) {
      Constants.Flywheel.FLYWHEEL_S0C_KS -= 0.1;
    } else if (pid2 == 5 && mech2 == 4) {
      Constants.Flywheel.FLYWHEEL_S0C_KG -= 0.1;
    }
  }

  public void IndexStuffCap(int[] broom, int[] mechbroom) {
    int pid = broom[index];
    int mech = mechbroom[mechIndex];

    PIDDecreaseArm(pid, mech);
    PIDDecreaseElevator(pid, mech);
    PIDDecreaseTootsie(pid, mech);
  }

  public static String broomIndex() {
    if (broom[index] == 1) {
      return "P-value";
    } else if (broom[index] == 2) {
      return "I-value";
    } else if (broom[index] == 3) {
      return "D-value";
    } else if (broom[index] == 4) {
      return "KS-value";
    } else if (broom[index] == 5) {
      return "KG-value";
    }
    return "something no work bruv";
  }

  public static String mechIndex() {
    if (mechbroom[mechIndex] == 1) {
      return "Arm";
    } else if (mechbroom[mechIndex] == 2) {
      return "Elevator";
    } else if (mechbroom[mechIndex] == 3) {
      return "Tootsie Slide";
    }
    return "something no work bruv";
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
