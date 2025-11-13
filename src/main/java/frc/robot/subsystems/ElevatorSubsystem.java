package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.util.LoggedTalonFX;

public class ElevatorSubsystem extends SubsystemBase {

  private static ElevatorSubsystem instance;
  private TalonFX motor1;
  private TalonFX motor2;
  public TalonFX master;
  private MotionMagicVoltage request = new MotionMagicVoltage(0);
  private VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0);
  private LinearFilter elevatorFilter;
  private double currentHeightToF;
  private boolean elevatorZeroed;

  private MotionMagicConfigs mmc;
  private ElevatorPositions currentLevel;
  private CANrange distance; // Time of Flight (xToF) sensor

  public ElevatorSubsystem() {
    // elevatorZeroed = false;

    motor1 =
        new LoggedTalonFX(
            "subsystems/Elevator/motor1",
            ElevatorConstants.MOTOR1_PORT,
            Constants.Swerve.SwerveType.JAMES_HARDEN.CANBUS_NAME);

    motor2 =
        new LoggedTalonFX(
            "subsystems/Elevator/motor2",
            Constants.ElevatorConstants.MOTOR2_PORT,
            Constants.Swerve.SwerveType.JAMES_HARDEN.CANBUS_NAME);

    request = new MotionMagicVoltage(0);

    Follower follower = new Follower(ElevatorConstants.MOTOR1_PORT, false);
    motor2.setControl(follower);

    // elevator zeroing
    Slot1Configs s1c =
        new Slot1Configs()
            .withKP(ElevatorConstants.S1C_KP)
            .withKI(ElevatorConstants.S1C_KI)
            .withKD(ElevatorConstants.S1C_KD)
            .withKS(ElevatorConstants.S0C_KS)
            .withKG(ElevatorConstants.S0C_KG)
            .withKA(ElevatorConstants.S0C_KA)
            .withKV(ElevatorConstants.S0C_KV)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    // normal stuffs
    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(ElevatorConstants.S0C_KP)
            .withKI(ElevatorConstants.S0C_KI)
            .withKD(ElevatorConstants.S0C_KD)
            .withKS(ElevatorConstants.S0C_KS)
            .withKG(ElevatorConstants.S0C_KG)
            .withKA(ElevatorConstants.S0C_KA)
            .withKV(ElevatorConstants.S0C_KV)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(ElevatorConstants.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(ElevatorConstants.SUPPLY_CURRENT_LIMIT);

    TalonFXConfigurator m1Config = motor1.getConfigurator();
    TalonFXConfigurator m2Config = motor2.getConfigurator();

    m1Config.apply(clc);
    m2Config.apply(clc);
    m1Config.apply(s0c);
    m2Config.apply(s0c);
    m1Config.apply(s1c);
    m2Config.apply(s1c);

    MotionMagicConfigs mmc = new MotionMagicConfigs();
    mmc.MotionMagicCruiseVelocity = ElevatorConstants.MOTIONMAGIC_MAX_VELOCITY;
    mmc.MotionMagicAcceleration = ElevatorConstants.MOTIONMAGIC_MAX_ACCELERATION;

    m1Config.apply(mmc);
    m2Config.apply(mmc);

    // m1Config.apply(moc);
    // m2Config.apply(moc);

    master = motor1;

    // public boolean
    //  public boolean
    resetPositionFiltered();
  }

  public void setHeight() {
    master.setControl(
        request.withPosition(
            1.5
                * ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS
                / ElevatorConstants.CARRAIGE_UPDUCTION));
  }

  public static ElevatorSubsystem getInstance() {
    // TODO Auto-generated method stub
    if (instance == null) {
      instance = new ElevatorSubsystem();
    }
    return instance;
  }

  public boolean isAtPosition() {
    // TODO Auto-generated method stub
    return true;
  }

  public void elevateTo(ElevatorPositions intake) {
    // TODO Auto-generated method stub
    master.setControl(
        request.withPosition(
            intake.getHeight()
                * ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS
                / ElevatorConstants.CARRAIGE_UPDUCTION));
  }

  public boolean atIntake() {
    // TODO Auto-generated method stub
    return true;
  }

  public void resetPositionFiltered() {
    master.setPosition(
        currentHeightToF * Constants.ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS);
    
  }

  public void reduceCurrentLimits() {
    ((LoggedTalonFX) master).updateCurrentLimits(30, 10);
    
  } 

  public boolean checkCurrent() {
    // TODO Auto-generated method stub
    return true;
  }

  public void elevatorHasBeenZeroed() {
    // TODO Auto-generated method stub
    elevatorZeroed = true;
  }

  public void resetElevatorPositionToZero() {
    // TODO Auto-generated method stub
    master.setPosition(0);
  }

  public void moveElevatorNegative() {
    // TODO Auto-generated method stub
    master.setControl(velocityRequest.withVelocity(-5).withSlot(1));
  }

  public void ElevatorTorqueMode() {
    // TODO Auto-generated method stub
    master.setControl(torqueRequest.withOutput(Constants.ElevatorConstants.ELEVATOR_TORQUE));
  }

  public boolean isElevatorZeroed() {
    // TODO Auto-generated method stub
    return true;
  }
}
