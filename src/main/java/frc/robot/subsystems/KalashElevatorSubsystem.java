package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;

public class KalashElevatorSubsystem extends SubsystemBase {
  private TalonFX motor1;
  private TalonFX motor2;
  private TalonFX master;
  private MotionMagicConfigs mmc;
  ElevatorPositions currentLevel;
  private MotionMagicVoltage controlRequest = new MotionMagicVoltage(0);

  public KalashElevatorSubsystem() {
    motor1 = new TalonFX(ElevatorConstants.MOTOR1_PORT);
    motor2 = new TalonFX(ElevatorConstants.MOTOR2_PORT);

    Follower follower = new Follower(Constants.ElevatorConstants.MOTOR1_PORT, false);
    motor2.setControl(follower);

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

    mmc = new MotionMagicConfigs();
    mmc.MotionMagicCruiseVelocity = ElevatorConstants.MOTIONMAGIC_MAX_VELOCITY;
    mmc.MotionMagicAcceleration = (ElevatorConstants.MOTIONMAGIC_MAX_ACCELERATION);

    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    m2Config.apply(mmc);
    m2Config.apply(moc);
    m1Config.apply(mmc);
    m1Config.apply(moc);

    master = motor1;

    currentLevel = ElevatorPositions.Intake;
  }

  public void setHeight(ElevatorPositions level) {
    master.setControl(controlRequest.withPosition(level.height));
  }

  public void zeroEncoders() {
    master.setPosition(0);
  }

  public double getError() {
    return currentLevel.height
            * ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS
            / Constants.ElevatorConstants.CARRAIGE_UPDUCTION
        - master.getPosition().getValueAsDouble();
  }

  public boolean targetReached() {
    return (Math.abs(getError())) <= ElevatorConstants.SETPOINT_TOLERANCE;
  }

  public void periodic() {
    DogLog.log("subsystems/Elevator/isAtPosition", targetReached());
  }
}
