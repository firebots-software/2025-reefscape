// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.util.LoggedTalonFX;

public class ElevatorSubsystemMD2 extends SubsystemBase {
  private static ElevatorSubsystemMD2 instance;

  // config: wip, will update based on last years elevator subsystem before testing

  LoggedTalonFX motor1;
  LoggedTalonFX motor2;
  LoggedTalonFX master;

  MotionMagicVoltage request = new MotionMagicVoltage(0);

  private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private double currentHeightToF;
  private double targetHeight;

  public ElevatorSubsystemMD2() {
    motor1 =
        new LoggedTalonFX(
            "subsystems/Elevator/motor1",
            ElevatorConstants.MOTOR1_PORT,
            Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);
    motor2 =
        new LoggedTalonFX(
            "subsystems/Elevator/motor2",
            ElevatorConstants.MOTOR2_PORT,
            Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);

    Follower follower = new Follower(ElevatorConstants.MOTOR1_PORT, false);
    motor2.setControl(follower);

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

    motor1.updateCurrentLimits(
        ElevatorConstants.STATOR_CURRENT_LIMIT, ElevatorConstants.SUPPLY_CURRENT_LIMIT);
    motor2.updateCurrentLimits(
        ElevatorConstants.STATOR_CURRENT_LIMIT, ElevatorConstants.SUPPLY_CURRENT_LIMIT);

    TalonFXConfigurator m1Config = motor1.getConfigurator();
    TalonFXConfigurator m2Config = motor2.getConfigurator();

    m1Config.apply(s0c);
    m1Config.apply(s1c);
    m2Config.apply(s0c);
    m2Config.apply(s1c);

    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    MotionMagicConfigs mmc =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(ElevatorConstants.MOTIONMAGIC_MAX_ACCELERATION)
            .withMotionMagicCruiseVelocity(ElevatorConstants.MOTIONMAGIC_MAX_VELOCITY);

    m1Config.apply(mmc);
    m2Config.apply(mmc);
    m1Config.apply(moc);
    m2Config.apply(moc);

    master = motor1;
  }

  public static ElevatorSubsystemMD2 getInstance() {
    if (instance == null) {
      instance = new ElevatorSubsystemMD2();
    }
    return instance;
  }

  // use a control request to move to the height.
  public void setHeight(double height) {

    targetHeight = height*ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS;
    master.setControl(request.withPosition(targetHeight));
    
  }

  // getters
  public double getCurrentHeight() {
    return master.getPosition().getValueAsDouble();
  }

  public double getTargetHeight() {
    return targetHeight;
  }

  // distance from the target
  public double getErrorDist() {
    return Math.abs(targetHeight - getCurrentHeight());
  }

  public boolean isAtTargetHeight() {
    return getErrorDist() <= ElevatorConstants.SETPOINT_TOLERANCE;
  }

  // based on last year's code
  public boolean atIntake() {
    return getCurrentHeight() == (ElevatorPositions.Intake.height);
  }

  public void zeroElevator() {
    master.setPosition(0);
  }

  public boolean isElevatorZeroed() {
    return master.getPosition().getValueAsDouble() == 0;
  }

  public void reduceCurrentLimits() {
    master.updateCurrentLimits(30, 10);
  }

  // all of these created based on last year's code for one of the commands that requires it, my
  // interpretation of their use is listed

  // move elevator down, i assume this is for re-zeroing it.
  public void moveElevatorNegative() {
    master.setControl(velocityRequest.withVelocity(-5).withSlot(1));
  }

  // set torque mode from constants.
  public void ElevatorTorqueMode() {
    DogLog.log("subsystems/Elevator/usingTorqueMode", true);
    master.setControl(torqueRequest.withOutput(Constants.ElevatorConstants.ELEVATOR_TORQUE));
    // .withMaxAbsDutyCycle(Constants.ElevatorConstants.ELEVATOR_DUTY_CYCLE));
  }

  // apply the current limits defined in constants
  public void resetCurrentLimits() {
    master.updateCurrentLimits(
        Constants.ElevatorConstants.STATOR_CURRENT_LIMIT,
        Constants.ElevatorConstants.SUPPLY_CURRENT_LIMIT);
  }

  // log the supply and stator current, return bool depending if they are above certain thresholds.
  public boolean checkCurrent() {
    double Supplycurrent = Math.abs(master.getSupplyCurrent().getValue().magnitude());
    double Statorcurrent = Math.abs(master.getStatorCurrent().getValue().magnitude());
    DogLog.log("subsystems/Elevator/ZeroElevatorHardStop/supply", Supplycurrent);
    DogLog.log("subsystems/Elevator/ZeroElevatorHardStop/stator", Statorcurrent);

    if (Supplycurrent > 1.0 && Statorcurrent > 20) {
      return true;
    }
    return false;
  }

  // set the position based on tof sensor
  public void resetPositionFiltered() {
    master.setPosition(
        currentHeightToF * Constants.ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS);
    DogLog.log(
        "subsystems/Elevator/resetElevatorPosition",
        currentHeightToF * Constants.ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    DogLog.log("Current angle", getCurrentHeight());
    DogLog.log("Is at target", isAtTargetHeight());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
