// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.util.LoggedTalonFX;

public class ElevatorSubsystem extends SubsystemBase {
  private static ElevatorSubsystem instance;

  private LoggedTalonFX motor1;
  private LoggedTalonFX motor2;
  public LoggedTalonFX master;
  private CANrange distance;
  private LinearFilter elevatorFilter;

  private final MotionMagicVoltage controlRequest = new MotionMagicVoltage(null);
  private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(null);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(null);

  private ElevatorPositions currLevel;
  private double tolerance = 3.0;
  private boolean elevatorZeroed;
  private double currentHeightToF;

  public ElevatorSubsystem() {
    motor1 = new LoggedTalonFX(ElevatorConstants.MOTOR1_PORT);
    motor2 = new LoggedTalonFX(ElevatorConstants.MOTOR2_PORT);
    distance =
        new CANrange(
            ElevatorConstants.CANRANGE_PORT, Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);
    elevatorZeroed = false;
    elevatorFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    master = motor1;
    Follower follower = new Follower(ElevatorConstants.MOTOR1_PORT, false);
    motor2.setControl(follower);

    TalonFXConfigurator m1Config = motor1.getConfigurator();

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(ElevatorConstants.S0C_KP)
            .withKI(ElevatorConstants.S0C_KI)
            .withKD(ElevatorConstants.S0C_KD);

    Slot1Configs s1c =
        new Slot1Configs()
            .withKP(ElevatorConstants.S1C_KP)
            .withKI(ElevatorConstants.S1C_KI)
            .withKD(ElevatorConstants.S1C_KD)
            .withKS(ElevatorConstants.S0C_KS)
            .withKG(ElevatorConstants.S0C_KG)
            .withKA(ElevatorConstants.S0C_KA)
            .withKV(ElevatorConstants.S0C_KV);

    motor1.updateCurrentLimits(
        ElevatorConstants.STATOR_CURRENT_LIMIT, ElevatorConstants.SUPPLY_CURRENT_LIMIT);

    MotionMagicConfigs mmc =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(ElevatorConstants.ACCELERATION)
            .withMotionMagicCruiseVelocity(ElevatorConstants.CRUISE_VELOCITY);

    m1Config.apply(s0c);
    m1Config.apply(mmc);

    currentHeightToF = elevatorFilter.calculate(getToFDistance());
  }

  public static ElevatorSubsystem getInstance() {
    if (instance == null) {
      instance = new ElevatorSubsystem();
    }
    return instance;
  }

  public void setHeight(ElevatorPositions level) {
    currLevel = level;
    master.setControl(controlRequest.withPosition(level.height));
  }

  public ElevatorPositions getLevel() {
    return currLevel;
  }

  public double getErrorDist() {
    return ((currLevel.height * ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS)
            / ElevatorConstants.CARRAIGE_UPDUCTION)
        - master.getPosition().getValueAsDouble();
  }

  public boolean isAtPosition() {
    return (Math.abs(getErrorDist())) <= tolerance;
  }

  public boolean atIntake() {
    return currLevel.equals(ElevatorPositions.Intake);
  }

  public void zeroElevator() {
    master.setPosition(0);
  }

  public void elevatorHasBeenZeroed() {
    elevatorZeroed = true;
  }

  public boolean isElevatorZeroed() {
    return elevatorZeroed;
  }

  public void ElevatorTorqueMode() {
    master.setControl(torqueRequest.withOutput(ElevatorConstants.ELEVATOR_TORQUE));
  }

  public void reduceCurrentLimits() {
    master.updateCurrentLimits(30, 10);
  }

  public void resetCurrentLimits() {
    master.updateCurrentLimits(
        ElevatorConstants.STATOR_CURRENT_LIMIT, ElevatorConstants.SUPPLY_CURRENT_LIMIT);
  }

  public void moveElevatorNegative() {
    master.setControl(velocityRequest.withVelocity(-5).withSlot(1));
  }

  public boolean checkCurrent() {
    double Supplycurrent = Math.abs(master.getSupplyCurrent().getValue().magnitude());
    double Statorcurrent = Math.abs(master.getStatorCurrent().getValue().magnitude());
    if (Supplycurrent > 1.0 && Statorcurrent > 20) {
      return true;
    }
    return false;
  }

  public void resetPositionFiltered() {
    master.setPosition(
        currentHeightToF * ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS);
  }

  public double getToFDistance() {
    // 0.11 is the sensor offset
    return distance.getDistance().getValueAsDouble() - Constants.ElevatorConstants.SENSOR_OFFSET;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    DogLog.log("Current angle", getLevel());
    DogLog.log("Is at target", isAtPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}