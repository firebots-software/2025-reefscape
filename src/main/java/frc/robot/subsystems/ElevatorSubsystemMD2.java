// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.util.LoggedTalonFX;

public class ElevatorSubsystemMD2 extends SubsystemBase {
  private static ElevatorSubsystemMD2 instance;
  private boolean elevatorZeroed;

  LoggedTalonFX motor1;
  LoggedTalonFX motor2;
  LoggedTalonFX master;

  MotionMagicVoltage request = new MotionMagicVoltage(null);
  TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0);
  VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private double targetHeight;
  private double currentHeightToF;
  private ElevatorPositions currentLevel;
  private double tolerance = 3.0;

  public ElevatorSubsystemMD2() {
    elevatorZeroed = false;
    motor1 = new LoggedTalonFX(1);
    motor2 = new LoggedTalonFX(2);
    currentLevel = ElevatorPositions.Intake;

    master = motor1;
    Follower follower = new Follower(1, false);
    motor2.setControl(follower);

    Slot0Configs s0c = new Slot0Configs().withKP(1.0).withKI(1.0).withKD(1.0);

    motor1.updateCurrentLimits(1.0, 1.0);
    motor2.updateCurrentLimits(1.0, 1.0);

    MotionMagicConfigs mmc =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(ElevatorConstants.ACCELERATION)
            .withMotionMagicCruiseVelocity(ElevatorConstants.CRUISE_VELOCITY);

    TalonFXConfigurator m1Config = motor1.getConfigurator();

    m1Config.apply(s0c);
    m1Config.apply(mmc);
  }

  public void setHeight(double height) {
    targetHeight = height;
    master.setControl(request.withPosition(targetHeight));
  }

  public double getCurrentHeight() {
    return master.getPosition().getValueAsDouble();
  }

  public double getTargetHeight() {
    return targetHeight;
  }

  public double getErrorDist() {
    return Math.abs(targetHeight - getCurrentHeight());
  }

  public boolean isAtTargetHeight() {
    return getErrorDist() <= tolerance;
  }

  public void zeroElevator() {
    master.setPosition(0);
  }

  public void ElevatorTorqueMode() {
    master.setControl(torqueRequest.withOutput(Constants.ElevatorConstants.ELEVATOR_TORQUE));
  }

  public void reduceCurrentLimits() {
    master.updateCurrentLimits(30, 10);
  }

  public void moveElevatorNegative() {
    master.setControl(velocityRequest.withVelocity(-5).withSlot(1));
  }

  public void resetCurrentLimits() {
    master.updateCurrentLimits(
        Constants.ElevatorConstants.STATOR_CURRENT_LIMIT,
        Constants.ElevatorConstants.SUPPLY_CURRENT_LIMIT);
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
        currentHeightToF * Constants.ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS);
  }

  public boolean atIntake() {
    return currentLevel.equals(ElevatorPositions.Intake);
  }

  public static ElevatorSubsystemMD2 getInstance() {
    if (instance == null) {
      instance = new ElevatorSubsystemMD2();
    }
    return instance;
  }

  public boolean isElevatorZeroed() {
    return elevatorZeroed;
  }

  public void elevatorHasBeenZeroed() {
    elevatorZeroed = true;
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
