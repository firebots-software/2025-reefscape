// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.util.LoggedTalonFX;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private static ElevatorSubsystem instance;

  private LoggedTalonFX motor1;
  private LoggedTalonFX motor2;
  public LoggedTalonFX master;

  private MotionMagicConfigs mmc;
  private ElevatorPositions currentLevel;
  private CANrange distance; // Time of Flight (ToF) sensor

  private ElevatorSubsystem() {
    // Initialize motors

    distance =
        new CANrange(
            ElevatorConstants.CANRANGE_PORT, Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);

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

    currentLevel = ElevatorPositions.Intake;

    // Set up motor followers and deal with inverted motors
    Follower follower = new Follower(ElevatorConstants.MOTOR1_PORT, false);
    motor2.setControl(follower);

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(ElevatorConstants.S0C_KP)
            .withKI(ElevatorConstants.S0C_KI)
            .withKD(ElevatorConstants.S0C_KD)
            .withKS(ElevatorConstants.S0C_KS)
            .withKG(ElevatorConstants.S0C_KG)
            .withKA(ElevatorConstants.S0C_KA)
            .withKV(ElevatorConstants.S0C_KV);

    motor1.updateCurrentLimits(
        ElevatorConstants.STATOR_CURRENT_LIMIT, ElevatorConstants.SUPPLY_CURRENT_LIMIT);
    motor2.updateCurrentLimits(
        ElevatorConstants.STATOR_CURRENT_LIMIT, ElevatorConstants.SUPPLY_CURRENT_LIMIT);

    TalonFXConfigurator m1Config = motor1.getConfigurator();
    TalonFXConfigurator m2Config = motor2.getConfigurator();

    m1Config.apply(s0c);
    m2Config.apply(s0c);
    
    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    // Apply MotionMagic to motors
    mmc = new MotionMagicConfigs();
    mmc.MotionMagicCruiseVelocity = ElevatorConstants.MOTIONMAGIC_MAX_VELOCITY;
    mmc.MotionMagicAcceleration = ElevatorConstants.MOTIONMAGIC_MAX_ACCELERATION;

    m1Config.apply(mmc);
    m2Config.apply(mmc);

    m1Config.apply(moc);
    m2Config.apply(moc);
    
    master = motor1;
    resetPosition();
  }

  // instance for elevator subsystem
  public static ElevatorSubsystem getInstance() {
    if (instance == null) {
      instance = new ElevatorSubsystem();
    }
    return instance;
  }

  public void resetPosition() {
    // TODO: add constant to convert distance to encoder values
    if (distance.isConnected()) {
      master.setPosition(
          this.getToFDistance());
      DogLog.log(
          "subsystems/Elevator/resetElevatorPosition",
          this.getToFDistance());
    }
  }

  public double getError() {
    return currentLevel.height * ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS
        - master.getPosition().getValueAsDouble();
  }

  public ElevatorPositions getLevel() {
    return currentLevel;
  }

  public void elevateTo(ElevatorPositions level) {
    this.currentLevel = level;
    this.setPosition(level.height);
  }

  private void setPosition(double height) {
    master.setControl(
        new MotionMagicVoltage(
            height * ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS));
    DogLog.log(
        "subsystems/Elevator/elevatorSetpoint(rot)",
        height * ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS);
  }

  // TODO: ONLY FOR DEBUGGING
  public void testElevator(double height) {
    this.setPosition(height);
  }

  public boolean isAtPosition() {
    return (Math.abs(getError()) <= ElevatorConstants.SETPOINT_TOLERANCE);
  }

  public boolean canFunnelTransferCoralToScoring() {
    return this.getLevel().equals(Constants.ElevatorConstants.ElevatorPositions.Intake)
        && this.getError() < Constants.ElevatorConstants.MAX_POSITIONAL_ERROR;
  }

  public double getToFDistance() {
    //0.11 is the sensor offset
    return distance.getDistance().getValueAsDouble()- Constants.ElevatorConstants.sensorOffset;
  }

  @Override
  public void periodic() {
    // Time of Flight Sensor
    DogLog.log("subsystems/Elevator/ToF/Distance", getToFDistance());
    DogLog.log("subsystems/Elevator/ToF/Connected", distance.isConnected());

    DogLog.log("subsystems/Elevator/isAtPosition", this.isAtPosition());
    DogLog.log("subsystems/Elevator/targetPosition", currentLevel.getPosition());
    DogLog.log("subsystems/Elevator/targetHeight", currentLevel.getHeight());
    DogLog.log("subsystems/Elevator/currentHeightDist", master.getPosition().getValueAsDouble() * Constants.ElevatorConstants.CONVERSION_FACTOR_UP_ROTATIONS_TO_DISTANCE/Constants.ElevatorConstants.CARRAIGE_UPDUCTION);
    DogLog.log("subsystems/Elevator/currentHeightRot", master.getPosition().getValueAsDouble());

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // Simulate encoder behavior based on motor speed
    double simulatedSpeed = master.getVelocity().getValueAsDouble();
    double currentPosition = master.getPosition().getValueAsDouble();

    // Update simulated position based on speed (simplified example)
    double newPosition = currentPosition + simulatedSpeed * 0.02; // Assuming a 20ms loop
    master.setPosition(
        newPosition); // Alarming to have this since running this on the robot will lead to

    // Log simulation data for debugging
    DogLog.log("Simulated Position", newPosition);
    DogLog.log("Simulated Speed", simulatedSpeed);
  }
}
