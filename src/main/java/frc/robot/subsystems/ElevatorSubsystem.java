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
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.LinearFilter;
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

  private LinearFilter elevatorFilter;
  private double currentHeightToF;
  private boolean elevatorZeroed;

  private MotionMagicConfigs mmc;
  private ElevatorPositions currentLevel;
  private CANrange distance; // Time of Flight (ToF) sensor

  private final MotionMagicVoltage controlRequest = new MotionMagicVoltage(0);
  private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private ElevatorSubsystem() {
    // Initialize motors
    elevatorZeroed = false;
    elevatorFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

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
    m2Config.apply(s0c);
    m1Config.apply(s1c);
    m2Config.apply(s1c);

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
    currentHeightToF = elevatorFilter.calculate(getToFDistance());
    resetPositionFiltered();
  }

  // instance for elevator subsystem
  public static ElevatorSubsystem getInstance() {
    if (instance == null) {
      instance = new ElevatorSubsystem();
    }
    return instance;
  }

  public boolean tofIsConnected() {
    return distance.isConnected();
  }

  public void resetPositionFiltered() {
    master.setPosition(
        currentHeightToF * Constants.ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS);
    DogLog.log(
        "subsystems/Elevator/resetElevatorPosition",
        currentHeightToF * Constants.ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS);
  }

  public void resetPosition() {
    if (tofIsConnected()) {
      master.setPosition(
          this.getToFDistance()
              * Constants.ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS);
      DogLog.log(
          "subsystems/Elevator/resetElevatorPosition",
          this.getToFDistance()
              * Constants.ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS);
    }
  }

  public void resetPosition(double posInHeight) {
    // TODO: add constant to convert distance to encoder values
    master.setPosition(
        posInHeight * Constants.ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS);
    DogLog.log(
        "subsystems/Elevator/resetElevatorPosition",
        posInHeight * Constants.ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS);
  }

  // Hardstop Zeroing functions:
  public void moveElevatorNegative() {
    master.setControl(velocityRequest.withVelocity(-5).withSlot(1));
  }

  public void reduceCurrentLimits() {
    master.updateCurrentLimits(30, 10);
  }

  public void resetCurrentLimits() {
    master.updateCurrentLimits(
        Constants.ElevatorConstants.STATOR_CURRENT_LIMIT,
        Constants.ElevatorConstants.SUPPLY_CURRENT_LIMIT);
  }

  public void resetElevatorPositionToZero() {
    master.setPosition(0);
    // master.setControl(controlRequest.withPosition(master.getPosition().getValueAsDouble()).withSlot(0));
    // master.setPosition(0);
    // master.setControl(controlRequest.withPosition(0).withSlot(0));
    // master.setPosition(0);
  }

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

  public double getError() {
    return currentLevel.height
            * ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS
            / Constants.ElevatorConstants.CARRAIGE_UPDUCTION
        - master.getPosition().getValueAsDouble();
  }

  public ElevatorPositions getLevel() {
    return currentLevel;
  }

  public boolean atIntake() {
    return currentLevel.equals(ElevatorPositions.Intake);
  }

  public void elevateTo(ElevatorPositions level) {
    this.currentLevel = level;
    this.setPosition(level.height);
  }

  public void setPosition(double height) {
    master.setControl(
        controlRequest
            .withPosition(
                height
                    * ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS
                    / ElevatorConstants.CARRAIGE_UPDUCTION)
            .withSlot(0));
    DogLog.log(
        "subsystems/Elevator/elevatorSetpoint(rot)",
        height
            * ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS
            / ElevatorConstants.CARRAIGE_UPDUCTION);
  }

  public void ElevatorTorqueMode() {
    DogLog.log("subsystems/Elevator/usingTorqueMode", true);
    master.setControl(torqueRequest.withOutput(Constants.ElevatorConstants.ELEVATOR_TORQUE));
    // .withMaxAbsDutyCycle(Constants.ElevatorConstants.ELEVATOR_DUTY_CYCLE));
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
    // 0.11 is the sensor offset
    DogLog.log(
        "subsystems/Elevator/ToF/DistanceNoOffset", distance.getDistance().getValueAsDouble());
    return distance.getDistance().getValueAsDouble() - Constants.ElevatorConstants.SENSOR_OFFSET;
  }

  public boolean isElevatorZeroed() {
    return elevatorZeroed;
  }

  public void elevatorHasBeenZeroed() {
    elevatorZeroed = true;
  }

  @Override
  public void periodic() {
    currentHeightToF = elevatorFilter.calculate(getToFDistance());
    // Time of Flight Sensor
    DogLog.log("subsystems/Elevator/getError", getError());
    DogLog.log("subsystems/Elevator/ToF/Distance", getToFDistance());
    DogLog.log("subsystems/Elevator/ToF/Connected", distance.isConnected());
    DogLog.log("subsystems/Elevator/ToF/LinearFilterDistance", currentHeightToF);

    DogLog.log("subsystems/Elevator/isAtPosition", this.isAtPosition());
    DogLog.log("subsystems/Elevator/targetPosition", currentLevel.getPosition());
    DogLog.log("subsystems/Elevator/targetHeightDist", currentLevel.getHeight());
    DogLog.log(
        "subsystems/Elevator/targetHeightRot",
        currentLevel.getHeight()
            * Constants.ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS
            / Constants.ElevatorConstants.CARRAIGE_UPDUCTION);
    DogLog.log(
        "subsystems/Elevator/currentHeightDist",
        master.getPosition().getValueAsDouble()
            * Constants.ElevatorConstants.CONVERSION_FACTOR_UP_ROTATIONS_TO_DISTANCE
            * Constants.ElevatorConstants.CARRAIGE_UPDUCTION);
    DogLog.log("subsystems/Elevator/currentHeightRot", master.getPosition().getValueAsDouble());
    DogLog.log(
        "subsystems/Elevator/command",
        this.getCurrentCommand() == null ? "NOTHING" : this.getCurrentCommand().getName());
    DogLog.log(
        "subsystems/Elevator/resetPositionBoolean",
        this.isAtPosition() && this.getLevel().equals(ElevatorPositions.Intake));
    DogLog.log(
        "subsystems/Elevator/targetisIntake", this.getLevel().equals(ElevatorPositions.Intake));
    DogLog.log("subsystems/Elevator/targetLevel", this.getLevel().toString());
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
