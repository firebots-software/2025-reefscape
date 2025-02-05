// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private LoggedTalonFX master;
  private Double holdPosValue = 0.0;

  private MotionMagicConfigs mmc;
  private ElevatorPositions currentLevel;
  private CANrange distance;

  private ElevatorSubsystem() {
    // Initialize motors

    distance = new CANrange(0);

    motor1 = new LoggedTalonFX(ElevatorConstants.MOTOR1_PORT);
    motor2 = new LoggedTalonFX(ElevatorConstants.MOTOR2_PORT);

    currentLevel = ElevatorPositions.Intake;

    // Set up motor followers and deal with inverted motors
    Follower follower = new Follower(ElevatorConstants.MOTOR1_PORT, false);
    motor2.setControl(follower);

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(ElevatorConstants.S0C_KP)
            .withKI(ElevatorConstants.S0C_KI)
            .withKD(ElevatorConstants.S0C_KD)
            .withKS(0);

    motor1.updateCurrentLimits(
        ElevatorConstants.STATOR_CURRENT_LIMIT, ElevatorConstants.SUPPLY_CURRENT_LIMIT);
    motor2.updateCurrentLimits(
        ElevatorConstants.STATOR_CURRENT_LIMIT, ElevatorConstants.SUPPLY_CURRENT_LIMIT);

    TalonFXConfigurator m1Config = motor1.getConfigurator();
    TalonFXConfigurator m2Config = motor2.getConfigurator();

    m1Config.apply(s0c);
    m2Config.apply(s0c);

    // Apply MotionMagic to motor1(master)
    mmc = new MotionMagicConfigs();
    mmc.MotionMagicCruiseVelocity = ElevatorConstants.MOTIONMAGIC_KV;
    mmc.MotionMagicAcceleration = ElevatorConstants.MOTIONMAGIC_KA;

    m1Config.apply(mmc);
    m2Config.apply(mmc);
  }

  // instance for elevator subsystem
  public static ElevatorSubsystem getInstance() {
    if (instance == null) {
      instance = new ElevatorSubsystem();
    }
    return instance;
  }

  public void resetPosition() {
    //TODO: add constant to convert distance to encoder values
    if (distance.isConnected()) {
      master.setPosition(
          (getDistance()) * ElevatorConstants.CONVERSION_FACTOR);
    }
  }

  public double getError() {
    return currentLevel.height - motor1.getPosition().getValueAsDouble();
  }

  public ElevatorPositions getLevel() {
    return currentLevel;
  }

  public void elevateTo(ElevatorPositions level) {
    this.currentLevel = level;
    this.setPosition(level.height);
  }

  private void setPosition(double position) {
    motor1.setControl(new MotionMagicVoltage(position * ElevatorConstants.CONVERSION_FACTOR));
  }

  public boolean isAtPosition() {
    boolean toReturn = (Math.abs(getError()) <= ElevatorConstants.SETPOINT_TOLERANCE);
    return toReturn;
  }

  public boolean canFunnelTransferCoralToDale() {
    return this.getLevel().equals(Constants.ElevatorConstants.ElevatorPositions.Intake)
        && this.getError() < Constants.ElevatorConstants.MAX_POSITIONAL_ERROR;
  }

  public boolean exampleCondition() {
    // Query some boolean level, such as a digital sensor.
    return false;
  }

  public double getDistance(){
    return distance.getDistance().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // Simulate encoder behavior based on motor speed
    double simulatedSpeed = master.getVelocity().getValueAsDouble();
    double currentPosition = master.getPosition().getValueAsDouble();

    // Update simulated position based on speed (simplified example)
    double newPosition = currentPosition + simulatedSpeed * 0.02; // Assuming a 20ms loop
    master.setPosition(newPosition);

    // Log simulation data for debugging
    SmartDashboard.putNumber("Simulated Position", newPosition);
    SmartDashboard.putNumber("Simulated Speed", simulatedSpeed);
  }

  public boolean atTargetPosition() {
    double tolerance = 0.5;
    return master.getClosedLoopError().getValueAsDouble() <= tolerance;
  }
}

//   public double getSpeed() {
//     return master.getVelocity().getValueAsDouble();
//   }

//   public boolean getStopped() {
//     return getSpeed() == 0;
//   }

//   public void holdPosition() {
//     holdPosition(holdPosValue);
//   }

//   public void holdPosition(double pos) {
//     master.setControl(new MotionMagicVoltage(pos));
//   }

//   public double getPIDError() {
//     return master.getClosedLoopError().getValueAsDouble();
//   }

//   public void setHoldPos() {
//     holdPosValue = master.getPosition().getValueAsDouble();
//   }

//   public void resetEncoderPos() {
//     master.setPosition(0);
//   }

//   public boolean isAtSetpoint() {
//     return Math.abs(getPIDError()) < Constants.ElevatorConstants.SETPOINT_TOLERANCE;
//   }

//   public boolean getBottomLimits() {
//     return master.getSupplyCurrent().getValueAsDouble()
//         > Constants.ElevatorConstants.STATOR_CURRENT_LIMIT;
//   }

//   public double getEncoderPos() {
//     return master.getPosition().getValueAsDouble();
//   }

//   public void setLevelOfElevator(int level) {
//     double targetPosition =
//         switch (level) {  // writing 1 here would be intake level and etc.
//           case 1 -> Constants.ElevatorConstants.INTAKE_LEVEL;
//           case 2 -> Constants.ElevatorConstants.LEVEL_1;
//           case 3 -> Constants.ElevatorConstants.LEVEL_2;
//           case 4 -> Constants.ElevatorConstants.LEVEL_3;
//           case 5 -> Constants.ElevatorConstants.LEVEL_4;
//           default -> throw new IllegalArgumentException("Invalid level: " + level);
//         };

//     holdPosition(targetPosition);
//     SmartDashboard.putString("Elevator Error", "Level: " + level + ", Error: " + getPIDError());
//   }
// }
