// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.util.LoggedTalonFX;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static ElevatorSubsystem instance;

  private LoggedTalonFX motor1;
  private LoggedTalonFX motor2;

  private MotionMagicConfigs mmc;

  private ElevatorSubsystem() {
    // Initialize motors
    // TODO: Would it make more sense to call these motorup/down?
    motor1 = new LoggedTalonFX(Constants.ElevatorConstants.MOTOR1_PORT);
    motor2 = new LoggedTalonFX(Constants.ElevatorConstants.MOTOR2_PORT);

    // Set up motor followers and deal with inverted motors
    Follower invertedFollower = new Follower(Constants.ElevatorConstants.MOTOR1_PORT, false);
    motor2.setControl(invertedFollower);

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(Constants.ElevatorConstants.S0C_KP)
            .withKI(Constants.ElevatorConstants.S0C_KI)
            .withKD(Constants.ElevatorConstants.S0C_KD)
            .withKS(0);


    motor1.updateCurrentLimits(
        Constants.ElevatorConstants.STATOR_CURRENT_LIMIT,
        Constants.ElevatorConstants.SUPPYLY_CURRENT_LIMIT);
    motor2.updateCurrentLimits(
        Constants.ElevatorConstants.STATOR_CURRENT_LIMIT,
        Constants.ElevatorConstants.SUPPYLY_CURRENT_LIMIT);

    TalonFXConfigurator m1Config = motor1.getConfigurator();
    TalonFXConfigurator m2Config = motor2.getConfigurator();

    // Apply Slot0Configs to motor1(master)
    TalonFXConfigurator motor1Configurator = motor1.getConfigurator();
    motor1Configurator.apply(s0c);

    // Apply MotionMagic to motor1(master)
    mmc = new MotionMagicConfigs();
    mmc.MotionMagicCruiseVelocity = Constants.ElevatorConstants.MOTIONMAGIC_KV;
    mmc.MotionMagicAcceleration = Constants.ElevatorConstants.MOTIONMAGIC_KA;
    motor1Configurator.apply(mmc);

    m1Config.apply(mmc);
    m2Config.apply(mmc);

    // Apply Slot0Configs to master
    TalonFXConfigurator masterConfigurator = motor1.getConfigurator();
    masterConfigurator.apply(s0c);
  }

  // subsystem for elavator commands
  public static ElevatorSubsystem getInstance() {
    if (instance == null) {
      instance = new ElevatorSubsystem();
    }
    return instance;
  }

  public void elevate(ElevatorPositions position) {
    double height = position.height;
    this.setPosition(height);
  }

  private void setPosition(double position){
    motor1.setControl(new MotionMagicVoltage(position));
  }

  // public void elevate(double speed) {
  //   motor1.setControl(ControlMode.PercentOutput, speed);
  // }

  // public double getSpeed(){
  //   return motor1.getSelectedSensorVelocity();
  // }
  // public boolean getStopped(){
  //   return getSpeed() == 0;
  // }
  // public void holdPosition() {
  //   motor1.set(ControlMode.Position, holdPosValue);
  // }

  // public void holdPosition(double pos) {
  //   motor1.set(ControlMode.Position, pos);
  // }

  // public double getPIDError() {
  //   return motor1.getClosedLoopError();
  // }

  // public void setHoldPos() {
  //   holdPosValue = motor1.getSelectedSensorPosition();
  // }

  // public void resetEncoderPos() {
  //   motor1.setSelectedSensorPosition(0);
  // }

  // public boolean isAtSetpoint() {
  //   return motor1.isMotionProfileFinished();
  // }

  // public boolean getBottomLimits() {
  //   if(motor1.getMotorOutputCurrent() > Constant.currentlimit){
  //     return true;
  //   }else{
  //     return false;
  //   }
  // }

  // public double getEncoderPos() {
  //   return motor1.getSelectedSensorPosition();
  // }
  // // setting levels for certain positions in the field; must use constants for converting motors
  // to rotational angle
  // public void setLevelOfElavat(int level) {
  // if (level == 1){
  //    motor1.set(ControlMode.MotionMagic, ElevatorConstants.level1);
  //     holdPosValue = ElevatorConstants.level1;
  //     holdPosition();
  //     SmartDashboard.putString("elevator error", "level: " + level + ", Error: " +
  // getPIDError());
  // } else if (level == 2) {
  //     motor1.set(ControlMode.MotionMagic, ElevatorConstants.level2);
  //     holdPosValue = ElevatorConstants.level2;
  //     holdPosition();
  //     SmartDashboard.putString("elevator error", "level: " + level + ", Error: " +
  // getPIDError());
  //   } else if (level == 3) {
  //     motor1.set(ControlMode.MotionMagic, ElevatorConstants.level4);
  //     holdPosValue = ElevatorConstants.level4;
  //     holdPosition();
  //     SmartDashboard.putString("elevator error", "level: " + level + ", Error: " +
  // getPIDError());
  //   }
  //   else if (level == 4) {
  //     motor1.set(ControlMode.MotionMagic, ElevatorConstants.level4);
  //     holdPosValue = ElevatorConstants.level4;
  //     holdPosition();
  //     SmartDashboard.putString("elevator error", "level: " + level + ", Error: " +
  // getPIDError());
  // }

  /**
   * An example method querying a boolean level of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem level, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean level, such as a digital sensor.
    return false;
  }

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  //   if (getBottomLimits()) {
  //     resetEncoderPos();
  //     holdPosValue = 0;
  //     motor1.set(ControlMode.PercentOutput, 0);
  //   }
  // }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
