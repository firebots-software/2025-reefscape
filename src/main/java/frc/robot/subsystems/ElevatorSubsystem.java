// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static ElevatorSubsystem instance;

  private LoggedTalonFX motor1;
  private LoggedTalonFX motor2;


  public ElevatorSubsystem() {
    // Initialize motors
    motor1 = new LoggedTalonFX(0, 0);
    motor2 = new LoggedTalonFX(0, 0);

    // Set up motor followers and deal with inverted motors
    Follower invertedFollower = new Follower(0, false);
    motor2.setControl(invertedFollower);

    TalonFXConfigurator m1Config = motor1.getConfigurator();
    TalonFXConfigurator m2Config = motor2.getConfigurator();

    // motion profiling
    CurrentLimitsConfigs clc =
    new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(0);
    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    Slot0Configs s0c = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    eleff =
        new eleFeedforward(0, 0,0);

    m1Config.apply(moc);
    m2Config.apply(moc);

    // TODO: Why do we apply Current Limit Configs to each motor, but then only do s0c on the
    // master?
    // Apply Current Limit to all motors
    m1Config.apply(clc);
    m2Config.apply(clc);

    // Assign master motor and apply Slot0Configs to master
    master = motor1;
    TalonFXConfigurator masterConfigurator = master.getConfigurator();
    masterConfigurator.apply(s0c);

    // Apply MotionMagicConfigs to master motor
    mmc = new MotionMagicConfigs();
    mmc.MotionMagicCruiseVelocity =
        Constants.Arm.MOTIONMAGIC_KV * Constants.Arm.INTEGRATED_ARM_CONVERSION_FACTOR;
    mmc.MotionMagicAcceleration =
        Constants.Arm.MOTIONMAGIC_KA * Constants.Arm.INTEGRATED_ARM_CONVERSION_FACTOR;
    // mmc.MotionMagicJerk = 1600;
    masterConfigurator.apply(mmc);
  }
  //subsystem for elavator commands
  public static ElevatorSubsystem getInstance() {
      if (instance == null) {
        instance = new ElevatorSubsystem();
    }
      return instance;
  }

  public void elevate(double speed) {
    master.set(ControlMode.PercentOutput, speed);
  }

  public double getSpeed(){
    return master.getSelectedSensorVelocity();
  } 
  public boolean getStopped(){
    return getSpeed() == 0;
  }
  public void holdPosition() {
    master.set(ControlMode.Position, holdPosValue);
  }

  public void holdPosition(double pos) {
    master.set(ControlMode.Position, pos);
  }

  public double getPIDError() {
    return master.getClosedLoopError();
  }

  public void setHoldPos() {
    holdPosValue = master.getSelectedSensorPosition();
  }

  public void resetEncoderPos() {
    master.setSelectedSensorPosition(0);
  }

  public boolean isAtSetpoint() {
    return master.isMotionProfileFinished();
  }

  public boolean getBottomLimits() {
    if(master.getMotorOutputCurrent() > Constant.currentlimit){
      return true;
    }else{
      return false;
    }
  }

  public double getEncoderPos() {
    return master.getSelectedSensorPosition();
  }
  // setting levels for certain positions in the field; must use constants for converting motors to rotational angle
  public void setLevelOfElavat(int level) {
  if (level == 1){
     master.set(ControlMode.MotionMagic, ElevatorConstants.level1);
      holdPosValue = ElevatorConstants.level1;
      holdPosition();
      SmartDashboard.putString("elevator error", "level: " + level + ", Error: " + getPIDError());
  } else if (level == 2) {
      master.set(ControlMode.MotionMagic, ElevatorConstants.level2);
      holdPosValue = ElevatorConstants.level2;
      holdPosition();
      SmartDashboard.putString("elevator error", "level: " + level + ", Error: " + getPIDError());
    } else if (level == 3) {
      master.set(ControlMode.MotionMagic, ElevatorConstants.level4);
      holdPosValue = ElevatorConstants.level4;
      holdPosition();
      SmartDashboard.putString("elevator error", "level: " + level + ", Error: " + getPIDError());
    }
    else if (level == 4) {
      master.set(ControlMode.MotionMagic, ElevatorConstants.level4);
      holdPosValue = ElevatorConstants.level4;
      holdPosition();
      SmartDashboard.putString("elevator error", "level: " + level + ", Error: " + getPIDError());
  }
  }

  /**
   * An example method querying a boolean level of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem level, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean level, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getBottomLimits()){
      resetEncoderPos();
      holdPosValue = 0;
      master.set(ControlMode.PercentOutput, 0);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}