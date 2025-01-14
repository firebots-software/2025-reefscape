// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.util.LoggedTalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static ElevatorSubsystem instance;

  private LoggedTalonFX motor1;
  private LoggedTalonFX motor2;
  private LoggedTalonFX master;
  private Double holdPosValue;
  
  
    public ElevatorSubsystem() {
      // Initialize motors
      motor1 = new LoggedTalonFX(0);
      motor2 = new LoggedTalonFX(0);
  
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
  
      m1Config.apply(moc);
      m2Config.apply(moc);
  
      // master?
      // Apply Current Limit to all motors
      m1Config.apply(clc);
      m2Config.apply(clc);
  
      // Assign master motor and apply Slot0Configs to master
      master = motor1;
      TalonFXConfigurator masterConfigurator = master.getConfigurator();
      masterConfigurator.apply(s0c);
  
      // Apply MotionMagicConfigs to master motor
      MotionMagicConfigs mmc = new MotionMagicConfigs();
    //  mmc.MotionMagicCruiseVelocity =
    //      Constants.ElevatorConstants;
    //  mmc.MotionMagicAcceleration =
        //  Constants.Arm.MOTIONMAGIC_KA * Constants.Arm.INTEGRATED_ARM_CONVERSION_FACTOR;
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
      master.setControl(new MotionMagicVoltage(speed));
    }
  
    public double getSpeed(){
      var velocity = master.getVelocity();
      return velocity.getValueAsDouble();
    } 
    public boolean getStopped(){
      return getSpeed() == 0;
    }
    public void holdPosition() {
      //convert pos to voltage
      master.setControl(new MotionMagicVoltage(0));
    }
  
    public void holdPosition(double pos) {
      //convert pos to voltage
      master.setControl(new MotionMagicVoltage(pos));
    }
  
    public double getPIDError() {
      var error = master.getClosedLoopError();
      return error.getValueAsDouble();
    }
  
    public void setHoldPos() {
      var pos = master.getPosition();
      holdPosValue = pos.getValueAsDouble();
  }

  public void resetEncoderPos() {
    master.setPosition(0);
  }

  public boolean isAtSetpoint() {
    return getPIDError() < Constants.ElevatorConstants.S0C_KP;

  }

  public boolean getBottomLimits() {
    if(master.getSupplyCurrent().getValue().magnitude() > Constants.ElevatorConstants.currentLimit){
      return true;
    }else{
      return false;
    }
  }

  public double getEncoderPos() {
    var encoderPos = master.getPosition();
    return encoderPos.getValueAsDouble();
  }
  // setting levels for certain positions in the field; must use constants for converting motors to rotational angle
  public void setLevelOfElavat(int level) {
  if (level == 1){
      master.setControl(new MotionMagicVoltage(null));
      holdPosValue = Constants.ElevatorConstants.level1;
      holdPosition();
      SmartDashboard.putString("elevator error", "level: " + level + ", Error: " + getPIDError());
  } else if (level == 2) {
      master.setControl(new MotionMagicVoltage(null));
      holdPosValue = Constants.ElevatorConstants.level2;
      holdPosition();
      SmartDashboard.putString("elevator error", "level: " + level + ", Error: " + getPIDError());
    } else if (level == 3) {
      master.setControl(new MotionMagicVoltage(null));
      holdPosValue = Constants.ElevatorConstants.level3;
      holdPosition();
      SmartDashboard.putString("elevator error", "level: " + level + ", Error: " + getPIDError());
    }
    else if (level == 4) {
      master.setControl(new MotionMagicVoltage(null));
      holdPosValue = Constants.ElevatorConstants.level4;
      holdPosition();
      SmartDashboard.putString("elevator error", "level: " + level + ", Error: " + getPIDError());
  }
  }

  /**
   * 
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
      holdPosValue = 0.0;
      master.setControl(new MotionMagicVoltage(0));
    }
    DogLog.log("HoldPosValue", holdPosValue);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}