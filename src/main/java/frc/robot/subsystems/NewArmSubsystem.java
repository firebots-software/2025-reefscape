// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class NewArmSubsystem extends SubsystemBase {

  private static NewArmSubsystem instance;

  private LoggedTalonFX armMotor;
  private DutyCycleEncoder revEncoder;
  private ArmFeedforward armFeedForward;
  private MotionMagicConfigs motionMagicConfigs;

  private boolean initialized = false;
  private double targetDegrees;
  private double armHorizontalOffset;

  public static NewArmSubsystem getInstance() {
    if (instance == null) {
      instance = new NewArmSubsystem();
    }
    return instance;
  }

  /** Creates a new NewArmSubsystem. */
  public NewArmSubsystem() {
    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.Arm.ARM_STATOR_CURRENT_LIMIT_AMPS)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.Arm.ARM_SUPPLY_CURRENT_LIMIT_AMPS);
    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    Slot0Configs s0c = new Slot0Configs().withKP(Constants.Arm.S0C_KP).withKI(0).withKD(0);
    armFeedForward =
        new ArmFeedforward(Constants.Arm.ARMFF_KS, Constants.Arm.ARMFF_KG, Constants.Arm.ARMFF_KV);

    // Initialize master motor only
    armMotor = new LoggedTalonFX(Constants.Arm.LT_PORT);

    TalonFXConfigurator masterConfigurator = armMotor.getConfigurator();
    masterConfigurator.apply(moc);
    masterConfigurator.apply(clc); // Apply current limits to the master motor
    masterConfigurator.apply(s0c); // Apply PID settings to the master motor

    // Apply MotionMagicConfigs to master motor
    motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Arm.MOTIONMAGIC_KV;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Arm.MOTIONMAGIC_KA;
    masterConfigurator.apply(motionMagicConfigs);

    // Initialize absolute encoder
    revEncoder = new DutyCycleEncoder(Constants.Arm.ENCODER_PORT);

    // Wait until absolute encoder is connected and set master motor's position
    new Thread(
            () -> {
              try {
                do {
                  Thread.sleep(250);
                } while (!revEncoder.isConnected());

                armMotor.setPosition(revEncoder.get());
                initialized = true;
                SmartDashboard.putBoolean("Motor encoder init", initialized);

              } catch (InterruptedException e) {
                e.printStackTrace();
              }
            })
        .run();

    // targetDegrees = getCorrectedDegrees() + 10d;
    // enableArm = false;
  }

  public void setPosition(double angleDegrees) {
    // angleDegrees = MathUtil.clamp(angleDegrees, 198, 351);
    targetDegrees = angleDegrees;
    if (initialized) {
      armMotor.setControl(new MotionMagicVoltage(getRotationFromAngle(angleDegrees)));
      // .withFeedForward(
      //    armFeedForward.calculate(Units.degreesToRadians(getDegrees() - 198), 0)));
    }
  }

  private double getDegrees() {
    return revEncoder.get() * 360d;
  }

  private double getRotationFromAngle(double angleDegrees) {

    return angleDegrees / 360d;
  }

  public boolean atTarget(double endToleranceDegrees) {
    if (getDegrees() < targetDegrees + endToleranceDegrees
        && getDegrees() > targetDegrees - endToleranceDegrees) {
      return true;
    } else return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // setPosition(270);
    SmartDashboard.putNumber("ARM Abs Enc Raw", revEncoder.get());
    SmartDashboard.putNumber("ARM Arm Degrees", getDegrees());
    SmartDashboard.putNumber("ARM Target Degrees", targetDegrees);
  }
}
