// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Seconds), // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> armMotor.setControl(m_voltReq.withOutput(volts.in(Volts))), null, this));

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
    moc.withInverted(InvertedValue.CounterClockwise_Positive);
    FeedbackConfigs fc = new FeedbackConfigs();
    ClosedLoopGeneralConfigs clgc = new ClosedLoopGeneralConfigs();
    Slot0Configs s0c = new Slot0Configs().withKP(Constants.Arm.S0C_KP * 0.75).withKI(0).withKD(0);

    armFeedForward =
        new ArmFeedforward(Constants.Arm.ARMFF_KS, Constants.Arm.ARMFF_KG, Constants.Arm.ARMFF_KV);

    // Initialize master motor only
    armMotor = new LoggedTalonFX(Constants.Arm.LT_PORT);

    TalonFXConfigurator masterConfigurator = armMotor.getConfigurator();

    masterConfigurator.apply(moc);
    masterConfigurator.apply(clc); // Apply current limits to the master motor
    masterConfigurator.apply(s0c); // Apply PID settings to the master motor
    masterConfigurator.apply(fc);
    masterConfigurator.apply(clgc);

    // Apply MotionMagicConfigs to master motor
    motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Arm.MOTIONMAGIC_KV;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Arm.MOTIONMAGIC_KA;
    masterConfigurator.apply(motionMagicConfigs);

    // Initialize absolute encoder
    revEncoder = new DutyCycleEncoder(Constants.Arm.ENCODER_PORT);

    // Wait until absolute encoder is connected and set master motor's position
    // new Thread(
    //         () -> {
    //           try {
    //             do {
    //               Thread.sleep(250);
    //             } while (!revEncoder.isConnected());

    //             // armMotor.setPosition(revEncoder.get() * 54);
    //             initialized = true;
    //             SmartDashboard.putBoolean("Motor encoder init", initialized);

    //           } catch (InterruptedException e) {
    //             e.printStackTrace();
    //           }
    //         })
    //     .run();

    // targetDegrees = getCorrectedDegrees() + 10d;
    // enableArm = false;
  }

  public void setPosition(double angleDegrees) {

    // angleDegrees = MathUtil.clamp(angleDegrees, 202, 351);
    targetDegrees = angleDegrees;
    armMotor.setControl(new MotionMagicVoltage((0.159344d * angleDegrees)).withSlot(0));
    // .withFeedForward(
    //     armFeedForward.calculate(Units.degreesToRadians(getDegrees() - 198), 0)));
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
      // armMotor.disable();
      return true;
    } else return false;
  }

  public void moveMuyNegative() {
    armMotor.setControl(new MotionMagicVoltage((0.159344d * (-1000)) + 0.355).withSlot(0));
  }

  public boolean checkCurrent() {
    double current = armMotor.getTorqueCurrent().getValue().magnitude();

    if (current < -10) {
      armMotor.disable();
      return true;
    }

    return false;
  }

  public void zeroSensor() {
    armMotor.setPosition(0);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("ARM at target", atTarget(5));
    SmartDashboard.putNumber("ARM Abs Enc Raw", revEncoder.get());
    SmartDashboard.putNumber("ARM Arm Degrees", getDegrees());
    SmartDashboard.putNumber("ARM Abs -> Rel Val", revEncoder.get() * 54);
    SmartDashboard.putNumber("ARM Target Degrees", targetDegrees);
  }
}
