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
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class ArmSubsystem extends SubsystemBase {

  private static ArmSubsystem instance;

  private LoggedTalonFX armMotor;
  private DutyCycleEncoder revEncoder;
  private ArmFeedforward armFeedForward;
  private MotionMagicConfigs motionMagicConfigs;
  private final MotionMagicVoltage controlRequest = new MotionMagicVoltage(0);
  private double encoderDegrees;
  private TalonFXSimState armSim;
  private static final double kGearRatio = 10.0;
  private LinearSystem<N2, N1, N2> elevatorSystem = new 
  private final DCMotorSim m_motorSimModel =
    new DCMotorSim(kGearRatio,DCMotor.getKrakenX60Foc(1), 0.001);
  private double targetDegrees;

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

  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  public ArmSubsystem() {
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
    armSim = armMotor.getSimState();
    armSim.Orientation = ChassisReference.CounterClockwise_Positive;
    armSim.setSupplyVoltage(12);
  }

  public void setPosition(double angleDegrees) {
    targetDegrees = angleDegrees;
    armMotor.setControl(
        controlRequest.withPosition(Constants.Arm.ANGLE_TO_ENCODER_ROTATIONS(angleDegrees)));
  }

  private double calculateDegrees() {
    return revEncoder.get() * 360d;
  }

  public boolean atTarget(double endToleranceDegrees) {
    if (encoderDegrees < targetDegrees + endToleranceDegrees
        && encoderDegrees > targetDegrees - endToleranceDegrees) {
      return true;
    } else return false;
  }

  public void moveMuyNegative() {
    double veryNegativeNumberToTurnTo = -1000d;
    armMotor.setControl(
        controlRequest
            .withPosition(Constants.Arm.ANGLE_TO_ENCODER_ROTATIONS(veryNegativeNumberToTurnTo))
            .withSlot(0));
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



    encoderDegrees = calculateDegrees();
    // This method will be called once per scheduler run
    DogLog.log("Arm at target", atTarget(5));
    DogLog.log("Arm Degrees", encoderDegrees);
    DogLog.log("Arm Target Degrees", targetDegrees);
  }
}
