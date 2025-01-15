package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem instance;

  private LoggedTalonFX master; // Only one motor (master) is used now
  private DutyCycleEncoder revEncoder;
  private boolean enableArm;
  private ArmFeedforward armff;
  private MotionMagicConfigs mmc;

  private boolean initialized = false;

  private double targetDegrees;
  private double armHorizontalOffset = 0.0d;

  public ArmSubsystem() {
    // Initialize Current Limit, Slot0Configs, and ArmFeedForward
    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.Arm.ARM_STATOR_CURRENT_LIMIT_AMPS)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.Arm.ARM_SUPPLY_CURRENT_LIMIT_AMPS);
    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    Slot0Configs s0c = new Slot0Configs().withKP(Constants.Arm.S0C_KP).withKI(0).withKD(0);
    armff =
        new ArmFeedforward(Constants.Arm.ARMFF_KS, Constants.Arm.ARMFF_KG, Constants.Arm.ARMFF_KV);

    // Initialize master motor only
    master = new LoggedTalonFX(Constants.Arm.LT_PORT);

    TalonFXConfigurator masterConfigurator = master.getConfigurator();
    masterConfigurator.apply(moc);
    masterConfigurator.apply(clc); // Apply current limits to the master motor
    masterConfigurator.apply(s0c); // Apply PID settings to the master motor

    // Apply MotionMagicConfigs to master motor
    mmc = new MotionMagicConfigs();
    mmc.MotionMagicCruiseVelocity = Constants.Arm.MOTIONMAGIC_KV;
    mmc.MotionMagicAcceleration = Constants.Arm.MOTIONMAGIC_KA;
    masterConfigurator.apply(mmc);

    // Initialize absolute encoder
    revEncoder = new DutyCycleEncoder(Constants.Arm.ENCODER_PORT);

    // Wait until absolute encoder is connected and set master motor's position
    new Thread(
            () -> {
              try {
                do {
                  Thread.sleep(250);
                } while (!revEncoder.isConnected());
                master.setPosition((getAbsolutePosition()));
                initialized = true;

              } catch (InterruptedException e) {
                e.printStackTrace();
              }
            })
        .run();

    targetDegrees = getCorrectedDegrees() + 10d;
    enableArm = false;
  }

  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  public void resetPosition() {
    if (revEncoder.isConnected()) {
      master.setPosition((getAbsolutePosition()));
    }
  }

  private void setPosition(double angleDegrees) {
    angleDegrees = MathUtil.clamp(angleDegrees, 198, 351);
    // if (initialized && enableArm) {
    master.setControl(
        new MotionMagicVoltage(calculateIntegratedTargetRots(angleDegrees))
            .withFeedForward(
                armff.calculate((2 * Math.PI * getRawDegrees().magnitude()) / 360d, 0)));
    // }
  }

  public void setTargetDegrees(double angleDegrees) {
    targetDegrees = angleDegrees;
    setPosition(targetDegrees);
  }

  private double getAbsolutePosition() {
    return (revEncoder.get()
            - Constants.Arm.ABSOLUTE_ENCODER_HORIZONTAL
            + Constants.Arm.ABSOLUTE_HORIZONTAL_OFFSET
            + 1d)
        % 1;
  }

  private Angle getMotorPosRotations() {
    if (!initialized) {
      System.out.println(
          "WARNING: Motor Position looked at, but initialization not complete yet. Returning 0");
      return Degrees.of(0);
    }
    return master.getPosition().getValue();
  }

  private Angle getArmPosRotations() {
    return Rotations.of(getMotorPosRotations().magnitude());
  }

  public Angle getRawDegrees() {
    return Degrees.of(getArmPosRotations().magnitude() * 360d);
  }

  private double calculateIntegratedTargetRots(double angleDegrees) {
    double armRots = (angleDegrees - (revEncoder.get() * 360)) / 360d + armHorizontalOffset;
    return armRots;
  }

  public double getCorrectedDegrees() {
    return getRawDegrees().magnitude();
  }

  public boolean atTarget(double tolerance) {
    return Math.abs(targetDegrees - getCorrectedDegrees()) < tolerance;
  }

  public void setEnable(boolean toset) {
    this.enableArm = toset;
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putString(
        "ARM Command",
        this.getCurrentCommand() == null ? "none" : this.getCurrentCommand().getName());
    SmartDashboard.putNumber("ARM Abs Enc Raw", revEncoder.get());
    SmartDashboard.putNumber("ARM Abs Enc Func", getAbsolutePosition());
    SmartDashboard.putNumber("ARM Integrated Rotations", getMotorPosRotations().magnitude());
    SmartDashboard.putNumber(
        "ARM Integrated Current", master.getSupplyCurrent().getValue().magnitude());
    SmartDashboard.putNumber("ARM Integrated Error", master.getClosedLoopError().getValue());
    SmartDashboard.putNumber("ARM Arm Rotations", getArmPosRotations().magnitude());
    SmartDashboard.putNumber("ARM Arm Degrees", getRawDegrees().magnitude());
    SmartDashboard.putNumber("ARM Arm Degrees Corrected", getCorrectedDegrees());
    SmartDashboard.putNumber("ARM Target Degrees", targetDegrees);
    SmartDashboard.putNumber(
        "ARM Target Integrated Rots", calculateIntegratedTargetRots(targetDegrees));
    SmartDashboard.putNumber(
        "ARM FeedForward Calculations",
        armff.calculate((2 * Math.PI * getRawDegrees().magnitude()) / 360d, 0));
    SmartDashboard.putNumber("Master Velocity", master.getVelocity().getValue().magnitude());

    SmartDashboard.putNumber("Encoder Position", revEncoder.get());

    periodicSignalLogger();
  }

  public void periodicSignalLogger() {
    SignalLogger.writeDouble("ARM Abs Enc Func: ", getAbsolutePosition());
    SignalLogger.writeDouble(
        "ARM Integrated Current: ", master.getSupplyCurrent().getValue().magnitude());
    SignalLogger.writeDouble("ARM Integrated Error: ", master.getClosedLoopError().getValue());
    SignalLogger.writeDouble("Arm Corrected Degrees", getCorrectedDegrees());
    SignalLogger.writeDouble("Target Arm Degrees", targetDegrees);
    SignalLogger.writeDouble("Master Velocity", master.getVelocity().getValue().magnitude());
  }
}
