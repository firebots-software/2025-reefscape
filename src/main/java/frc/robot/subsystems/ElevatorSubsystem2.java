
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
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.LoggedTalonFX;;
// import frc.robot.Constants.Elevator.ElevatorPositions;

public class ElevatorSubsystem2 extends SubsystemBase {
    private TalonFX motor1;
    private TalonFX motor2;
    public TalonFX master;
    public double targetHeight;

    private MotionMagicConfigs mmc;

    private final MotionMagicVoltage controlRequest = new MotionMagicVoltage(0);

    public ElevatorSubsystem2() {
        motor1 =
        new TalonFX(
            ElevatorConstants.MOTOR1_PORT,
            Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);
        motor2 =
        new LoggedTalonFX(
            "subsystems/Elevator/motor2",
            ElevatorConstants.MOTOR2_PORT,
            Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);

        Follower follower = new Follower(ElevatorConstants.MOTOR1_PORT, false);
        motor2.setControl(follower);


        Slot1Configs s1c =
        new Slot1Configs()
            // .withKP(ElevatorConstants.S1C_KP)
            // .withKI(ElevatorConstants.S1C_KI)
            // .withKD(ElevatorConstants.S1C_KD)
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

        ((LoggedTalonFX) motor1).updateCurrentLimits(
            ElevatorConstants.STATOR_CURRENT_LIMIT_AMPS, ElevatorConstants.SUPPLY_CURRENT_LIMIT_AMPS);
        ((LoggedTalonFX) motor2).updateCurrentLimits(
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
    }

    public void setPosition(double height) {
        if (height < Constants.ElevatorConstants.minHeight) height = Constants.ElevatorConstants.minHeight;
        if (height > Constants.ElevatorConstants.maxHeight) height = Constants.ElevatorConstants.maxHeight;
        
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

    public void resetElevatorPositionToZero() {
        master.setPosition(0);
    }

    public double getHeight() {
        return master.getRotorPosition().getValueAsDouble()* ElevatorConstants.CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS/ ElevatorConstants.CARRAIGE_UPDUCTION;
    }

    public void periodic() {
        DogLog.log(
            "subsystems/Elevator/currentheight",
            getHeight());
    }
}