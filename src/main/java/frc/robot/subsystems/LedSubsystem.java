// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class LedSubsystem extends SubsystemBase {

  private static final int STRIP_LENGTH = 50;

  private static final int LED_OFFSET = 8;

  private static final FireAnimation FLAME_ANIMATION =
      new FireAnimation(1, 0.15, STRIP_LENGTH, 0.4, 0.4, false, LED_OFFSET);
  private static final SingleFadeAnimation MATCH_IDLE_ANIMATION =
      new SingleFadeAnimation(255, 0, 255, 0, 0, STRIP_LENGTH, LED_OFFSET);
  private static final StrobeAnimation INTAKE_FLASH_ANIMATION =
      new StrobeAnimation(242, 200, 82, 0, 0, STRIP_LENGTH, LED_OFFSET);
  private static final StrobeAnimation ALIGNMENT_FLASH_ANIMATION =
      new StrobeAnimation(30, 150, 130, 0, 0, STRIP_LENGTH, LED_OFFSET);
  private static final StrobeAnimation SCORING_SOLID_ANIMATION =
      new StrobeAnimation(32, 250, 61, 0, 1, STRIP_LENGTH, LED_OFFSET);

  public enum LedState {
    IDLE("idle", null),
    SETUP("setup", null),
    MATCH_IDLE("match_idle", MATCH_IDLE_ANIMATION),
    INTAKE_FLASH("intake_flash", INTAKE_FLASH_ANIMATION),
    ALIGNMENT_FLASH("alignment_flash", ALIGNMENT_FLASH_ANIMATION),
    SCORING_SOLID("scoring_solid", SCORING_SOLID_ANIMATION),
    FLAME("flame", FLAME_ANIMATION);

    private final String name;
    private final Animation animation;

    LedState(String name, Animation animation) {
      this.name = name;
      this.animation = animation;
    }
  }

  private LedState currentState = LedState.SCORING_SOLID;
  private LedState lastState = LedState.IDLE;
  private final CANdle candle;

  /** Creates a new ExampleSubsystem. */
  public LedSubsystem() {
    this.candle = new CANdle(39);
    candle.configVBatOutput(VBatOutputMode.On);
    candle.configBrightnessScalar(1);
  }

  public void updateAlliance(Alliance a) {
    if (a == Alliance.Red) {
      MATCH_IDLE_ANIMATION.setB(0);
      MATCH_IDLE_ANIMATION.setR(255);
    } else if (a == Alliance.Blue) {
      MATCH_IDLE_ANIMATION.setB(255);
      MATCH_IDLE_ANIMATION.setR(0);
    }
  }

  /**
   * Updates the LEDs to change their behaviour based on the LedState passed in.
   *
   * @return a command that updates the LEDs State
   */
  public Command updateLedsCommand(LedState state) {
    return runOnce(
        () -> {
          this.currentState = state;
        });
  }

  /**
   * updateLedsForSensors is a command that is used during robot setup to help check the various
   * sensors on the robot. Will need to be connected to the sensors' subsystems on the actual bot
   */
  public void updateLedsForSensors(
      BooleanSupplier inSensor, BooleanSupplier funnelSensor, BooleanSupplier elevatorSensor) {
    if (inSensor.getAsBoolean()) {
      candle.setLEDs(0, 255, 0, 0, 0, 2);
    } else {
      candle.setLEDs(0, 0, 0, 0, 0, 2);
    }

    if (funnelSensor.getAsBoolean()) {
      candle.setLEDs(0, 255, 0, 0, 2, 2);
    } else {
      candle.setLEDs(0, 0, 0, 0, 2, 2);
    }

    if (elevatorSensor.getAsBoolean()) {
      candle.setLEDs(0, 255, 0, 0, 4, 2);
    } else {
      candle.setLEDs(0, 0, 0, 0, 4, 2);
    }
  }

  public void ledsOff() {
    candle.setLEDs(0, 0, 0, 0, 0, STRIP_LENGTH + 8);
  }

  @Override
  public void periodic() {
    if (currentState != lastState) {
      lastState = currentState;

      if (currentState.animation != null) {
        System.out.println("Update animation to " + currentState.name);
        candle.animate(currentState.animation);
      } else {
        ledsOff();
      }
    }

    if (currentState == LedState.SETUP) {
      updateLedsForSensors(null, null, null);
    }

    DogLog.log("Subsystem/LED/LEDstate", this.currentState.name);
  }
}
