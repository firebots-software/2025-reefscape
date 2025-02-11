package frc.robot.commands.DebugCommands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * A command that logs a key-value pair using DogLog.
 *
 * <p>This command supports a wide variety of types through overloaded constructors. When scheduled,
 * it immediately logs the provided value using the corresponding DogLog.log overload and then ends.
 */
public class DogLogCmd extends InstantCommand {

  /**
   * Log a boolean value.
   *
   * @param key the key to log to.
   * @param value the boolean value to log.
   */
  public DogLogCmd(String key, boolean value) {
    super(() -> DogLog.log(key, value));
  }

  /**
   * Log a boolean array.
   *
   * @param key the key to log to.
   * @param value the boolean array to log.
   */
  public DogLogCmd(String key, boolean[] value) {
    super(() -> DogLog.log(key, value));
  }

  /**
   * Log a double value.
   *
   * @param key the key to log to.
   * @param value the double value to log.
   */
  public DogLogCmd(String key, double value) {
    super(() -> DogLog.log(key, value));
  }

  /**
   * Log a double array.
   *
   * @param key the key to log to.
   * @param value the double array to log.
   */
  public DogLogCmd(String key, double[] value) {
    super(() -> DogLog.log(key, value));
  }

  /**
   * Log a float value.
   *
   * @param key the key to log to.
   * @param value the float value to log.
   */
  public DogLogCmd(String key, float value) {
    super(() -> DogLog.log(key, value));
  }

  /**
   * Log a float array.
   *
   * @param key the key to log to.
   * @param value the float array to log.
   */
  public DogLogCmd(String key, float[] value) {
    super(() -> DogLog.log(key, value));
  }

  /**
   * Log an int array.
   *
   * @param key the key to log to.
   * @param value the int array to log.
   */
  public DogLogCmd(String key, int[] value) {
    super(() -> DogLog.log(key, value));
  }

  /**
   * Log a long value.
   *
   * @param key the key to log to.
   * @param value the long value to log.
   */
  public DogLogCmd(String key, long value) {
    super(() -> DogLog.log(key, value));
  }

  /**
   * Log a long array.
   *
   * @param key the key to log to.
   * @param value the long array to log.
   */
  public DogLogCmd(String key, long[] value) {
    super(() -> DogLog.log(key, value));
  }

  /**
   * Log a String value.
   *
   * @param key the key to log to.
   * @param value the String value to log.
   */
  public DogLogCmd(String key, String value) {
    super(() -> DogLog.log(key, value));
  }

  /**
   * Log a String array.
   *
   * @param key the key to log to.
   * @param value the String array to log.
   */
  public DogLogCmd(String key, String[] value) {
    super(() -> DogLog.log(key, value));
  }
}
