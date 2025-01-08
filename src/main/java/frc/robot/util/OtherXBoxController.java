package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;

// TODO: SEE IF THIS IS STILL NECESSARY (WE USED THIS DUE TO BOOLEAN EVENT LOOP SLOWDOWN)
public class OtherXBoxController extends CommandXboxController {
  final Trigger a, b, x, y;
  final Trigger leftBump, rightBump;
  final Trigger start, back;
  final Trigger leftStick, rightStick;
  // final Trigger POV;
  double threshold;
  int port;
  public static class OI {
    public static final double LEFT_JOYSTICK_DEADBAND = 0.07;
    public static final double RIGHT_JOYSTICK_DEADBAND = 0.07;
    public static final int JOYSTICK_A_PORT = 0;

    public enum XBoxButtonID {
      /** A. */
      A(1),
      /** B. */
      B(2),
      /** X. */
      X(3),
      /** Y. */
      Y(4),
      /** Left bumper. */
      LeftBumper(5),
      /** Right bumper. */
      RightBumper(6),
      /** Left stick. */
      LeftStick(9),
      /** Right stick. */
      RightStick(10),
      /** Back. */
      Back(7),
      /** Start. */
      Start(8);
      public final int value;

      XBoxButtonID(int value) {
        this.value = value;
      }
    }

    public enum AxisID {
      /** Left X. */
      LeftX(0),
      /** Right X. */
      RightX(4),
      /** Left Y. */
      LeftY(1),
      /** Right Y. */
      RightY(5),
      /** Left trigger. */
      LeftTrigger(2),
      /** Right trigger. */
      RightTrigger(3);

      /** Axis value. */
      public final int value;

      AxisID(int value) {
        this.value = value;
      }
    }
  }

  public OtherXBoxController(int port) {
    super(port);
    this.port = port;
    a = new JoystickButton(getHID(), OI.XBoxButtonID.A.value);
    b = new JoystickButton(getHID(), OI.XBoxButtonID.B.value);
    x = new JoystickButton(getHID(), OI.XBoxButtonID.X.value);
    y = new JoystickButton(getHID(), OI.XBoxButtonID.Y.value);
    start = new JoystickButton(getHID(), OI.XBoxButtonID.Start.value);
    back = new JoystickButton(getHID(), OI.XBoxButtonID.Back.value);
    leftStick = new JoystickButton(getHID(), OI.XBoxButtonID.LeftStick.value);
    rightStick = new JoystickButton(getHID(), OI.XBoxButtonID.RightStick.value);
    leftBump = new JoystickButton(getHID(), OI.XBoxButtonID.LeftBumper.value);
    rightBump = new JoystickButton(getHID(), OI.XBoxButtonID.RightBumper.value);
    // POV = new
    threshold = 0.5;
  }

  @Override
  public Trigger a() {
    return a;
  }

  @Override
  public Trigger b() {
    return b;
  }

  @Override
  public Trigger x() {
    return x;
  }

  @Override
  public Trigger y() {
    return y;
  }

  @Override
  public Trigger leftBumper() {
    return leftBump;
  }

  @Override
  public Trigger rightBumper() {
    return rightBump;
  }

  @Override
  public double getLeftY() {
    return this.getRawAxis(OI.AxisID.LeftY.value);
  }

  @Override
  public double getLeftX() {
    return this.getRawAxis(OI.AxisID.LeftX.value);
  }

  @Override
  public double getRightX() {
    return this.getRawAxis(OI.AxisID.RightX.value);
  }

  @Override
  public double getRightY() {
    return this.getRawAxis(OI.AxisID.RightY.value);
  }

  @Override
  public Trigger leftTrigger() {
    return new Trigger(
        () -> this.getRawAxis(OI.AxisID.LeftTrigger.value) > this.threshold);
  }

  @Override
  public Trigger rightTrigger() {
    return new Trigger(
        () -> this.getRawAxis(OI.AxisID.RightTrigger.value) > this.threshold);
  }

  @Override
  public Trigger leftTrigger(double t) {
    return new Trigger(() -> this.getRawAxis(OI.AxisID.LeftTrigger.value) > t);
  }

  @Override
  public Trigger rightTrigger(double t) {
    return new Trigger(() -> this.getRawAxis(OI.AxisID.RightTrigger.value) > t);
  }

  @Override
  public Trigger start() {
    return start;
  }

  @Override
  public Trigger back() {
    return back;
  }

  @Override
  public Trigger leftStick() {
    return leftStick;
  }

  @Override
  public Trigger rightStick() {
    return rightStick;
  }

  public Trigger povUp() {
    return this.pov(0);
  }

  public Trigger povUpRight() {
    return pov(45);
  }

  public Trigger povRight() {
    return pov(90);
  }

  public Trigger povDownRight() {
    return pov(135);
  }

  public Trigger povDown() {
    return pov(180);
  }

  public Trigger povDownLeft() {
    return pov(225);
  }

  public Trigger povLeft() {
    return pov(270);
  }

  public Trigger povUpLeft() {
    return pov(315);
  }

  public Trigger povCenter() {
    return pov(-1);
  }

  public Trigger pov(int angle) {
    return pov1(0, angle);
  }

  public int getPOV(int pov) {
    return DriverStation.getStickPOV(this.port, pov);
  }

  public Trigger pov1(int pov, int angle) {
    // SmartDashboard.putBoolean("", false)
    return new Trigger(
        new BooleanSupplier() {

          @Override
          public boolean getAsBoolean() {
            // TODO Auto-generated method stub
            return getPOV(pov) == angle;
            // throw new UnsupportedOperationException("Unimplemented method 'getAsBoolean'");
          }
        });
  }
}

