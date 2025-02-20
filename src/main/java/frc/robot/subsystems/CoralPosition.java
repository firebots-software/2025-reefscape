package frc.robot.subsystems;

public class CoralPosition {
  private static boolean coralInFunnel = false;
  private static boolean coralInTootsieSlide = false;

  public static void preLoadedCoral() {
    coralInTootsieSlide = true;
  }

  public static boolean isCoralInFunnel() {
    return coralInFunnel;
  }

  public static boolean isCoralInTootsieSlide() {
    return coralInTootsieSlide;
  }

  public static void setCoralInFunnel(boolean newCondition) {
    coralInFunnel = newCondition;
  }

  public static void setCoralInTootsieSlide(boolean newCondition) {
    coralInTootsieSlide = newCondition;
  }
}
