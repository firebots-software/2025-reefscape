package frc.robot.util;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import java.util.LinkedList;
import java.util.Queue;

public class WindowAverage {
  private Queue<Double> averageWindow;

  private final int MAX_WINDOW_SIZE = 50;
  private final double FIRST_VALUE =
      ElevatorPositions.Intake.height / Constants.ElevatorConstants.CARRAIGE_UPDUCTION;

  public WindowAverage() {
    averageWindow = new LinkedList<Double>();
    averageWindow.add(FIRST_VALUE);
  }

  public double getAverage() {
    if (averageWindow.size() < MAX_WINDOW_SIZE) return averageWindow.peek();

    double sum = 0.0;
    for (double value : averageWindow) {
      sum += value;
    }

    return sum / averageWindow.size();
  }

  public void addValue(double value) {
    if (averageWindow.size() == MAX_WINDOW_SIZE) return;

    averageWindow.add(value);
  }

  public void clearWindow() {
    if (averageWindow.size() == 1) return;

    averageWindow.clear();
    averageWindow.add(FIRST_VALUE);
  }
}
