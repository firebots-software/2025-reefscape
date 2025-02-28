package frc.robot.util;

import dev.doglog.DogLog;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import java.util.LinkedList;
import java.util.Queue;

public class WindowAverage {
  private Queue<Double> averageWindow;

  private final int MAX_WINDOW_SIZE = 20;
  private final double FIRST_VALUE =
      ElevatorPositions.Intake.height / Constants.ElevatorConstants.CARRAIGE_UPDUCTION;
  private boolean replacedFirstValue = false;
//HIGH:0.03
//LOW: 0.01
//In Code: 
//
  public WindowAverage() {
    averageWindow = new LinkedList<Double>();
    averageWindow.add(FIRST_VALUE);
  }

  public double getAverage() {
    if (averageWindow.size() < MAX_WINDOW_SIZE) {
      DogLog.log("mattcs/averaging", true);
      return averageWindow.peek();
    }

    double sum = 0.0;
    DogLog.log("mattcs/averaging", true);
    for (double value : averageWindow) {
      sum += value;
    }

    return sum / averageWindow.size();
  }

  public void addValue(double value) {
    if (averageWindow.size() == MAX_WINDOW_SIZE) {
      DogLog.log("mattcs/max_size_reached", true);
      return;
    }
    ;
    DogLog.log("mattcs/max_size_reached", false);
    DogLog.log("mattcs/size", averageWindow.size());

    if (!replacedFirstValue) {
      averageWindow.poll();
      replacedFirstValue = true;
    }
    
    if ((value <= 0.03) && (value >= 0.02)) {
      averageWindow.add(value);
    }
    else {
      if (averageWindow.size() == 0) {
        averageWindow.add(FIRST_VALUE);
        replacedFirstValue = false;
      }
    }
    
  }

  public void clearWindow() {
    if (averageWindow.size() == 1) {
      DogLog.log("mattcs/skipping_clear", true);
      return;
    }
    DogLog.log("mattcs/skipping_clear", false);

    averageWindow.clear();
    averageWindow.add(FIRST_VALUE);
    replacedFirstValue = false;
  }
}
