// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.DaleCommands.ArmToAngleAndSpinFlywheel;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Dealgaenate extends SequentialCommandGroup {
  /** Creates a new Dealgaenate. */
  public Dealgaenate(ArmSubsystem arm, ElevatorSubsystem elevator, ElevatorPositions position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // TODO: To whomever reads this, have fun finding actual numbers;
    // Fe fi fo fum, i proclaim that ritvik's a bum
    addCommands(new SetElevatorLevel(elevator, position));
    addCommands(new ArmToAngleAndSpinFlywheel(Constants.Arm.EXTENDED_ANGLE, arm, elevator));
    // addCommands(new SetElevatorLevelInstant(elevator, ElevatorPositions.safePosition));
  }
}
