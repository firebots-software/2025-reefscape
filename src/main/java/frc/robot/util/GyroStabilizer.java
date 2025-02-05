package frc.robot.util;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.subsystems.SwerveSubsystem;

public class GyroStabilizer {
    private static SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();
    private static Pigeon2 pigeon = driveTrain.getPigeon2();

    //  getRotation3d() returns a Rotation3d object, which contains the roll, pitch, and yaw of the robot
    /*  Rotation3d important methods
        double	getX():	
            Returns the counterclockwise rotation angle around the X axis (roll) in radians.
        double	getY():	
            Returns the counterclockwise rotation angle around the Y axis (pitch) in radians.
        double	getZ():	
            Returns the counterclockwise rotation angle around the Z axis (yaw) in radians.
     */
    
}
