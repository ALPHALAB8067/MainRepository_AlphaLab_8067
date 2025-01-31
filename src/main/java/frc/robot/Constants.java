// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = 4.9;
      // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(1.2, 0.0001, 0);
    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.5, 0.0, 0.05);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  public static final class PositionVelocities
  {
    //1
    public static  final double RightVelocity1 = 3000; 
    public static final double LeftVelocity1 = RightVelocity1 - 600;

    //2 

    public static final double RightVelocity2 = 5000; 
    public static final double LeftVelocity2 = RightVelocity2 - 600;

    //3

    public static final double RightVelocity3 = 5000; 
    public static final double LeftVelocity3 = RightVelocity3 - 600;

    //4

    public static final double RightVelocity4 = 5000; 
    public static final double LeftVelocity4 = RightVelocity4 - 600;

    //5

    public static final double RightVelocity5 = 5000; 
    public static final double LeftVelocity5 = RightVelocity5 - 600;

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class PositionAngles
  {
    //1
    public static  final double Angle1 = 26; 

    //2 

    public static final double Angle2 = 26; 

    //3

    public static final double Angle3 = 25;

    //4

    public static final double Angle4 = 26; 
    //5 
    
    public static final double Angle5 = 26;
}
}