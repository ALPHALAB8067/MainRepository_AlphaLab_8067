// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlignAprilTag extends SubsystemBase {
  /** Creates a new AimAndAlign_SS. */

  boolean targetVisible = false; 
  double targetYaw = 0.0;
  PhotonCamera camera = new PhotonCamera("photo2");
  double iD;
  double targetRangeY = 0.0;
  double maxforwardspeed;
  double forward;
  double y;

  double kP = 0.9;
  double kI = 0.05;
  double kD = 0.0;

  double kP2 = 2.7;
  double kI2 = 0.075;
  double kD2 = 0.0;

  PIDController mPidController1 = new PIDController(kP, kI, kD);
  PIDController mPidController2 = new PIDController(kP2, kI2, kD2);

  public AlignAprilTag() {

  
  }
  
  public double ForwardAim(double turnKP, double maxforwardspeed){
     var results = camera.getLatestResult();

     

    if (results.hasTargets()){
      var target = results.getBestTarget();
      targetYaw = target.getYaw();
      targetVisible = true;
      iD = target.getFiducialId();
      Transform3d bestcameratotarget = target.getBestCameraToTarget();

      y = bestcameratotarget.getY();

      }  
    // forward = y * turnKP * -maxforwardspeed;
   // forward = (mPidController1.calculate(y, 0));

    if(y > 0.12 || y < -0.12){
      forward = mPidController1.calculate(y, 0);
  } else if(y < 0.12 || y > -0.12){
      forward = mPidController2.calculate(y, 0);
           System.out.println("pid2" + forward);
}

   
    System.out.println("forward:" + -forward);
    System.out.println("y" + y);
   return forward;
  }

  public double WhichPosition(){
 var results = camera.getLatestResult();

    return iD;
    }
      

  @Override 
  public void periodic() {
    SmartDashboard.putNumber("Position", WhichPosition());
    SmartDashboard.putBoolean("Icanseeyou", camera.getLatestResult().hasTargets());
    // This method will be called once per scheduler run
  }
}
