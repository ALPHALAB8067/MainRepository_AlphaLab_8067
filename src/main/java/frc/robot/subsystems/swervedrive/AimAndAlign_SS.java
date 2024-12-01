// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AimAndAlign_SS extends SubsystemBase {
  /** Creates a new AimAndAlign_SS. */

  boolean targetVisible = false; 
  double targetYaw = 0.0;
  PhotonCamera camera = new PhotonCamera("photo2");
  double iD;
   double kP = 0.0075;
  double kI = 0.0;
  double kD = 0.0;
double posn;


  PIDController mPidController1 = new PIDController(kP, kI, kD);

  public AimAndAlign_SS() {
    

  
  }

  public double AimAtApril(double turnKP, double maxturnspeed){
     var results = camera.getLatestResult();


     if (results.hasTargets()){
      var target = results.getBestTarget();
        Transform3d bestcameratotarget = target.getBestCameraToTarget();
      targetYaw = bestcameratotarget.getZ();
      targetVisible = true;
      iD = target.getFiducialId();
      }  
    //double turn = 1.0 * turnKP * maxturnspeed;
    double turn = mPidController1.calculate(targetYaw, 180);
    if(targetYaw > 0){
      posn = -1;
    }else if (targetYaw < 0){
       posn = 1;

    }
    return -turn * posn;
  }

  public double WhichPosition(){
 var results = camera.getLatestResult();

    if (results.hasTargets()){
      var target = results.getBestTarget();
      targetYaw = target.getYaw();
      targetVisible = true;
      iD = target.getFiducialId();
      }  
    return iD;
    }
      

  @Override 
  public void periodic() {
    SmartDashboard.putNumber("Position", WhichPosition());
    SmartDashboard.putBoolean("Icanseeyou", camera.getLatestResult().hasTargets());
    // This method will be called once per scheduler run
  }

  public double ForwardAim(int i, double d) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'ForwardAim'");
  }
}
