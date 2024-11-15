// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DistanceFromAprilTag extends SubsystemBase {
  /** Creates a new AimAndAlign_SS. */

  boolean targetVisible = false; 
  PhotonCamera camera = new PhotonCamera("photo2");
  double iD;

  double x;
  public DistanceFromAprilTag() {

    
  }

  public double metersfromapriltagx(){
    var results = camera.getLatestResult();

    if (results.hasTargets()){
      var target = results.getBestTarget();
      Transform3d bestcameratotarget = target.getBestCameraToTarget();
      x = bestcameratotarget.getX();
      targetVisible = true;
      iD = target.getFiducialId();
      }  
      return x;
    
    }
    

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Icanseeyou", camera.getLatestResult().hasTargets());
    // This method will be called once per scheduler run

  }
}
