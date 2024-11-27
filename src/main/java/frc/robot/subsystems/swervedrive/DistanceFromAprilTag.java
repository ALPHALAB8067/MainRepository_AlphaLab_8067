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
  int iD;

  double x;
  double H = 3.048;
  double a = 0.0;
  double d = 0.0;
  double S = -65;
  double v = 0.0;


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

    
    
  public double distancefrombasket(){
      switch (iD) {
          case 1:
              return x + 6.10008818;
          case 2:
              return x + 6.25027006;
          case 3:
              return x + 6.09805508;
          case 4:
              return x + 6.25027006;
          case 5:
              return x + 6.10008818;
          default:
              return 0.0;
      }
}

  public double calculateangle(){
  
    d = distancefrombasket();
    double numerator = Math.tan(S) * distancefrombasket() - 2 * H;
    double denominator = -d;
    a = Math.atan(numerator / denominator);
    return a; 
    }    

  public double calculatespeed(){
  
    d = distancefrombasket();
    double result = Math.sqrt(
      -((9.8 * Math.pow(d, 2) * (1 + Math.pow(Math.tan(a), 2))) / (2 * H - 2 * d * Math.tan(a)))
  );
    v = result;
    return result; 
    }    


  

  

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Icanseeyou", camera.getLatestResult().hasTargets());
    // This method will be called once per scheduler run

  }
}
