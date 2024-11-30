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

  double x;
  double d = 0.0;
  double v = 0.0;
  double Puissance;


  public DistanceFromAprilTag() {

    
  }

  public double metersfromapriltagx(){
    var results = camera.getLatestResult();

    if (results.hasTargets()){
      var target = results.getBestTarget();
      Transform3d bestcameratotarget = target.getBestCameraToTarget();
      x = bestcameratotarget.getX();
      targetVisible = true;
      
      }  

      return x;
    
    }

  public int iD(){ 
    var results = camera.getLatestResult();

    if (results.hasTargets()){
      var target = results.getBestTarget();
      int iD = target.getFiducialId();
      return iD;
    }  else {
      return 0;
    }


    }  


    
    
  public double distancefrombasket(){
      switch (iD()) {
          case 1:
              return metersfromapriltagx() +  6.09805508;
              // 6.10008818;
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

    double S = -65;
      double H = 3.048;
        S = Math.toRadians(-65);



    double numerator = (Math.tan(S) * distancefrombasket()) - (2 * H);
    double denominator = -distancefrombasket();
    double a = Math.atan(numerator / denominator);
    double degree = Math.toDegrees(a);
    return 90 -(degree-3.9); 
    }    

    public double calculateRADIANS(){

    double S = -65;
      double H = 3.048;
        S = Math.toRadians(-65);



    double numerator = (Math.tan(S) * distancefrombasket()) - (2 * H);
    double denominator = -distancefrombasket();
    double a = Math.atan(numerator / denominator);
    return a; 
    }    

  public double calculatespeed(){
      double H = 3.048;

  
    double result = Math.sqrt(
      ((9.8 * Math.pow(distancefrombasket(), 2) * (1 + (Math.pow(Math.tan(Math.toRadians(calculateRADIANS())), 2)))) / ((2 * H) - (2 * distancefrombasket() * Math.tan(Math.toRadians(calculateRADIANS())))))
  );


    return result; 
    }    

  public double calculateRPMout(){
    
    double rpm = ((calculatespeed() / 1.037) * 6000) / 47.87; 
    return rpm;
   }

  public double calculateRPMmotor(){
    double conversion = 5.95;
    double beltconversion = 1.75;
    double RPMmotor = (calculateRPMout()/1.75) * 5.95;
    return RPMmotor;
}
  public double motor(){
    Puissance = calculateRPMmotor()/5676;
    return Puissance;
  }
  

  

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Icanseeyou", camera.getLatestResult().hasTargets());
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Angle yourself", calculateangle());
    SmartDashboard.putNumber("Shootitboy", Puissance);

    SmartDashboard.putNumber("HOWFARUATBRO", distancefrombasket());
    SmartDashboard.putNumber("ms", calculatespeed());
    SmartDashboard.putNumber("rpm", calculateRPMout());
        SmartDashboard.putNumber("puissance", motor());




  }
}
