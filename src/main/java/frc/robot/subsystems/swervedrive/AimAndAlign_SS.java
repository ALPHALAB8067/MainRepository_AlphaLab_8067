// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AimAndAlign_SS extends SubsystemBase {
  /** Creates a new AimAndAlign_SS. */

  boolean targetVisible = false; 
  double targetYaw = 0.0;
  PhotonCamera camera = new PhotonCamera("photo");
  double iD = 0.0;

  public AimAndAlign_SS() {

    var results = camera.getLatestResult();

    if (results.hasTargets()){
      var target = results.getBestTarget();
      targetYaw = target.getYaw();
      targetVisible = true;
      iD = target.getFiducialId();
      }
  }

  public double AimAtApril(double turnKP, double maxturnspeed){
    double turn = -1.0 * targetYaw * turnKP * maxturnspeed;
    return turn;
  }

  public double WhichPosition(){
    return iD;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Position", WhichPosition());
    // This method will be called once per scheduler run
  }
}
