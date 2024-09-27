// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule_SS extends SubsystemBase {

  private final CANSparkMax driveMotor;
  private final CANSparkMax steerMotor;
  private final CANcoder canhihi;
   

  public SwerveModule_SS(int driveMotorID, int steerMotorID, int canhihiID) {
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    steerMotor = new CANSparkMax(steerMotorID, MotorType.kBrushless);
    canhihi = new CANcoder(canhihiID);
  }

  public void setDesiredSpeeds(SwerveModuleState state) {
    double speedinessiroonie = state.speedMetersPerSecond;
    double wantedAngle = state.angle.getDegrees();

    driveMotor.set(speedinessiroonie);
    steerMotor.set(wantedAngle);
  }


  public SwerveModuleState getState() {
    StatusSignal<Double> statusSignal = canhihi.getAbsolutePosition();
    double angle = statusSignal.getValue();
    return new SwerveModuleState(driveMotor.get(), Rotation2d.fromDegrees(angle));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
