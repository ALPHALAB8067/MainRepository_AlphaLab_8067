// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Scoop_SS extends SubsystemBase {
  /** Creates a new Scoop_SS. */
  private final TalonSRX mScoopMotor;
  public Scoop_SS() {
    mScoopMotor = new TalonSRX(21);
  }

  public void Scoop_up(){
    mScoopMotor.set(ControlMode.PercentOutput, 0.3);
  }

  public void Scoop_down(){
    mScoopMotor.set(ControlMode.PercentOutput, -0.3);
  }
  public double GetScoopEncoderValue (){
    return mScoopMotor.getSelectedSensorPosition();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("scoop position",GetScoopEncoderValue());
  }
}
