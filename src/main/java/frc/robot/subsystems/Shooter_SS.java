// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter_SS extends SubsystemBase {

  private final CANSparkMax mLeftMotorMaster;
  private final CANSparkMax mLeftMotorSlave;
  private final CANSparkMax mRightMotorMaster;
  private final CANSparkMax mRightMotorSlave;


  /** Creates a new Shooter_SS. */
  public Shooter_SS() {
     
    mLeftMotorMaster = new CANSparkMax(19, MotorType.kBrushless);
    mLeftMotorMaster.setIdleMode(IdleMode.kCoast);
    mLeftMotorSlave = new CANSparkMax(20, MotorType.kBrushless);
    mLeftMotorSlave.setIdleMode(IdleMode.kCoast);
    mRightMotorMaster = new CANSparkMax(21, MotorType.kBrushless);
    mRightMotorMaster.setIdleMode(IdleMode.kCoast);
    mRightMotorSlave = new CANSparkMax(22, MotorType.kBrushless);
    mRightMotorSlave.setIdleMode(IdleMode.kCoast);

    mLeftMotorSlave.follow(mLeftMotorMaster);
    mRightMotorSlave.follow(mRightMotorMaster);

    mLeftMotorMaster.setInverted(true);
    mRightMotorMaster.setInverted(true);
  }

  public void shoot() {
    mLeftMotorMaster.set(0.2);
    mRightMotorMaster.set(0.2);
  }

  public void suck() {
    mLeftMotorMaster.set(-0.05);
    mRightMotorMaster.set(-0.05);
  }

  public void stop() {
    mLeftMotorMaster.stopMotor();
    mRightMotorMaster.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
