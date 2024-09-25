package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

//import com.kauailabs.navx.frc.AHRS; 

public class Swerve_SS extends SubsystemBase {
  /** Creates a new Swerve_SS. */

  private final SwerveModule_SS frontLeft, frontRight, backLeft, backRight;
  private final SwerveDriveKinematics kinematics;
  //private final AHRS navx;

  public Swerve_SS() {

    frontLeft = new SwerveModule_SS(11, 12, 1);
    frontRight = new SwerveModule_SS(13, 14, 2);
    backLeft = new SwerveModule_SS(15,16, 3);
    backRight = new SwerveModule_SS(17, 18, 4);

    kinematics = new SwerveDriveKinematics(
      new Translation2d(12, 12),
      new Translation2d(12, -12),
      new Translation2d(-12, 12),
      new Translation2d(-12, -12)
    );

    //navx = new AHRS();
  }

  public void drive (double fwd, double strf, double tokyodrift) {

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(
      new ChassisSpeeds(fwd, strf, tokyodrift)
    );

//MAX SPEED MAX SPEED MAX SPEED MAX SPEED MAX SPEED MAX SPEED MAX SPEED MAX SPEED MAX SPEED MAX SPEED MAX SPEED MAX SPEED MAX SPEED MAX SPEED 
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 0.5);

    frontLeft.setDesiredSpeeds(states[0]);
    frontRight.setDesiredSpeeds(states[1]);
    backLeft.setDesiredSpeeds(states[2]);
    backRight.setDesiredSpeeds(states[3]);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
