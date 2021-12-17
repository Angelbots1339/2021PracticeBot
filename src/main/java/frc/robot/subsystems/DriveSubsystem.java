// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import java.util.function.DoubleSupplier;


/** Add your docs here. */
public class DriveSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax leftMotorTop = new CANSparkMax(DriveConstants.leftMotorTopPort, MotorType.kBrushless);
  private CANSparkMax leftMotorFront = new CANSparkMax(DriveConstants.leftMotorFrontPort, MotorType.kBrushless);
  private CANSparkMax leftMotorBack = new CANSparkMax(DriveConstants.leftMotorBackPort, MotorType.kBrushless);
  private CANSparkMax rightMotorTop = new CANSparkMax(DriveConstants.rightMotorTopPort, MotorType.kBrushless);
  private CANSparkMax rightMotorFront = new CANSparkMax(DriveConstants.rightMotorFrontPort, MotorType.kBrushless);
  private CANSparkMax rightMotorBack = new CANSparkMax(DriveConstants.rightMotorBackPort, MotorType.kBrushless);

  private SpeedControllerGroup leftMotorControllerGroup = new SpeedControllerGroup(
      new SpeedController[] { leftMotorTop, leftMotorFront, leftMotorBack });
  private SpeedControllerGroup rightMotorControllerGroup = new SpeedControllerGroup(
      new SpeedController[] { rightMotorTop, rightMotorFront, rightMotorBack });

  private DifferentialDrive m_Drive = new DifferentialDrive(leftMotorControllerGroup, rightMotorControllerGroup);

  private PIDController leftPID = new PIDController(DriveConstants.leftKP, 0, 0);
  private PIDController rightPID = new PIDController(DriveConstants.rightKP, 0, 0);

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka);

  private Pose2d pose = new Pose2d();

  private DifferentialDriveKinematics m_DriveKinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);
                                                                                                                     
  private DifferentialDriveOdometry m_DriveOdometry = new DifferentialDriveOdometry(new Rotation2d(), pose);

  private AnalogGyro gyro = new AnalogGyro(0);// TODO add Constants

  public DriveSubsystem() {
      
  }
  

  /**
   * Drives the robot using arcade controls. Intend use for inline command reason
   * for Suppliers
   * 
   * @param fwd supplier for forward movement
   * @param rot supplier for rotation rotation
   */
  public void arcadeDrive(DoubleSupplier fwd, DoubleSupplier rot) {
    m_Drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble());
  }

  /**
   * Drive the robot through tank drive using volts
   * 
   * @param leftVolts  left motor speeds in volts
   * @param rightVolts right motor speeds in volts
   */
  public void tankDriveVolts(Double leftVolts, Double rightVolts) {
    leftMotorControllerGroup.setVoltage(leftVolts);
    rightMotorControllerGroup.setVoltage(rightVolts);
  }

  /**
   * @return current heading from gyro in a {@link Rotation2d} 
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  /**
   * @return total distance right encoder has traveled in meters
   */

  public double getDistanceRight() {
    // todo add clicks per rot and wheel r and gear ratio
    return rightMotorTop.getEncoder().getPosition() * DriveConstants.wheelRotPerMotorRot * DriveConstants.wheelDiameter
        * Math.PI;
  }

  /**
   * 
   * @return total distance left encoder has traveled in meters
   */
  public double getDistanceLeft() {
    // todo add clicks per rot and wheel r and gear ratio
    return leftMotorTop.getEncoder().getPosition() * DriveConstants.wheelRotPerMotorRot * DriveConstants.wheelDiameter
        * Math.PI;

  }
  /**
   * 
   * @return Wheel speeds in type {@link DifferentialDriveWheelSpeeds} from encoders in meters per sec
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftMotorTop.getEncoder().getVelocity() * DriveConstants.wheelRotPerMotorRot * DriveConstants.wheelDiameter
            * Math.PI / 60,
        rightMotorTop.getEncoder().getVelocity() * DriveConstants.wheelRotPerMotorRot * DriveConstants.wheelDiameter
            * Math.PI / 60);
  }

 @Override
  public void periodic() {
    pose = m_DriveOdometry.update(getHeading(), getDistanceRight(), getDistanceLeft());
  }

  //Getters
  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPid() {
    return leftPID;
  }

  public PIDController getRightPid() {
    return rightPID;
  }

  public DifferentialDriveKinematics getKinematics() {
    return m_DriveKinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  

}
