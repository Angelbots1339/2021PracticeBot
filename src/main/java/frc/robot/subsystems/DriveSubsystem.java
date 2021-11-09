// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Add your docs here. */
public class DriveSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonFX frontLeftMotor = new TalonFX(DriveConstants.FrontLeftMotorPort);
  private TalonFX backLeftMotor = new TalonFX(DriveConstants.BackLeftMotorPort);
  private TalonFX frontRightMotor = new TalonFX(DriveConstants.FrontRightMotorPort);
  private TalonFX backRightMotor = new TalonFX(DriveConstants.BackRightMotorPort);


  


  // @Override
  // public void initDefaultCommand() {
    
  //   // Set the default command for a subsystem here.
  //   // setDefaultCommand(new MySpecialCommand());
  // }
  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }
}
