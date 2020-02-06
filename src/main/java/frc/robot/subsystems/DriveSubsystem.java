/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // CANSparkMax leftMotor1 = new CANSparkMax(RobotMap.LEFT_MOTOR_1, CANSparkMaxLowLevel.MotorType.kBrushless);
  // CANSparkMax leftMotor2 = new CANSparkMax(RobotMap.LEFT_MOTOR_2, CANSparkMaxLowLevel.MotorType.kBrushless);
  // CANSparkMax rightMotor1 = new CANSparkMax(RobotMap.RIGHT_MOTOR_1, CANSparkMaxLowLevel.MotorType.kBrushless);
  // CANSparkMax rightMotor2 = new CANSparkMax(RobotMap.RIGHT_MOTOR_2, CANSparkMaxLowLevel.MotorType.kBrushless);
  // public SpeedControllerGroup left = new SpeedControllerGroup(leftMotor1, leftMotor2);
  // SpeedControllerGroup right = new SpeedControllerGroup(rightMotor1, rightMotor2);  
  // public DifferentialDrive drive = new DifferentialDrive(left, right);
  
  @Override
  protected void initDefaultCommand() {

    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveCommand());
  }
}
