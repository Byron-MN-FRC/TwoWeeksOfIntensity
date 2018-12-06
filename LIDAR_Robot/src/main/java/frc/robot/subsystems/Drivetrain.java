/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 *
 */
public class Drivetrain extends Subsystem {
	// Put methods for controlling this subsystem
    // here. Call these from Commands.
	public static WPI_TalonSRX motorLeftMaster = new WPI_TalonSRX(RobotMap.talonIDLeftMaster);
	public static WPI_TalonSRX motorLeftFollower = new WPI_TalonSRX(RobotMap.talonIDLeftFollower);
	
	public static WPI_TalonSRX motorRightMaster = new WPI_TalonSRX(RobotMap.talonIDRightMaster);
  public static WPI_TalonSRX motorRightFollower = new WPI_TalonSRX(RobotMap.talonIDRightFollower);
  
  // public static SpeedControllerGroup drivetrainLeft = new SpeedControllerGroup(motorLeftMaster,motorLeftFollower);
  // public static SpeedControllerGroup drivetrainRight = new SpeedControllerGroup(motorRightMaster,motorRightFollower);
  
  // private double joystickY;
  // private double joystickTwist;


	public void initDefaultCommand () {

	}

	// public void driveWithJoystick(Joystick joyStick) {
  //   // store the current Y and Twist values in local fields for updateStatus
  //   joystickY = joyStick.getY();
  //   joystickTwist = joyStick.getTwist();

  //   Drivetrain.arcadeDrive(joystickY, joystickTwist);

  // }
  
  public void driveForward(double speed){
    if(RobotMap.inRange){
      motorLeftFollower.set(0);
      motorLeftMaster.set(0);
      motorRightMaster.set(0);
      motorRightFollower.set(0);
    } else{
    motorLeftFollower.set(speed);
    motorLeftMaster.set(speed);
    motorRightMaster.set(-speed);
    motorRightFollower.set(-speed);
    }
  }

//	public void updateStatus() {
//
//	}
	
//	public void stop() {
//    Drivetrain.arcadeDrive(0,0);
//	}
}