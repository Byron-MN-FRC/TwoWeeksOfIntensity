/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveForward extends Command {

        private double speed;
        
        public DriveForward(double inputSpeed) {
            requires(Robot.kDrivetrain);
            
            speed = inputSpeed;

        }
    
        protected void initialize() {
            Robot.kDrivetrain.driveForward(speed);
        }
    
        protected void execute() {
            Robot.kDrivetrain.driveForward(speed);
        }
    
        protected boolean isFinished() {
           return false;
        }
    
        protected void end() {
            Robot.kDrivetrain.driveForward(0);
        }
    
        protected void interrupted() {
            Robot.kDrivetrain.driveForward(0);
        }
    }