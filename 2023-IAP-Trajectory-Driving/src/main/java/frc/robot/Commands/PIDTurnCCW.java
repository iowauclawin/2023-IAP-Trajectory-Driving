// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;


public class PIDTurnCCW extends CommandBase {
  DriveTrain dt;
  double setPointAngle;
  PIDController stinkypoopoo = new PIDController(0.3/90, 0.002, 0 );
  int motorsign;
  

  /** Creates a new PIDTurnCCW. */
  //To get the P constant, it is the motor power divided by the set point
  public PIDTurnCCW(DriveTrain dt, double setPointAngle) {
    
    this.dt = dt; //sets variable dt = to dt
    stinkypoopoo.setTolerance(5); //Tells the robot how much it can be different from the angle we want
    this.setPointAngle = setPointAngle; //sets variable setPointAngle = setPointAngle
    addRequirements (dt);
  if (setPointAngle > 0){ //tells what the robot to do if setPointAngle is negative or going counterclockwise
    motorsign = 1;
  }else { //tells what the robot to do if setPointAngle is positive or going clockwise
    motorsign = -1;
  }

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetNavx(); //resets the Navx
    dt.tankDrive(0, 0); //sets the left and right motor to speed 0, not moving
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = stinkypoopoo.calculate(dt.getAngle(), setPointAngle);
    dt.tankDrive(-output*motorsign, output*motorsign);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stinkypoopoo.atSetpoint(); //Returns whether the robot is at the setPointAngle
  }
}
