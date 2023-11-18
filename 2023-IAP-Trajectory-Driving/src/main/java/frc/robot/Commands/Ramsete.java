package frc.robot.Commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Ramsete extends SequentialCommandGroup { //Creates a new ramsete
    String trajectoryJSON = "pathplanner/generatedJSON/New New New Path.wpilib.json"; //Makes variable trajectoryJSON, which is composed of letters, string.
    Trajectory trajectory = new Trajectory(); //Creates the object trajectory in the Trajectory class
    
    public Ramsete() {
        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
            Constants.RamseteConstants.kS, //puts inside variable SimpleMotorFeedforwards the number in kS from RamseteConstants
            Constants.RamseteConstants.kV, //puts inside variable SimpleMotorFeedforwards the number in kV from RamseteConstants
            Constants.RamseteConstants.kA), //puts inside variable SimpleMotorFeedforwards the number in kA from RamseteConstants
        Constants.RamseteConstants.kDriveKinematics, //Puts inside variable SimpleMotorforwards the number in kDriveKinematics from RamseteConstants
        11); //Has the max voltage used be 11
        TrajectoryConfig config = new TrajectoryConfig(
            Constants.RamseteConstants.kMaxSpeed, //Adds what the max speed is, which is the value of the constant kMaxSpeed
            Constants.RamseteConstants.kMaxAcceleration) //Applys what the max acceleration can be, which is the value of the constant kMaxAcceleration
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.RamseteConstants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint); //adds the constraints to the autoVoltageConstraint object 
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON); //Gets the trajectory path of trajectoryJSON and puts it into trajectoryPath 
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath); //Has trajectory read the code version of the trajectory path
             } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace()); //
            }
        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            RobotContainer.dt::getPose,
            new RamseteController(Constants.RamseteConstants.kRamseteB, Constants.RamseteConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.RamseteConstants.kS,
                Constants.RamseteConstants.kV,
                Constants.RamseteConstants.kA),
            Constants.RamseteConstants.kDriveKinematics,
            RobotContainer.dt::getWheelSpeeds,
            new PIDController(Constants.RamseteConstants.kPVel, 0, 0),
            new PIDController(Constants.RamseteConstants.kPVel, 0, 0),
            // RamseteCommand passes volts to the callback
            RobotContainer.dt::tankDrive,
            RobotContainer.dt);

        RobotContainer.dt.getField2d().getObject("traj").setTrajectory(trajectory);

        CommandBase ramc = ramseteCommand.handleInterrupt(() -> RobotContainer.dt.tankDrive(0.0, 0.0))
            .andThen(() -> RobotContainer.dt.tankDrive(0.0, 0.0));
        
        addCommands(ramc);
        }

    }

