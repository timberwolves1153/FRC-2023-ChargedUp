// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class LimelightAlignLeft extends CommandBase{
    private Swerve swerve;

    private Command currCommand;

    // tune this value to change the distance to the left that the robot travels
    private double distanceOffset = 0.5588;

    //Change this value to increase the pid. Total PID constant value will be this value plus the value in Constants
    private double pidConstantIncrease = 2;
    private double distanceFromTagMeters = 0.75;
    //Change this value to the distance from the center of the Limelight camera to the center of the robot drive base
    private double distanceToCenter = 0.16;

    private final ProfiledPIDController thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController + pidConstantIncrease, 
            0, 
            0, 
            Constants.AutoConstants.kThetaControllerConstraints
        );

    public LimelightAlignLeft(Swerve swerve) {
        this.swerve = swerve;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Command generateAlignCommand() {
        if(!LimelightHelpers.getTV("limelight")) {
            currCommand = null;
            return new InstantCommand();
        }

        Pose2d currRobotPose = swerve.getPose();

        double deltaTargetX = -LimelightHelpers.getBotPose_TargetSpace("limelight")[0] + distanceToCenter + distanceOffset;
        double deltaTargetY = LimelightHelpers.getBotPose_TargetSpace("limelight")[2] + distanceFromTagMeters;
        double rotationRadians = Math.PI;

        //Pose2d finalRobotPose = currRobotPose.transformBy(new Transform2d(new Translation2d(0, -deltaTargetX), new Rotation2d(Math.PI - currRobotPose.getRotation().getRadians())));
        Pose2d finalRobotPose = horizTransform(currRobotPose, new Translation2d(deltaTargetY, deltaTargetX), new Rotation2d(Math.PI - currRobotPose.getRotation().getRadians()));

        SmartDashboard.putNumber("Initial Pose X", currRobotPose.getX());
        SmartDashboard.putNumber("Initial Pose Y", currRobotPose.getY());
        SmartDashboard.putNumber("Initial Pose Rotation", currRobotPose.getRotation().getDegrees());

        SmartDashboard.putNumber("Final Pose X", finalRobotPose.getX());
        SmartDashboard.putNumber("Final Pose Y", finalRobotPose.getY());
        SmartDashboard.putNumber("Final Pose Rotation", finalRobotPose.getRotation().getDegrees());
        
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
        
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(List.of(currRobotPose, finalRobotPose), config);

        currCommand = baseSwerveCommand(trajectory);
        return currCommand;
    }

    public Pose2d horizTransform(Pose2d initialPose, Translation2d translation, Rotation2d rotation) {
        return new Pose2d(initialPose.getTranslation().plus(translation), initialPose.getRotation().plus(rotation));
    }

    public SwerveControllerCommand baseSwerveCommand(Trajectory trajectory) {
        SwerveControllerCommand command = new SwerveControllerCommand(
            trajectory,
            swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController + pidConstantIncrease, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController + pidConstantIncrease, 0, 0),
            thetaController,
            swerve::setModuleStates,
            swerve);
        return command;
    }

    public Command getCommand() {
        return currCommand;
    }
}
