//package frc.robot.commands.CommandGroups.java;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

public class TestAuto {

    private Swerve swerve;
    private Constants constants;
    //private SwerveDriveOdometry swerveOdometry;
    //private PathPlannerTrajectory traj;
    //private SwerveDriveKinematics swerveDriveKinematics;


    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
             new InstantCommand(() -> {
               // Reset odometry for the first path you run during auto
               if(isFirstPath){
                   swerve.resetOdometry(traj.getInitialHolonomicPose());
               }
             }),
             new PPSwerveControllerCommand(
                 traj, 
                 this::getPose, // Pose supplier
                // Constants.Swerve.swerveKinematics, // SwerveDriveinematics
                 new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                 new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 this.swerve::setModuleStates(null), // Module states consumer
                 true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                 this.swerve // Requires this drive subsystem
             )
         );
     }
}

