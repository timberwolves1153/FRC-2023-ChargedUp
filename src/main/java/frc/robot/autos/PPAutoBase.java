package frc.robot.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class PPAutoBase extends SequentialCommandGroup {
    private Swerve swerve;
    public PIDController thetaController =
            new PIDController(
                Constants.AutoConstants.kPThetaController, 0, 0);
    public PPAutoBase(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
       // thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }
    public SequentialCommandGroup followTrajectoryCommand(PathPlannerTrajectory path1, boolean isFirstPath) {
        PIDController thetaController = new PIDController(3, 0, 0);
        PIDController xController = new PIDController(3, 0, 0);
        PIDController yController = new PIDController(3, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        return new SequentialCommandGroup(
             new InstantCommand(() -> {
               // Reset odometry for the first path you run during auto
               if(isFirstPath){
                    PathPlannerTrajectory transformed = PathPlannerTrajectory.transformTrajectoryForAlliance(path1, DriverStation.getAlliance());
                    swerve.resetOdometry(transformed.getInitialHolonomicPose());
               }
             }),
             new PPSwerveControllerCommand(
                 path1,
                 swerve::getPose, // Pose supplier
                 Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                 xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 yController, // Y controller (usually the same values as X controller)
                 thetaController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 swerve::setModuleStates,  // Module states consumer
                 true, //Automatic mirroring
                 swerve // Requires this drive subsystem
             )
             .andThen(() -> swerve.stop())
         );
     }
}