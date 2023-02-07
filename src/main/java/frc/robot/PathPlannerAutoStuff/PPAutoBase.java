package frc.robot.PathPlannerAutoStuff;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
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

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public PPSwerveControllerCommand baseSwerveCommand(PathPlannerTrajectory trajectory) {
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(
            trajectory, 
            swerve::getPose, 
            Constants.Swerve.swerveKinematics, 
            new PIDController(Constants.AutoConstants.kPXController, 0, 0), 
            new PIDController(Constants.AutoConstants.kPYController, 0, 0), 
            thetaController, 
            swerve::setModuleStates, 
            swerve);
        return command;
}
}
