package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoBase extends SequentialCommandGroup {
    
    //Swerve s_Swerve;
    
    public Swerve swerve;
    public Trajectory trajectory;
    public static final ProfiledPIDController thetaController =
    new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);


    public AutoBase(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

        public SwerveControllerCommand WPIbaseSwerveCommand(Trajectory trajectory) {
            //HolonomicDriveController controller = new HolonomicDriveController(new PIDController(Constants.AutoConstants.kPXController, 0, 0), new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController);
            //SwerveControllerCommand command2 = new SwerveControllerCommand(trajectory, swerve::getPose, Constants.Swerve.swerveKinematics, controller, trajectory,getstat, swerve::setModuleStates, swer)
            SwerveControllerCommand command = new SwerveControllerCommand(trajectory, swerve::getPose, Constants.Swerve.swerveKinematics, new PIDController(Constants.AutoConstants.kPXController, 0, 0), new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController, swerve::setModuleStates, swerve);
      
            
            new SwerveControllerCommand(
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

