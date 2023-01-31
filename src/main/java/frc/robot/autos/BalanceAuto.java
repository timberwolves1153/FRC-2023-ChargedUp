package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;

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

public class BalanceAuto extends SequentialCommandGroup {
    
    //Swerve s_Swerve;
    private final String BALANCE_AUTO_PATH = "pathplanner/generatedJSON/BalancePath.wpilib.json";
    Trajectory trajectory;

    public BalanceAuto(Swerve swerve) {
        
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        Path path = Filesystem.getDeployDirectory().toPath().resolve(BALANCE_AUTO_PATH);
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(path);
        } catch (IOException e) {
            trajectory = null;
            System.err.println(e.getMessage());
        }

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectory,
                swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                swerve::setModuleStates,
                swerve);

        addCommands(
            new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand);
    }
    
}
