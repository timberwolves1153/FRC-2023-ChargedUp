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
import frc.robot.commands.AutoBalanceWithRoll;
import frc.robot.subsystems.Swerve;

public class AutoBalanceAuto extends AutoBase {
    
    //Swerve s_Swerve;
    private final String BALANCE_AUTO_PATH1 = "pathplanner/generatedJSON/ScoreAndBalancePath1.wpilib.json";
    private final String BALANCE_AUTO_PATH2 = "pathplanner/generatedJSON/ScoreAndBalancePath2.wpilib.json";
    Trajectory trajectory;
    Trajectory trajectory2;

    SwerveControllerCommand TrajectoryPath1 = WPIbaseSwerveCommand(trajectory);
    SwerveControllerCommand TrajectoryPath2 = WPIbaseSwerveCommand(trajectory2);
    

    public AutoBalanceAuto(Swerve swerve) {
        super(swerve);
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        Path path = Filesystem.getDeployDirectory().toPath().resolve(BALANCE_AUTO_PATH1);
        Path path2 = Filesystem.getDeployDirectory().toPath().resolve(BALANCE_AUTO_PATH2);
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(path);
            trajectory2 = TrajectoryUtil.fromPathweaverJson(path2);
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
            WPIbaseSwerveCommand(trajectory),
            WPIbaseSwerveCommand(trajectory2),
            new AutoBalanceWithRoll(swerve, () -> true));
    }
    
}
