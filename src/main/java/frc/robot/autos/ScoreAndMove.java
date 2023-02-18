package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.PivotToPosition;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;

public class ScoreAndMove extends AutoBase {
    
    private final String SCORE_AND_MOVE_PATH1 = "pathplanner/generatedJSON/ScoreAndMove1.wpilib.json";
    Trajectory trajectory;
    private final String SCORE_AND_MOVE_PATH2 = "pathplanner/generatedJSON/ScoreAndMove2.wpilib.json";
    Trajectory trajectory2;

    //SwerveControllerCommand trajectoryPath1 = baseSwerveCommand(trajectory);

    public ScoreAndMove(Swerve swerve, Pivot pivot, Collector collector) {

        super(swerve);
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        Path path = Filesystem.getDeployDirectory().toPath().resolve(SCORE_AND_MOVE_PATH1);
        Path path2 = Filesystem.getDeployDirectory().toPath().resolve(SCORE_AND_MOVE_PATH2);

        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(path);
            trajectory2 = TrajectoryUtil.fromPathweaverJson(path2);
        } catch (IOException e) {
            trajectory = null;
            trajectory2 = null;
            System.err.println(e.getMessage());
        }


        addCommands(
            new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
            baseSwerveCommand(trajectory),
            new PivotToPosition(Constants.PivotSetpoints.L2, pivot).withTimeout(1),
            new WaitCommand(0.5),
            new InstantCommand(() -> collector.collectorIntake()),
            new WaitCommand(0.25),
            new PivotToPosition(Constants.PivotSetpoints.HYBRID, pivot).withTimeout(1),
            baseSwerveCommand(trajectory2),
            new InstantCommand(() -> collector.collectorStop())
        );
    }
}
