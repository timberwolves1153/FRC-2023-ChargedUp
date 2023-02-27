package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.commands.AutoBalanceWithRoll;
import frc.robot.commands.WaitCommand;
import frc.robot.subsystems.Swerve;

public class ScoreAndBalanceTest extends PPAutoBase {

    private Swerve swerve;

    public ScoreAndBalanceTest(Swerve swerve) {
        super(swerve);
        
        // PathPlannerTrajectory path1 = PathPlanner.loadPath("ScoreAndMove1", new PathConstraints(3, 2));
        // PathPlannerTrajectory path2 = PathPlanner.loadPath("TestPath2", new PathConstraints(3, 2));

        PathPlannerTrajectory path1 = PathPlanner.loadPath("ScoreAndBalanceRedLeft1", new PathConstraints(3.0, 2.0));
        PathPlannerTrajectory path2 = PathPlanner.loadPath("ScoreAndBalanceRedLeft2", new PathConstraints(3.0, 2.0));
        addCommands(
            followTrajectoryCommand(path1, true),
            new WaitCommand(0.5),
            followTrajectoryCommand(path2, false),
            new AutoBalanceWithRoll(swerve, () -> true)
        );
    }

    
}
