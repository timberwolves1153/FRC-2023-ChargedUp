package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.commands.AutoBalanceWithRoll;
import frc.robot.subsystems.Swerve;

public class TestBalance extends PPAutoBase{
    
    private Swerve swerve;
    public TestBalance(Swerve swerve) {
        super(swerve);
        addRequirements(swerve);
        PathPlannerTrajectory test1 = PathPlanner.loadPath("StraightLine", new PathConstraints(2, 2));

        addCommands(
           followTrajectoryCommand(test1, true)
        );
    }
}
