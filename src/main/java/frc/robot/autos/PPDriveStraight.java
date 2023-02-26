package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.commands.AutoBalanceWithRoll;
import frc.robot.commands.QuickAutoBalance;
import frc.robot.subsystems.Swerve;

public class PPDriveStraight extends PPAutoBase{
    

    public PPDriveStraight(Swerve swerve) {
        super(swerve);
        PathPlannerTrajectory test1 = PathPlanner.loadPath("Test1", new PathConstraints(2, 1.5));
        PathPlannerTrajectory balancePath1 = PathPlanner.loadPath("BalancePath", new PathConstraints(1.5, 1));
        PathPlannerTrajectory path1 = PathPlanner.loadPath("DriveStraighPath", new PathConstraints(3.0, 2.0));

        addCommands(
            followTrajectoryCommand(test1, true),
            new AutoBalanceWithRoll(swerve, () -> true)
        );
    }
}
