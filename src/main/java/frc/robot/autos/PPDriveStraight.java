package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutoBalanceWithRoll;
import frc.robot.commands.ConeIntake;
import frc.robot.commands.QuickAutoBalance;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.PIDPivot;
import frc.robot.subsystems.Swerve;

public class PPDriveStraight extends PPAutoBase{
    

    public PPDriveStraight(Swerve swerve, Collector collector, ConeIntake coneIntake, PIDPivot pidPivot) {
        super(swerve);
        PathPlannerTrajectory test1 = PathPlanner.loadPath("Test1", new PathConstraints(2, 1.5));
        //PathPlannerTrajectory balancePath1 = PathPlanner.loadPath("BalancePath", new PathConstraints(1.5, 1));
        //PathPlannerTrajectory path1 = PathPlanner.loadPath("DriveStraighPath", new PathConstraints(3.0, 2.0));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("CollectCone", new ConeIntake(collector));
        eventMap.put("slowConeIntake", new InstantCommand(() -> collector.slowConeIntake()));
        eventMap.put("pivot1", Commands.runOnce(() -> pidPivot.setSetpointDegrees(-40)));

        FollowPathWithEvents command = new FollowPathWithEvents(followTrajectoryCommand(test1, true), test1.getMarkers(), eventMap);

        addCommands(
            command
            // followTrajectoryCommand(test1, true),
            // new AutoBalanceWithRoll(swerve, () -> true)
        );
    }
}
