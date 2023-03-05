package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoBalanceWithRoll;
import frc.robot.commands.CubeIntake;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.PIDExtender;
import frc.robot.subsystems.PIDPivot;
import frc.robot.subsystems.Swerve;

public class Score1AndMove extends PPAutoBase{

    public Score1AndMove(Swerve swerve, Collector collector, PIDExtender pidExtender, PIDPivot pidPivot) {
        super(swerve);

        PathPlannerTrajectory Score1AndMove = PathPlanner.loadPath("Score1AndMove", new PathConstraints(2.0, 2.0));
        //TODO Auto-generated constructor stub

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Outtake", new CubeIntake(collector));
        eventMap.put("HighNodePivot", Commands.runOnce(() -> pidPivot.setSetpointDegrees(20.5)));
        eventMap.put("HighNodeExtender", Commands.runOnce(() -> pidExtender.setSetpointInches(20)));

        FollowPathWithEvents command = new FollowPathWithEvents(followTrajectoryCommand(Score1AndMove, true), Score1AndMove.getMarkers(), eventMap);

        addCommands(
        command, 
        new AutoBalanceWithRoll(swerve, () -> true));

    }
    
}