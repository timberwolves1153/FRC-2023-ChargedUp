package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ConeIntake;
import frc.robot.commands.CubeIntake;
import frc.robot.commands.ScoreCube;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.PIDExtender;
import frc.robot.subsystems.PIDPivot;
import frc.robot.subsystems.Swerve;

public class Score3NoBump extends PPAutoBase{

    public Score3NoBump(Swerve swerve, Collector collector, PIDExtender pidExtender, PIDPivot pidPivot) {
        super(swerve);

        PathPlannerTrajectory score3Part1 = PathPlanner.loadPath("Score3Part1", new PathConstraints(2.5, 2.5));
        PathPlannerTrajectory score3Part2 = PathPlanner.loadPath("Score3Part2", new PathConstraints(2.25, 2.25));
        //TO DO Auto-generated constructor stub

        HashMap<String, Command> eventMap = new HashMap<>();
    
        // eventMap.put("ExtendIn1", Commands.runOnce(() -> pidExtender.setSetpointInches(0), pidExtender));
         eventMap.put("ReadyCubePositionPivot", Commands.runOnce(() -> pidPivot.setSetpointDegrees(-45)));
         eventMap.put("ReadyCubePositionExtender", Commands.runOnce(() -> pidExtender.setSetpointInches(-1)));
         eventMap.put("ExtendTo0", Commands.runOnce(() -> pidExtender.setSetpointInches(-2), pidExtender));
         eventMap.put("CubeIntake", new InstantCommand(() -> collector.cubeIntake()));
         eventMap.put("GroundCubeExtend", Commands.runOnce(() -> pidExtender.setSetpointInches(17)));
         eventMap.put("CubeSlowIntake", new InstantCommand(() -> collector.cubeIntake()));
         eventMap.put("HighNodePivot2", Commands.runOnce(() -> pidPivot.setSetpointDegrees(-30)));
         eventMap.put("HighNodeExtender2", Commands.runOnce(() -> pidExtender.setSetpointInches(7)));
         eventMap.put("GroundCubeExtend2", Commands.runOnce(() -> pidExtender.setSetpointInches(19), pidExtender));
         eventMap.put("ReadyCubePositionExtender2", Commands.runOnce(() -> pidExtender.setSetpointInches(7), pidExtender));
         

// CHANGE THE EXTENDER SETPOINTS
// CHANGE THE EXTENDER SETPOINTS
// CHANGE THE EXTENDER SETPOINTS
// CHANGE THE EXTENDER SETPOINTS
// CHANGE THE EXTENDER SETPOINTS
// CHANGE THE EXTENDER SETPOINTS


        FollowPathWithEvents command1 = new FollowPathWithEvents(followTrajectoryCommand(score3Part1, true), score3Part1.getMarkers(), eventMap);
        FollowPathWithEvents command2 = new FollowPathWithEvents(followTrajectoryCommand(score3Part2, true), score3Part2.getMarkers(), eventMap);
        addCommands(
        new ScoreCube(collector).withTimeout(0.2),
        //Commands.runOnce(() -> pidPivot.setSetpointDegrees(-40), pidPivot),
        command1,
        new ScoreCube(collector).withTimeout(0.4),
        command2,
        new ScoreCube(collector).withTimeout(0.5)
        );

    }
    
}
