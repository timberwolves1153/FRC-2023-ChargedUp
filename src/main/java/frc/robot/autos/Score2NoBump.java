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
import frc.robot.commands.CubeIntake;
import frc.robot.commands.ScoreCube;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.PIDExtender;
import frc.robot.subsystems.PIDPivot;
import frc.robot.subsystems.Swerve;

public class Score2NoBump extends PPAutoBase{

    public Score2NoBump(Swerve swerve, Collector collector, PIDExtender pidExtender, PIDPivot pidPivot) {
        super(swerve);

        PathPlannerTrajectory Score2Left = PathPlanner.loadPath("Score2WIthNoBump", new PathConstraints(2.5, 2.2));
        //TO DO Auto-generated constructor stub

        HashMap<String, Command> eventMap = new HashMap<>();
    
        // eventMap.put("ExtendIn1", Commands.runOnce(() -> pidExtender.setSetpointInches(0), pidExtender));
         eventMap.put("ReadyCubePositionPivot", Commands.runOnce(() -> pidPivot.setSetpointDegrees(-35)));
         eventMap.put("ReadyCubePositionExtender", Commands.runOnce(() -> pidExtender.setSetpointInches(-1)));
         eventMap.put("ExtendTo0", Commands.runOnce(() -> pidExtender.setSetpointInches(-2), pidExtender));
         eventMap.put("CubeIntake", new InstantCommand(() -> collector.cubeIntake()));
         eventMap.put("GroundCubeExtend", Commands.runOnce(() -> pidExtender.setSetpointInches(17)));
         eventMap.put("CubeSlowIntake", new InstantCommand(() -> collector.cubeIntake()));
         eventMap.put("HighNodePivot2", Commands.runOnce(() -> pidPivot.setSetpointDegrees(26)));
         eventMap.put("HighNodeExtender2", Commands.runOnce(() -> pidExtender.setSetpointInches(27)));
         eventMap.put("scoreCube", new InstantCommand(() -> collector.coneIntake()));
         eventMap.put("RetractExtender", Commands.runOnce(() -> pidExtender.setSetpointInches(0), pidExtender));



        FollowPathWithEvents command = new FollowPathWithEvents(followTrajectoryCommand(Score2Left, true), Score2Left.getMarkers(), eventMap);

        addCommands(
        Commands.runOnce(() -> pidPivot.setSetpointDegrees(31), pidPivot),
        new InstantCommand(() -> collector.slowConeIntake()),
        new WaitCommand(1),
        Commands.runOnce(() -> pidExtender.setSetpointInches(29), pidExtender),
        new WaitCommand(1.55),
        new CubeIntake(collector).withTimeout(0.5),
        Commands.runOnce(() -> pidExtender.setSetpointInches(-4), pidExtender),
        new WaitCommand(1),
        //Commands.runOnce(() -> pidPivot.setSetpointDegrees(-40), pidPivot),
        command,
        Commands.runOnce(() -> pidExtender.setSetpointInches(0), pidExtender)
        //new ScoreCube(collector).withTimeout(0.75),
       // Commands.runOnce(() -> pidExtender.setSetpointInches(1), pidExtender));
        );
    }
    
}
