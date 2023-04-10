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
import frc.robot.commands.AutoBalanceWithRoll;
import frc.robot.commands.CubeIntake;
import frc.robot.commands.ScoreCube;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.PIDExtender;
import frc.robot.subsystems.PIDPivot;
import frc.robot.subsystems.Swerve;

public class Score2AndBalanceBump extends PPAutoBase{

    public Score2AndBalanceBump(Swerve swerve, Collector collector, PIDExtender pidExtender, PIDPivot pidPivot) {
        super(swerve);

        PathPlannerTrajectory Score2Left = PathPlanner.loadPath("Score2WithBump", new PathConstraints(2.0, 2.0));
        PathPlannerTrajectory balance = PathPlanner.loadPath("Score2AndBalanceBump", new PathConstraints(3.0, 3.0));
        //TO DO Auto-generated constructor stub

        HashMap<String, Command> eventMap = new HashMap<>();
    
        // eventMap.put("ExtendIn1", Commands.runOnce(() -> pidExtender.setSetpointInches(0), pidExtender));
         eventMap.put("ReadyCubePositionPivot", Commands.runOnce(() -> pidPivot.setSetpointDegrees(-45)));
         eventMap.put("ReadyCubePositionExtender", Commands.runOnce(() -> pidExtender.setSetpointInches(-2)));
         eventMap.put("ExtendTo0", Commands.runOnce(() -> pidExtender.setSetpointInches(-2), pidExtender));
         eventMap.put("CubeIntake", new InstantCommand(() -> collector.cubeIntake()));
         eventMap.put("GroundCubePositionExtender", Commands.runOnce(() -> pidExtender.setSetpointInches(15)));
         eventMap.put("CubeSlowIntake", new InstantCommand(() -> collector.slowCubeIntake()));
         eventMap.put("HighNodePivot2", Commands.runOnce(() -> pidPivot.setSetpointDegrees(26)));
         eventMap.put("HighNodeExtender2", Commands.runOnce(() -> pidExtender.setSetpointInches(27)));
         eventMap.put("PivotDown", Commands.runOnce(() -> pidPivot.setSetpointDegrees(-25), pidPivot));


// CHANGE THE EXTENDER SETPOINTS
// CHANGE THE EXTENDER SETPOINTS
// CHANGE THE EXTENDER SETPOINTS
// CHANGE THE EXTENDER SETPOINTS
// CHANGE THE EXTENDER SETPOINTS
// CHANGE THE EXTENDER SETPOINTS


        FollowPathWithEvents command = new FollowPathWithEvents(followTrajectoryCommand(Score2Left, true), Score2Left.getMarkers(), eventMap);
        FollowPathWithEvents command2 = new FollowPathWithEvents(followTrajectoryCommand(balance, false), balance.getMarkers(), eventMap);

        addCommands(
        Commands.runOnce(() -> pidPivot.setSetpointDegrees(26), pidPivot),
        new InstantCommand(() -> collector.slowConeIntake()),
        new WaitCommand(1),
        Commands.runOnce(() -> pidExtender.setSetpointInches(28), pidExtender),
        new WaitCommand(1.55),
        new CubeIntake(collector).withTimeout(0.5),
        Commands.runOnce(() -> pidExtender.setSetpointInches(-4), pidExtender),
        new WaitCommand(1),
        command,
        new ScoreCube(collector).withTimeout(0.5),
        // Commands.runOnce(() -> pidExtender.setSetpointInches(1), pidExtender),
        command2,
        new AutoBalanceWithRoll(swerve, () -> true));

    }
    
}
