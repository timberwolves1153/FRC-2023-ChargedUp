package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoBalanceWithRoll;
import frc.robot.commands.ConeIntake;
import frc.robot.commands.CubeIntake;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.PIDExtender;
import frc.robot.subsystems.PIDPivot;
import frc.robot.subsystems.Swerve;

public class LeftScore2 extends PPAutoBase{

    public LeftScore2(Swerve swerve, Collector collector, PIDExtender pidExtender, PIDPivot pidPivot) {
        super(swerve);

        PathPlannerTrajectory Score2Left = PathPlanner.loadPath("Score2Left", new PathConstraints(2.0, 2.0));
        //TODO Auto-generated constructor stub

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("HighNodePivot1", Commands.runOnce(() -> pidPivot.setSetpointDegrees(20.5)));
        eventMap.put("HighNodeExtender1", Commands.runOnce(() -> pidExtender.setSetpointInches(20)));
        eventMap.put("ConeOuttake", new CubeIntake(collector));

        eventMap.put("ReadyCubePositionPivot", Commands.runOnce(() -> pidPivot.setSetpoint(-40)));
        eventMap.put("ReadyCubePositionExtender", Commands.runOnce(() -> pidPivot.setSetpoint(6.5)));

        eventMap.put("CubeIntake", new CubeIntake(collector));
        eventMap.put("GroundCubePositionPivot", Commands.runOnce(() -> pidPivot.setSetpoint(-50)));

        eventMap.put("HighNodePivot2", Commands.runOnce(() -> pidPivot.setSetpointDegrees(20.5)));
        eventMap.put("HighNodeExtender2", Commands.runOnce(() -> pidExtender.setSetpointInches(20)));

        eventMap.put("CubeOuttake", new ConeIntake(collector));



        FollowPathWithEvents command = new FollowPathWithEvents(followTrajectoryCommand(Score2Left, true), Score2Left.getMarkers(), eventMap);

        addCommands(
        command, 
        new AutoBalanceWithRoll(swerve, () -> true));

    }
    
}
