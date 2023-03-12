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
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.PIDExtender;
import frc.robot.subsystems.PIDPivot;
import frc.robot.subsystems.Swerve;

public class CenterScore1AndBalance extends PPAutoBase{

    public CenterScore1AndBalance(Swerve swerve, Collector collector, PIDExtender pidExtender, PIDPivot pidPivot) {
        super(swerve);
        PathPlannerTrajectory Score1andBalanceCenter = PathPlanner.loadPath("Score1andBalanceCenter", new PathConstraints(1.2, 1.5));
        PathPlannerTrajectory jiggle = PathPlanner.loadPath("JigglePath", new PathConstraints(1.2, 1.5));  
        //TODO Auto-generated constructor stub

        HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("Outtake", new CubeIntake(collector));
        // eventMap.put("HighNodePivot", Commands.runOnce(() -> pidPivot.setSetpointDegrees(20.5)));
        // eventMap.put("HighNodeExtender", Commands.runOnce(() -> pidExtender.setSetpointInches(20)));


        FollowPathWithEvents command = new FollowPathWithEvents(followTrajectoryCommand(Score1andBalanceCenter, true), Score1andBalanceCenter.getMarkers(), eventMap);
        
        addCommands(
        Commands.runOnce(() -> pidExtender.setSetpointInches(2), pidExtender),
        new WaitCommand(0.2),
        Commands.runOnce(() -> pidExtender.setSetpointInches(-3), pidExtender),
        Commands.runOnce(() -> pidPivot.setSetpointDegrees(23), pidPivot),
        new InstantCommand(() -> collector.slowConeIntake()),
        new WaitCommand(1),
        Commands.runOnce(() -> pidExtender.setSetpointInches(25), pidExtender),
        new WaitCommand(1.55),
        new CubeIntake(collector).withTimeout(0.25),
        Commands.runOnce(() -> pidExtender.setSetpointInches(-4), pidExtender),
        new WaitCommand(1.55),
        Commands.runOnce(() -> pidPivot.setSetpointDegrees(-40), pidPivot),
        command, 
        new AutoBalanceWithRoll(swerve, () -> true),
        new InstantCommand(() -> swerve.xPosition(true)));
    }
    
}
