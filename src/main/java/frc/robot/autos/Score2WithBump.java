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
import frc.robot.commands.CubeIntake;
import frc.robot.commands.WaitCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.PIDExtender;
import frc.robot.subsystems.PIDPivot;
import frc.robot.subsystems.Swerve;

public class Score2WithBump extends PPAutoBase{

    public Score2WithBump(Swerve swerve, Collector collector, PIDExtender pidExtender, PIDPivot pidPivot) {
        super(swerve);

        PathPlannerTrajectory Score2Right = PathPlanner.loadPath("Score2Speedbump Copy", new PathConstraints(2.0, 2.0));
        //TODO Auto-generated constructor stub

        HashMap<String, Command> eventMap = new HashMap<>();
    
        // eventMap.put("ExtendIn1", Commands.runOnce(() -> pidExtender.setSetpointInches(0), pidExtender));
         eventMap.put("ReadyCubePositionPivot", Commands.runOnce(() -> pidPivot.setSetpointDegrees(-31)));
         eventMap.put("ReadyCubePositionExtender", Commands.runOnce(() -> pidExtender.setSetpointInches(5.5)));
         eventMap.put("ExtendTo0", Commands.runOnce(() -> pidExtender.setSetpointInches(2), pidExtender));
         eventMap.put("CubeIntake", new InstantCommand(() -> collector.cubeNormalIntake()));
         eventMap.put("GroundCubePositionPivot", Commands.runOnce(() -> pidPivot.setSetpointDegrees(-57)));
         eventMap.put("CubeSlowIntake", new InstantCommand(() -> collector.slowCubeIntake()));
         eventMap.put("HighNodePivot2", Commands.runOnce(() -> pidPivot.setSetpointDegrees(20.5)));
         eventMap.put("HighNodeExtender2", Commands.runOnce(() -> pidExtender.setSetpointInches(45)));

        // eventMap.put("CubeOuttake", new ConeIntake(collector));



        FollowPathWithEvents command = new FollowPathWithEvents(followTrajectoryCommand(Score2Right, true), Score2Right.getMarkers(), eventMap);

        addCommands(
        Commands.runOnce(() -> pidPivot.setSetpointDegrees(20.5), pidPivot),
        new InstantCommand(() -> collector.slowConeIntake()),
        new WaitCommand(1),
        Commands.runOnce(() -> pidExtender.setSetpointInches(47), pidExtender),
        new WaitCommand(2),
        new CubeIntake(collector).withTimeout(0.5),
        Commands.runOnce(() -> pidExtender.setSetpointInches(3), pidExtender),
        new WaitCommand(2),
        Commands.runOnce(() -> pidPivot.setSetpointDegrees(-40), pidPivot),
        command,
        new WaitCommand(1),
        new ConeIntake(collector).withTimeout(0.5)
        //new AutoBalanceWithRoll(swerve, () -> true));
        );

    }
    
}