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

public class Score2NoBumpLimelight extends PPAutoBase{

    public Score2NoBumpLimelight(Swerve swerve, Collector collector, PIDExtender pidExtender, PIDPivot pidPivot, LimeLightAutoAlign limeLightAutoAlign, LimelightAlign limelightCenterAlign) {
        super(swerve);

        PathPlannerTrajectory path1 = PathPlanner.loadPath("LimeLightPath1", new PathConstraints(2.0, 2.0));
        PathPlannerTrajectory path2 = PathPlanner.loadPath("LimeLightPath2", new PathConstraints(2.0, 2.0));
        //TO DO Auto-generated constructor stub

        HashMap<String, Command> eventMap = new HashMap<>();
    
        // eventMap.put("ExtendIn1", Commands.runOnce(() -> pidExtender.setSetpointInches(0), pidExtender));
         eventMap.put("ReadyCubePositionPivot", Commands.runOnce(() -> pidPivot.setSetpointDegrees(-45)));
         eventMap.put("ReadyCubePositionExtender", Commands.runOnce(() -> pidExtender.setSetpointInches(-1)));
         eventMap.put("ExtendTo0", Commands.runOnce(() -> pidExtender.setSetpointInches(-2), pidExtender));
         eventMap.put("CubeIntake", new InstantCommand(() -> collector.cubeIntake()));
         eventMap.put("GroundCubeExtend", Commands.runOnce(() -> pidExtender.setSetpointInches(13)));
         eventMap.put("CubeSlowIntake", new InstantCommand(() -> collector.slowCubeIntake()));
         eventMap.put("HighNodePivot2", Commands.runOnce(() -> pidPivot.setSetpointDegrees(26)));
         eventMap.put("HighNodeExtender2", Commands.runOnce(() -> pidExtender.setSetpointInches(27)));

// CHANGE THE EXTENDER SETPOINTS
// CHANGE THE EXTENDER SETPOINTS
// CHANGE THE EXTENDER SETPOINTS
// CHANGE THE EXTENDER SETPOINTS
// CHANGE THE EXTENDER SETPOINTS
// CHANGE THE EXTENDER SETPOINTS


        FollowPathWithEvents command1 = new FollowPathWithEvents(followTrajectoryCommand(path1, true), path1.getMarkers(), eventMap);
        FollowPathWithEvents command2 = new FollowPathWithEvents(followTrajectoryCommand(path2, false), path2.getMarkers(), eventMap);
        LimeLightAutoAlign ll = new LimeLightAutoAlign(swerve);

        addCommands(
            new InstantCommand( () -> ll.generateAlignCommand()),
            new InstantCommand( () -> ll.getCommand().schedule())



        //    new InstantCommand(() -> swerve.resetOdometry(swerve.getPose())),
        // Commands.runOnce(() -> pidPivot.setSetpointDegrees(26), pidPivot),
        // new InstantCommand(() -> collector.slowConeIntake()),
        // new WaitCommand(1),
        // Commands.runOnce(() -> pidExtender.setSetpointInches(29), pidExtender),
        // new WaitCommand(1.55),
        // new CubeIntake(collector).withTimeout(0.5),
        // Commands.runOnce(() -> pidExtender.setSetpointInches(-4), pidExtender),
        // new WaitCommand(1),
        // //Commands.runOnce(() -> pidPivot.setSetpointDegrees(-40), pidPivot),
        // command1,
        
        // command2,
        // limelightCenterAlign.generateAlignCommand().withTimeout(3),
        // new ScoreCube(collector).withTimeout(0.5),
        // Commands.runOnce(() -> pidExtender.setSetpointInches(1), pidExtender)
        );

    }
    
}
