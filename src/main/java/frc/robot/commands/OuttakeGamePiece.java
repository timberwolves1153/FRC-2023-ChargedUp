package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Collector;

public class OuttakeGamePiece extends CommandBase {

    private Collector collector;

    public OuttakeGamePiece(Collector collector) {
        this.collector = collector;
        addRequirements(collector);
    }

      /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        GamePiece currPiece = collector.getCurrentGamePiece();
        if (GamePiece.CONE.equals(currPiece)) {
            collector.coneOuttake();
        } else if (GamePiece.CUBE.equals(currPiece)) {
            collector.cubeOuttake();
        } else {
            //something has gone wrong, just dont run the collector
            collector.collectorStop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        collector.collectorStop();
    }
}
