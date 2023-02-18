package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitCommand extends CommandBase {
    
    private double millis;
    private double startingTime;

    public WaitCommand(double seconds) {

        this.millis = seconds * 1000;

    }

    @Override
    public void initialize() {
        startingTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() >= millis + startingTime;
    }
}
