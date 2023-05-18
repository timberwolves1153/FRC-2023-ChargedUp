package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LimelightHelpers;

public class Limelight extends SubsystemBase{
    
    private NetworkTable table;
    private boolean tv;
    private double tx;
    private double ty;
    private double ta;

    private double x;
    private double y;
    private double a;

    public Limelight() {

        table = NetworkTableInstance.getDefault().getTable("limelight");
        tv = LimelightHelpers.getTV("limelight");
        tx = LimelightHelpers.getTX("limelight");
        ty = LimelightHelpers.getTY("limelight");
        ta = LimelightHelpers.getTA("limelight");
                

        LimelightHelpers.setPipelineIndex("limelight", 1); 
        
        
    }

    public double getTX() {
        return LimelightHelpers.getTX("limelight");
        
    }

    public double getTY() {
        return LimelightHelpers.getTY("limelight");
        
    }

    public double getTA() {
        return LimelightHelpers.getTA("limelight");
        
    }

    @Override
    public void periodic() {
        x = LimelightHelpers.getTX("limelight");
        y = LimelightHelpers.getTY("limelight");
        a = LimelightHelpers.getTA("limelight");
                
        SmartDashboard.putNumber("tx", x);
        SmartDashboard.putNumber("ty", y);
        SmartDashboard.putNumber("ta", a);
    }
}
