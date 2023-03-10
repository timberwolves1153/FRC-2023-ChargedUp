package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GamePiece;

public class Collector extends SubsystemBase{
    private WPI_TalonSRX collector;
    private GamePiece currectGamePiece;
    private Solenoid hinge;
 

    public Collector(){
        
        collector = new WPI_TalonSRX(59);
        hinge = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        config();

    }

    public void config(){
        collector.configFactoryDefault();
        collector.setNeutralMode(NeutralMode.Coast);
        hinge.set(true);
    }
    public void deployHinge() {
        hinge.set(false);
    }

    public void toggleHinge() {
        hinge.toggle();
    }

    public void coneIntake(){
        collector.setVoltage(-12.0);
    }

    public void cubeIntake(){
        collector.setVoltage(12.0);
    }

    public void coneOuttake(){
        collector.setVoltage(12.0);
    }

    public void cubeOuttake(){
        collector.setVoltage(-12.0);
    }

    public void slowConeIntake() {
        collector.setVoltage(-1.5);
    }

    public void slowCubeIntake() {
        collector.setVoltage(1.5);
    }

    public void collectorStop(){
        collector.setVoltage(0.0);
    }

    public GamePiece getCurrentGamePiece() {
        return this.currectGamePiece;
    }

    public void setCurrentGamePiece(GamePiece gamePiece) {
        this.currectGamePiece = gamePiece;
    }
}
