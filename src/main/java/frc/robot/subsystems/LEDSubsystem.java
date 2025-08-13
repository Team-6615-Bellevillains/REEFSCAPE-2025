package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SharedState;

@Logged
public class LEDSubsystem extends SubsystemBase {
    
    private CANdle candle = new CANdle(0);
    private Timer LEDTimer= new Timer();
    private boolean LEDState = false;

    public LEDSubsystem(){
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.7;
        candle.configAllSettings(config);
        candle.clearAnimation(0);
        candle.setLEDs(0, 0, 0);
        LEDTimer.start();
    }    
    
    @Override
    public void periodic() {
        if(SharedState.get().getCoralInWay()){
            candle.setLEDs(255, 0, 0, 0, 0, 8);
        } else {
            candle.setLEDs(0, 255, 0, 0, 0, 8);
        }

        if (LEDTimer.get() >= 0.5){
            if (LEDState){
                setAllSwerve(255, 255, 255);
            } else {
                setAllSwerve(0, 0, 0);
            }
            LEDState = !LEDState;
            LEDTimer.reset();
            LEDTimer.start();
        }
    }

    private void setAllSwerve(int r, int g, int b){
        candle.setLEDs(r, g, b, 0, 8, 103);
    }
}