package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleFaults;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SharedState;

public class LEDSubsystem extends SubsystemBase {
    
    private CANdle candle = new CANdle(0);
    // private int LEDCycles = 0;
    // private boolean LEDState = false;

    public LEDSubsystem(){
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 0.7;
        candle.configAllSettings(config);
        candle.clearAnimation(0);
        candle.setLEDs(0, 0, 0);
        setAllSwerve(255, 255, 255); // set the CANdle LEDs to white
        SingleFadeAnimation animation = new SingleFadeAnimation(255, 255, 255, 0, 0.5, 103, 8);
        candle.animate(animation);

    }    
    
    @Override
    public void periodic() {
        if(SharedState.get().getCoralInWay()){
            candle.setLEDs(255, 0, 0, 0, 0, 8);
        } else {
            candle.setLEDs(0, 255, 0, 0, 0, 8);
        }

    }

    private void setSwerve1(int r, int g, int b){
        candle.setLEDs(r, g, b, 0, 8, 25);
    }

    private void setSwerve2(int r, int g, int b){
        candle.setLEDs(r, g, b, 0, 8+25, 26);
    }

    private void setSwerve3(int r, int g, int b){
        candle.setLEDs(r, g, b, 0, 8+25+26, 26);
    }

    private void setSwerve4(int r, int g, int b){
        candle.setLEDs(r, g, b, 0, 8+25+26+26, 26);
    }

    private void setAllSwerve(int r, int g, int b){
        candle.setLEDs(r, g, b, 0, 8, 103);
    }
}