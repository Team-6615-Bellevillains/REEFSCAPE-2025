package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SharedState;

public class LEDSubsystem extends SubsystemBase {
    
    private CANdle candle = new CANdle(0);

    @Override
    public void periodic() {
        if(SharedState.get().getCoralInWay()){
            candle.setLEDs(255, 0, 0);
        } else {
            candle.setLEDs(0, 255, 0);
        }
    }
}
