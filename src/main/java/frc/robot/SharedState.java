package frc.robot;

import edu.wpi.first.epilogue.Logged;

@Logged
public class SharedState {
    private boolean loaded;
    private boolean aligned;
    private boolean coralInWay;
    private double laserCanDistance;

    private static SharedState instance = null;

    private SharedState() {}

    public static SharedState get() {
        if (instance == null) {
            instance = new SharedState();
        }

        return instance;
    }

    public void setLoaded(boolean loaded) {
        this.loaded = loaded;
    }
    
    public boolean isLoaded() {
        return loaded;
    }

    public void setAligned(boolean aligned){
        this.aligned = aligned;
    }    

    public boolean getAligned(){
        return this.aligned;
    }

    public void setCoralInWay(boolean input){
        coralInWay = input;
    }

    public boolean getCoralInWay(){
        return coralInWay;
    }

    public void setLaserCanDistance(double laserCanDistance) {
        this.laserCanDistance = laserCanDistance;
    }

    public double getLaserCanDistance(){
        return laserCanDistance;
    }
}
