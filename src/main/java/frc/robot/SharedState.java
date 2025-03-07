package frc.robot;

public class SharedState {
    private boolean loaded;

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
    
}
