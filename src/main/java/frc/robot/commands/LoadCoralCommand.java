package frc.robot.commands;


import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SharedState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Position;


public class LoadCoralCommand extends Command{
    private final PivotArmSubsystem subsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private int sawTrigger;
    private int tryingToReverse;
    private double rotationsAtStart;
    private double rotationsAtCapture;
    private double rotationsAtReverse;
    private LinearFilter coralMotorRpm_hpFilter;
    private Timer reverseTimer = new Timer(); 

    
    public LoadCoralCommand(PivotArmSubsystem subsystem, ElevatorSubsystem elevatorSubsystem){
        this.subsystem = subsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(subsystem, elevatorSubsystem);
        coralMotorRpm_hpFilter = LinearFilter.highPass(0.1, 0.02);
    }

    @Override
    public void initialize() {
        if (elevatorSubsystem.getPosition() != Position.L1) {
            this.end(true);
            return;
        }

        System.out.println("Initializing LoadCoral");
        sawTrigger = tryingToReverse=0;
        subsystem.loadCoral();
        coralMotorRpm_hpFilter.reset();
        rotationsAtStart = subsystem.grabberMotorRotations();
        SharedState.get().setLoaded(false);
       // coralMotorRpm = subsystem.grabberMotorRpm();

        subsystem.setGrabberCurrentLimit(2);

    }
    
    @Override
    public void execute() {
         
        double coralMotorRpm_Filter = coralMotorRpm_hpFilter.calculate(subsystem.grabberMotorRpm()); 
        
        SmartDashboard.putNumber("high pass filtered rpm:", coralMotorRpm_Filter);
        SmartDashboard.putNumber("rotations since trigger:", rotationsSinceTrigger());
        SmartDashboard.putNumber("rotations when load button is pressed:", rotationsAtStart);
        SmartDashboard.putNumber("rotations when coral captured by shooter:", rotationsAtCapture);
        SmartDashboard.putNumber("total rotations:", subsystem.grabberMotorRotations());
        SmartDashboard.putNumber("trying to reverse:", tryingToReverse);
        SmartDashboard.putNumber("rotations when reversed:", rotationsAtReverse);
        SmartDashboard.putNumber("saw trigger:", sawTrigger);
        SmartDashboard.putNumber("rotations when reversed:", rotationsAtReverse);
    
        // check to see if we've backed up enough
        if ((tryingToReverse != 0) && (rotationsAtReverse - subsystem.grabberMotorRotations() > 20)){
            tryingToReverse=0;
            subsystem.loadCoral();
        }

        // check for rpm disruption caused by loading of coral, if coral load detected, 
        if ((coralMotorRpm_Filter < -10)&&(tryingToReverse == 0)) { 
            sawTrigger++;
            if((sawTrigger == 1) && (tryingToReverse == 0)){
                rotationsAtCapture = subsystem.grabberMotorRotations();
            }
        }
        else{
            System.out.println("checking if conveyor should be reversed");
            if ((subsystem.grabberMotorRotations() - rotationsAtStart) >30){
                tryingToReverse++;
                subsystem.reverse();
                rotationsAtReverse = subsystem.grabberMotorRotations();
            }}}
    @Override
    public boolean isFinished() {
        return sawTrigger > 0 && rotationsSinceTrigger()>4;
    }
            
    @Override
    public void end(boolean cancelled) {
        if (!cancelled) {
            SharedState.get().setLoaded(true);
        }
        subsystem.stopMotors();
        subsystem.setGrabberCurrentLimit(40);
    }

    private double rotationsSinceTrigger(){
        return subsystem.grabberMotorRotations()-rotationsAtCapture;
    }
}
