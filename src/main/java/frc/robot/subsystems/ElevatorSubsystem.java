package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax leftMotor = new SparkMax(34, MotorType.kBrushless);
    private SparkMax rightMotor = new SparkMax(36, MotorType.kBrushless);
    private SparkClosedLoopController leftController = leftMotor.getClosedLoopController();
    private SparkClosedLoopController rightController = rightMotor.getClosedLoopController();
    private static final double downLimit = 0.4;
    private static final double upLimit = 0.35; 
    private static final double elevatorHeight = 57;
    private static final double rotationLimit = 46.125;
    private Position position;
    private static final double l2Inches = 10.25+5;
    private static final double l3Inches = 25.75+5;
    private static final double l4Inches = elevatorHeight+0.5;

    public ElevatorSubsystem(){
        SparkMaxConfig config = new SparkMaxConfig();
        rightMotor.getEncoder().setPosition(0);
        leftMotor.getEncoder().setPosition(0);
        config.closedLoop
        .p(0.2)
        .i(0)
        .d(0.15)
        .outputRange(-upLimit, downLimit);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.closedLoop.outputRange(-downLimit, upLimit);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        position = Position.L1;
    }

    @Override
    public void periodic() {
        // if(Math.abs(leftMotor.getEncoder().getVelocity()) <= 1 && !atPosition()){
        //     if (this.getCurrentCommand() != null) this.getCurrentCommand().cancel();
        //     if(position == Position.L4 || position == Position.L3){
        //         setPosition(Position.L1);
        //     } else {
        //         setPosition(Position.L1);
        //     }
        //     System.err.println("Elevator stuck!");
        // }
    }

    private double inchesToRotations(double inches){
        return (rotationLimit/elevatorHeight)*inches;
    }

    private double getPositionInches(){
        return (leftMotor.getEncoder().getPosition())/(rotationLimit/elevatorHeight);
    }

    private void moveElevator(double inches){
        System.out.println("Moving elevator to "+inches);
        rightController.setReference(-inchesToRotations(inches), ControlType.kPosition);
        leftController.setReference(inchesToRotations(inches), ControlType.kPosition);
    }

    public void setPosition(Position newPosition){
        switch(newPosition){
            case L1:
                moveElevator(0);
                position = Position.L1;
                break;
            case L2:
                moveElevator(l2Inches);
                position = Position.L2;
                break;
            case L3:
                moveElevator(l3Inches);
                position = Position.L3;
                break;
            case L4:
                moveElevator(l4Inches);  
                position = Position.L4;
                break;
        }
    }

    public boolean atPosition(){
        switch (position) {
            case L1:
                if (getPositionInches()<1.0){
                    return true;
                } else return false;
            case L2:
                if (getPositionInches()<(l2Inches+1) && getPositionInches()>(l2Inches-1)){
                    return true;
                } else return false;

            case L3:
                if (getPositionInches()<(l3Inches+1) && getPositionInches()>(l3Inches-1)){
                    return true;
                } else return false;
            case L4:
                if (getPositionInches()<(l4Inches+1) && getPositionInches()>(l4Inches-1)){
                    return true;
                } else return false;
            default:
                return false;
                
        }
    }

    public Position getPosition(){
        return position;
    }
    
    //public Command setPositionCommand(Position level){
    //    return this.runOnce(() -> setPosition(level));
    //}

    public enum Position{
        L1,
        L2,
        L3,
        L4
    }
}
