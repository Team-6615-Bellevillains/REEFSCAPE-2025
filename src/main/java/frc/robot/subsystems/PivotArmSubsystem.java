package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SharedState;
import frc.robot.utils.SharedUtils;

public class PivotArmSubsystem extends SubsystemBase {
    
    public SparkMax armMotor = new SparkMax(35, MotorType.kBrushless);
    public SparkClosedLoopController armController = armMotor.getClosedLoopController();
    private SparkMax grabberMotor = new SparkMax(33, MotorType.kBrushless);
    private SparkFlex conveyorMotor = new SparkFlex(20, MotorType.kBrushless);
    private boolean out = false;
    private static final double GEAR_RATIO = 9;


    public PivotArmSubsystem(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
        .p(1)
        .i(0)
        .d(0)
        .outputRange(-0.80, 0.5);
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armMotor.getEncoder().setPosition(0);
        armController.setReference( 0, ControlType.kPosition);

        SparkFlexConfig config2 = new SparkFlexConfig();
        config2.idleMode(IdleMode.kBrake);
        conveyorMotor.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        out = false;
    }

    @Override
    public void periodic() {
     //System.out.println((armMotor.getEncoder().getPosition()/9)*360);

     SmartDashboard.putNumber("grabber rpm", grabberMotor.getEncoder().getVelocity());
    }

    public boolean positionOut(){
        return out;
    }

    public void setArmPosition(boolean out){
        if(out){
            armController.setReference(degreesToRotations(30), ControlType.kPosition);
            SharedUtils.setCurrentLimit(armMotor, 40);
        } else {
            SharedUtils.setCurrentLimit(armMotor, 10);
            armMotor.set(-1);
        }
    }

    public double getPivotAngle(){
        return armMotor.getEncoder().getPosition()/(-GEAR_RATIO)*360;
    }

    public double degreesToRotations(double degrees){
        return -(degrees/360)*GEAR_RATIO;
    }

    public Command setArmPositionCommand(boolean out){
        return this.runOnce(()->setArmPosition(out));
    }
    
    public double grabberMotorCurrent(){
        return grabberMotor.getOutputCurrent();
    }
    public double grabberMotorRpm(){
        return grabberMotor.getEncoder().getVelocity();
    }
    public double grabberMotorRotations() {
        return grabberMotor.getEncoder().getPosition();
    }

    public void loadCoral(){
            grabberMotor.set(0.1);
            conveyorMotor.set(-0.1);
        }
    
    public void stopMotors(){
        grabberMotor.stopMotor();
        conveyorMotor.stopMotor();
    };
    public Command spitCoral(){
        return this.runEnd(() -> {
            grabberMotor.set(0.3);
            conveyorMotor.set(-0.1);
            SharedState.get().setLoaded(true);
        }, () -> {
            grabberMotor.stopMotor();
            conveyorMotor.stopMotor();
        });
    }
    
    public Command reverseCoral(){
        return this.runEnd(() -> {
            grabberMotor.set(-0.3);
            conveyorMotor.set(0.1);
        }, () -> {
            grabberMotor.stopMotor();
            conveyorMotor.stopMotor();
        });
    }

    /* 
     * this command sets the grabber position to the opposite of what it was (please forgve me for the awful name)
    */
    public Command invertInOut(){
        return this.runOnce(()->{
            out = !out;
            setArmPosition(out);
        });
    }
}

