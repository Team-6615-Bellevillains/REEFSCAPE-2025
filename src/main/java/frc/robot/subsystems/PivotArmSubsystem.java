package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SharedState;
import frc.robot.utils.SharedUtils;

@Logged
public class PivotArmSubsystem extends SubsystemBase {
    public SparkMax armMotor = new SparkMax(35, MotorType.kBrushless);
    public SparkClosedLoopController armController = armMotor.getClosedLoopController();

    private SparkMax grabberMotor = new SparkMax(33, MotorType.kBrushless);
    private SparkFlex conveyorMotor = new SparkFlex(20, MotorType.kBrushless);

    private Position position = Position.IN;
    private static final Dimensionless CONVERSION_FACTOR = Rotations.of(25).div(Degrees.of(360));
    private LaserCan laserCan = new LaserCan(0);


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

        setArmPosition(Position.IN);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Coral ROtation", getPivotAngle().in(Degrees));
        SmartDashboard.putNumber("Grabber Motor Current:", grabberMotorCurrent());
        SmartDashboard.putNumber("grabber motor rpm:", grabberMotorRpm());
        SmartDashboard.putNumber("arm current", armMotor.getOutputCurrent());
        SharedState.get().setCoralInWay(measureCoralInWay());
    }

    public void setArmPosition(Position position){
        switch (position) {
            case IN:
                SharedUtils.setCurrentLimit(armMotor, 8);
                armMotor.set(1);
                break;
            case OUT:
                SharedUtils.setCurrentLimit(armMotor, 2);
                armMotor.set(-1);
                break;
        }
    }

    public Angle getPivotAngle(){
        return Rotations.of(armMotor.getEncoder().getPosition())
                        .div(CONVERSION_FACTOR);
    }

    public Command setArmPositionCommand(Position position){
        return this.runOnce(()->setArmPosition(position));
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
        grabberMotor.set(0.15);
        conveyorMotor.set(-0.10);
        };

    public void reverse(){  
        grabberMotor.set(-0.1);
        conveyorMotor.set(0.05);
        };
    public void stopMotors(){
        grabberMotor.stopMotor();
        conveyorMotor.stopMotor();
    };

    public void setGrabberCurrentLimit(int currentLimit) {
        SharedUtils.setCurrentLimit(grabberMotor, currentLimit);
    }

    public void setGrabberMotor(double percentage) {
        grabberMotor.set(percentage);
    }

    public void throwBall(){
        grabberMotor.set(-0.75);
    }

    public Command l1Shot(){
        return this.runEnd(()->{
            grabberMotor.set(1);
        }, ()->{
            grabberMotor.stopMotor();
        });
    }

    public Command throwBallCommand(){
        return this.runEnd(()->{
            throwBall();
        }, ()->{
            stopMotors();
        });
    }

    public Command spitCoral(){
        return this.runEnd(() -> {
            grabberMotor.set(0.3);
            conveyorMotor.set(-0.1);
            SharedState.get().setLoaded(false);
        }, () -> {
            grabberMotor.stopMotor();
            conveyorMotor.stopMotor();
        });
    }

    public Command L1Shot(){
        return this.runEnd(() -> {
            grabberMotor.set(0.6);
        }, () -> {
            grabberMotor.stopMotor();
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
            setArmPosition(position == Position.IN ? Position.OUT : Position.IN);
        });
    }

    public Command grabAlgaeBargeShotCommand(){
        return this.runOnce(()->{
            grabberMotor.set(0.15);
        });
    }


    public Command resetCoralAngleCommand(){
        return this.runOnce(()->{
            armMotor.getEncoder().setPosition(0);
        });
    }


    public boolean measureCoralInWay(){
        Measurement measurement = laserCan.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            SharedState.get().setLaserCanDistance(measurement.distance_mm);
        }
        if(measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm < 350){
            return true;
        } else return false;
    }
    
    public enum Position {
        IN,
        OUT
    }

}

