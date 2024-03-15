package frc.robot.utilities;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class MotorUtil {
    /**Creates and returns a new spark max with set ID, invert, brake or coast mode, and voltage compensation OFF 
     * @param ID The CAN ID for the SparkMax.
     * @param invert Set True to invert motor, otherwise leave false
     * @param setBrakeMode Set True to set motor to break mode, False for coast mode
    */
    public static CANSparkMax initSparkMax(int ID, boolean invert, boolean setBrakeMode){
        final CANSparkMax newSpark = new CANSparkMax(ID, MotorType.kBrushless);
        newSpark.restoreFactoryDefaults();
        if(setBrakeMode){
            newSpark.setIdleMode(IdleMode.kBrake);
        }else{
            newSpark.setIdleMode(IdleMode.kCoast);
        }
        newSpark.setSmartCurrentLimit(60);
        newSpark.setInverted(invert);
        newSpark.burnFlash();

        return newSpark;
    }
    /**Creates and returns a new spark max with set ID, invert, brake or coast mode, and voltage compensation OFF 
     * @param ID The CAN ID for the SparkMax.
     * @param invert Set True to invert motor, otherwise leave false
     * @param setBrakeMode Set True to set motor to break mode, False for coast mode
     * @param voltageComp Enables and sets voltage compensation to specified value. Recommend 12 or under
    */
    public static CANSparkMax initSparkMax(int ID, boolean invert, boolean setBrakeMode, int voltageComp){
        final CANSparkMax newSpark = new CANSparkMax(ID, MotorType.kBrushless);
        newSpark.restoreFactoryDefaults();
        if(setBrakeMode){
            newSpark.setIdleMode(IdleMode.kBrake);
        }else{
            newSpark.setIdleMode(IdleMode.kCoast);
        }
        newSpark.setSmartCurrentLimit(60);
        newSpark.setInverted(invert);
        newSpark.enableVoltageCompensation(voltageComp);
        newSpark.burnFlash();

        return newSpark;
    }
}
