package frc.robot.utilities;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class MotorUtil {
    //Creates and returns a new spark max with given ID and settings
    public static CANSparkMax initSparkMax(int ID, boolean invert, boolean setBrakeMode, boolean enableVoltageComp){
        final CANSparkMax newSpark = new CANSparkMax(ID, MotorType.kBrushless);
        newSpark.restoreFactoryDefaults();
        if(setBrakeMode){
            newSpark.setIdleMode(IdleMode.kBrake);
        }else{
            newSpark.setIdleMode(IdleMode.kCoast);
        }
        newSpark.setSmartCurrentLimit(60);
        newSpark.setInverted(invert);
        if(enableVoltageComp){
            newSpark.enableVoltageCompensation(12);
        }
        newSpark.burnFlash();

        return newSpark;
    }
}
