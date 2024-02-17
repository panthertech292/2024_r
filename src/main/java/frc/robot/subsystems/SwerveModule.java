package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.robot.Constants;

public class SwerveModule {
    //ID
    public int moduleID;
    //Motors
    private CANSparkMax DriveMotor;
    private CANSparkMax AngleMotor;
    //Encoders
    private RelativeEncoder DriveEncoder;
    private RelativeEncoder NEOAngleEncoder;
    private CANCoder AngleCANCoder;
    //PIDs
    private SparkMaxPIDController DrivePID;
    private SparkMaxPIDController AnglePID;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);
    //Angles
    private Rotation2d v_offsetAngle;
    private Rotation2d v_lastAngle;

    public SwerveModule(int moduleID, SwerveModuleConstants ModuleConstants){
        this.moduleID = moduleID;
        this.v_offsetAngle = ModuleConstants.angleOffset;

        //Angle Encoder Init & Config
        AngleCANCoder = new CANCoder(ModuleConstants.cancoderID);
        configAngleEncoder();
        //Angle Motor Init & Config
        AngleMotor = new CANSparkMax(ModuleConstants.angleMotorID, MotorType.kBrushless);
        NEOAngleEncoder = AngleMotor.getEncoder();
        AnglePID = AngleMotor.getPIDController();
        configAngleMotor();
        //Drive Motor Init & Config
        DriveMotor = new CANSparkMax(ModuleConstants.driveMotorID, MotorType.kBrushless);
        DriveEncoder = DriveMotor.getEncoder();
        DrivePID = DriveMotor.getPIDController();
        configDriveMotor();

        v_lastAngle = getState().angle;
    }
    //Get and Set
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        //This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }
    public void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01)) ? v_lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        AnglePID.setReference(angle.getDegrees(), ControlType.kPosition);
        v_lastAngle = angle;
    }
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            DriveMotor.set(percentOutput);    
        }
        else {
            DrivePID.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity,0, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }
    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(AngleCANCoder.getAbsolutePosition());
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(DriveEncoder.getVelocity(), getAngle()); 
    }
    public Rotation2d getOffset(){
        return v_offsetAngle;
    }
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(DriveEncoder.getPosition(), getAngle());
    }
    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(NEOAngleEncoder.getPosition());
    }

    //Configs
    private void configAngleEncoder(){
        AngleCANCoder.configFactoryDefault();
        CANCoderConfiguration canConderConfig = new CANCoderConfiguration();
        canConderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canConderConfig.sensorDirection = Constants.SwerveConstants.canCoderInvert;
        canConderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canConderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        AngleCANCoder.configAllSettings(canConderConfig);
        //CANCoderUtil.setCANCoderBusUsage(AngleCANCoder, CCUsage.kMinimal); //TODO: Might want this?
    }
    private void configAngleMotor(){
        AngleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(AngleMotor, Usage.kPositionOnly); //TODO: Might want this?
        AngleMotor.setSmartCurrentLimit(Constants.SwerveConstants.angleContinuousCurrentLimit);
        AngleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
        AngleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);
        NEOAngleEncoder.setPositionConversionFactor(Constants.SwerveConstants.angleConversionFactor);
        AnglePID.setP(Constants.SwerveConstants.angleKP);
        AnglePID.setI(Constants.SwerveConstants.angleKI);
        AnglePID.setD(Constants.SwerveConstants.angleKD);
        AnglePID.setFF(Constants.SwerveConstants.angleKF);
        AngleMotor.enableVoltageCompensation(Constants.kVoltageComp);
        AngleMotor.burnFlash();
        resetToAbsolute();
    }
    public void resetToAbsolute(){
        double absolutePosition = getCanCoder().getDegrees() - v_offsetAngle.getDegrees();
        AngleMotor.getEncoder().setPosition(absolutePosition);
    }
    private void configDriveMotor(){
        DriveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(DriveMotor, Usage.kAll); //TODO: Might want this?
        DriveMotor.setSmartCurrentLimit(Constants.SwerveConstants.driveContinuousCurrentLimit);
        DriveMotor.setInverted(Constants.SwerveConstants.driveMotorInvert);
        DriveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);
        DriveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.driveConversionVelocityFactor);
        DriveEncoder.setPositionConversionFactor(Constants.SwerveConstants.driveConversionPositionFactor);
        DrivePID.setP(Constants.SwerveConstants.angleKP);
        DrivePID.setI(Constants.SwerveConstants.angleKI);
        DrivePID.setD(Constants.SwerveConstants.angleKD);
        DrivePID.setFF(Constants.SwerveConstants.angleKF);
        DriveMotor.enableVoltageCompensation(Constants.kVoltageComp);
        DriveMotor.burnFlash();
        DriveEncoder.setPosition(0.0);
    }
    
}
