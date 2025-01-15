package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.CANcoder;

//Dosyanın içindekiler:
//Drive Motor
//Rotate Motor
//Encoder (açı hesaplaması için)

class swerveModule{
    //Drive Motor
    SparkMax driveMotor;
    SparkMax steeringMotor;

    //Modül durumları
    private SwerveModuleState currentState;
    //Encoder
    CANcoder m_drivingCANcoder;
    CANcoder m_steeringCANCoder;

    //2 Tane PID controller kullanımı
    //1 tanesi sürüş için 1 tanesi de açı hesaplayabilmesi için
    PIDController drivePIDController;
    PIDController steeringPIDController;

    double m_chassisAngularOffset = 0;


    public swerveModule(int driveMotorPort,int steeringMotorPort){
        System.out.println("swerve");
        driveMotor = new SparkMax(driveMotorPort, MotorType.kBrushless);
        
        
        steeringMotor = new SparkMax(steeringMotorPort, MotorType.kBrushless);
        currentState = new SwerveModuleState();

        drivePIDController = new PIDController(0.1, 0, 0);
        steeringPIDController = new PIDController(0.1, 0, 0);
    }

    public SwerveModuleState getState() {
        return currentState;
    }

    public void setDesiredState(SwerveModuleState newState) {
        currentState = newState;
    }

}



public class swerve extends SubsystemBase{

    //Swerve Components
    swerveModule frontLeftSwerve = new swerveModule(2,3);
    swerveModule frontRightSwerve = new swerveModule(4,5);
    swerveModule rearLeftSwerve = new swerveModule(6,7);
    swerveModule rearRightSwerve = new swerveModule(8,9);

// Kinematik obje tanımlayıp onun ChassisSpeed'ini  alıp SwerveModuleState'e çevirme
    double chassisGenislik = Units.inchesToMeters(32);
    double chassisUzunluk = Units.inchesToMeters(32);

    // Tekerlerin robotun bizim belirlediğimiz merkezine olan uzaklığı
    Translation2d frontLeftLocation = new Translation2d(chassisUzunluk/2, chassisGenislik/2);
    Translation2d frontRightLocation = new Translation2d(chassisUzunluk/2, -chassisGenislik/2);
    Translation2d rearLeftLocation = new Translation2d(-chassisUzunluk/2, chassisGenislik/2);
    Translation2d rearRightLocation = new Translation2d(-chassisUzunluk/2, -chassisGenislik/2);

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation,
        frontRightLocation,
        rearLeftLocation,
        rearRightLocation
    );

    CommandJoystick controller;

    //Constructor
    public swerve(CommandJoystick inputoutput){
        System.out.println("swerve");
        controller = inputoutput;
    }

public void setChassisSpeed(ChassisSpeeds desired){
    SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(desired);


    // Tüm swerve modüllerinin durumunu belirlemek (hız/yön)
    frontLeftSwerve.setDesiredState(newStates[0]);
    frontRightSwerve.setDesiredState(newStates[1]);
    rearLeftSwerve.setDesiredState(newStates[2]);
    rearRightSwerve.setDesiredState(newStates[3]);
}

@Override
public void periodic(){

    //Controller'dan veri alma, x ve y verilerini sol joystickten alma
    ChassisSpeeds newDesiredSpeed = new ChassisSpeeds(
        //X ekseni hızı için
        controller.getRawAxis(0),
        //Y ekseni hızı için
        -controller.getRawAxis(1),
        //Z ekseni dönüş için
        controller.getRawAxis(2) 
    );
    
    setChassisSpeed(newDesiredSpeed);

    //Dashboard'a veri yollama
    //Front Left,Front Right,Rear Left, Rear Right
    double loggingState[] = {
        frontLeftSwerve.getState().angle.getDegrees(),
        frontLeftSwerve.getState().speedMetersPerSecond,
        frontRightSwerve.getState().angle.getDegrees(),
        frontRightSwerve.getState().speedMetersPerSecond,
        rearLeftSwerve.getState().angle.getDegrees(),
        rearLeftSwerve.getState().speedMetersPerSecond,
        rearRightSwerve.getState().angle.getDegrees(),
        rearRightSwerve.getState().speedMetersPerSecond,
    };

    SmartDashboard.putNumberArray("swerve module state", loggingState);
}

}
