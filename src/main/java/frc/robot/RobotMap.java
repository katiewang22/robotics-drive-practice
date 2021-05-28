package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class RobotMap {
    public static final int LEFT_FRONT_PORT = 3;
    public static final int RIGHT_FRONT_PORT = 4;
    public static final int LEFT_BACK_PORT = 1;
    public static final int RIGHT_BACK_PORT = 2;

    public static WPI_TalonFX leftFrontDriveMotor = new WPI_TalonFX(LEFT_FRONT_PORT);
    public static WPI_TalonFX rightFrontDriveMotor = new WPI_TalonFX(RIGHT_FRONT_PORT);
    public static WPI_TalonFX leftBackDriveMotor = new WPI_TalonFX(LEFT_BACK_PORT);
    public static WPI_TalonFX rightBackDriveMotor = new WPI_TalonFX(RIGHT_BACK_PORT);

    public static PigeonIMU arm_imu = new PigeonIMU(0);
    public static PigeonIMU drive_imu = new PigeonIMU(1);
}