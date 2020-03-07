/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterMotorConstants;

public class Shooter extends SubsystemBase
{
	private TalonSRX masterMotor = new TalonSRX(ShooterConstants.leftMotorChannel);
	private TalonSRX slaveMotor = new TalonSRX(ShooterConstants.rightMotorChannel);

	/**
	 * Creates an instance.
	 */
	public Shooter(XboxController controller)
	{
		masterMotor.setInverted(ShooterConstants.isLeftMotorInverted);
		slaveMotor.setInverted(ShooterConstants.isRightMotorInverted);

		slaveMotor.set(ControlMode.Follower, masterMotor.getDeviceID());

		for (TalonSRX motor : new TalonSRX[] { masterMotor, slaveMotor})
		{
			motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
			motor.configAllowableClosedloopError(ShooterMotorConstants.profileSlot, ShooterMotorConstants.pidAllowableError,
					ShooterMotorConstants.pidTimeoutMs);
			motor.configClosedLoopPeakOutput(ShooterMotorConstants.profileSlot, ShooterMotorConstants.pidPeakOutput,
					ShooterMotorConstants.pidTimeoutMs);
			motor.configClosedLoopPeriod(ShooterMotorConstants.profileSlot, ShooterMotorConstants.pidLoopPeriodMs,
					ShooterMotorConstants.pidTimeoutMs);
			motor.config_kP(ShooterMotorConstants.profileSlot, ShooterMotorConstants.pidP,
					ShooterMotorConstants.pidTimeoutMs);
			motor.config_kI(ShooterMotorConstants.profileSlot, ShooterMotorConstants.pidI,
					ShooterMotorConstants.pidTimeoutMs);
			motor.config_kD(ShooterMotorConstants.profileSlot, ShooterMotorConstants.pidD,
					ShooterMotorConstants.pidTimeoutMs);
			motor.config_kF(ShooterMotorConstants.profileSlot, ShooterMotorConstants.pidFF,
					ShooterMotorConstants.pidTimeoutMs);
			motor.config_IntegralZone(ShooterMotorConstants.profileSlot, ShooterMotorConstants.pidIZ,
					ShooterMotorConstants.pidTimeoutMs);
			motor.selectProfileSlot(ShooterMotorConstants.profileSlot, ShooterMotorConstants.primaryClosedLoop);
			motor.setSensorPhase(ShooterMotorConstants.phase);
		}
	}

	/***
	 * Sets the shooter motor speeds in velocity (RPM).
	 */
	public void setVelocity(double veolcity)
	{
		// Velocity is measured in encoder units per 100 ms.

		var unitsPer100Ms =
			veolcity * ShooterMotorConstants.countsPerRevolution * ShooterMotorConstants.ticksPerCount
			/ (60.0 * 1000.0 / 100.0);
		SmartDashboard.putNumber(DashboardConstants.shooterSetVelocityKey, unitsPer100Ms);
		masterMotor.set(ControlMode.Velocity, unitsPer100Ms);
	}

	/***
	 * Sets the shooter motor speeds in percentage.
	 */
	public void setPercentage(double percentage)
	{
		masterMotor.set(ControlMode.PercentOutput, percentage);
	}

	/***
	 * Stops the shooter motors.
	 */
	public void stop()
	{
		setPercentage(0);
	}
}
