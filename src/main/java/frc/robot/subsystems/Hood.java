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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.HoodMotorConstants;
import frc.robot.Constants.TurretConstants;

/***
 * A subsystem that controls the hood.
 */
public class Hood extends SubsystemBase
{
	private TalonSRX hoodMotor = new TalonSRX(TurretConstants.hoodMotorChannel);

	private DigitalInput upHoodLimit = new DigitalInput(TurretConstants.upHoodLimitChannel);
	private DigitalInput downHoodLimit = new DigitalInput(TurretConstants.downHoodLimitChannel);

	private XboxController controller;

	/**
	 * Creates an instance.
	 */
	public Hood(XboxController controller)
	{
		this.controller = controller;

		hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
		hoodMotor.configAllowableClosedloopError(
			HoodMotorConstants.profileSlot,
			HoodMotorConstants.pidAllowableError,
			HoodMotorConstants.pidTimeoutMs);
		hoodMotor.configClosedLoopPeakOutput(
			HoodMotorConstants.profileSlot,
			HoodMotorConstants.pidPeakOutput,
			HoodMotorConstants.pidTimeoutMs);
		hoodMotor.configClosedLoopPeriod(
			HoodMotorConstants.profileSlot,
			HoodMotorConstants.pidLoopPeriodMs,
			HoodMotorConstants.pidTimeoutMs);
		hoodMotor.config_kP(
			HoodMotorConstants.profileSlot,
			HoodMotorConstants.pidP,
			HoodMotorConstants.pidTimeoutMs);
		hoodMotor.config_kI(
			HoodMotorConstants.profileSlot,
			HoodMotorConstants.pidI,
			HoodMotorConstants.pidTimeoutMs);
		hoodMotor.config_kD(
			HoodMotorConstants.profileSlot,
			HoodMotorConstants.pidD,
			HoodMotorConstants.pidTimeoutMs);
		hoodMotor.config_kF(
			HoodMotorConstants.profileSlot,
			HoodMotorConstants.pidFF,
			HoodMotorConstants.pidTimeoutMs);
		hoodMotor.config_IntegralZone(
			HoodMotorConstants.profileSlot,
			HoodMotorConstants.pidIZ,
			HoodMotorConstants.pidTimeoutMs);
		hoodMotor.selectProfileSlot(
			HoodMotorConstants.profileSlot,
			HoodMotorConstants.primaryClosedLoop);

		hoodMotor.setSensorPhase(HoodMotorConstants.phase);

		hoodMotor.setInverted(TurretConstants.isHoodMotorInverted);
	}

	/**
	 * Moves the hood to the specifird position, in native units.
	 * @param position
	 */
	public void setHoodPosition(double position)
	{
		hoodMotor.set(ControlMode.Position, position);
	}

	/**
	 * Run hood motor at specified percantage.
	 * 
	 * @param percentage
	 */
	public void setHood(double percentage)
	{
		if (percentage < 0 && !downHoodLimit.get() || percentage > 0 && !upHoodLimit.get())
		{
			percentage = 0;
			controller.setRumble(RumbleType.kLeftRumble, 1);
		}
		else
		{
			controller.setRumble(RumbleType.kLeftRumble, 0);
		}

		// If the hood is all the way down, set its position to 0:
		if (!downHoodLimit.get())
		{
			hoodMotor.setSelectedSensorPosition(0);
		}

		SmartDashboard.putNumber(DashboardConstants.hoodMotorKey, percentage);

		// Scale percentage to reasonable speed:
		percentage *= percentage < 0
			? TurretConstants.hoodDownPercentageScaleFactor
			: TurretConstants.hoodUpPercentageScaleFactor;
		hoodMotor.set(ControlMode.PercentOutput, percentage);
	}

	/***
	 * Convenience method to stop all turret motion.
	 */
	public void stop()
	{
		setHood(0);
	}

	public boolean isHoodAtDownPosition()
	{
		return !downHoodLimit.get();
	}

	public boolean isHoodAtUpPosition()
	{
		return !upHoodLimit.get();
	}

	public double getCurrentHoodPosition()
	{
		return hoodMotor.getSelectedSensorPosition();
	}

	@Override
	public void periodic()
	{
		SmartDashboard.putBoolean(DashboardConstants.downHoodLimitKey, !downHoodLimit.get());
		SmartDashboard.putBoolean(DashboardConstants.upHoodLimitKey, !upHoodLimit.get());

		SmartDashboard.putNumber(DashboardConstants.hoodCurrentPositionKey, hoodMotor.getSelectedSensorPosition());
	}
}
