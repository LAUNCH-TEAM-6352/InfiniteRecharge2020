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
import frc.robot.Constants.HoodConstants;

/***
 * A subsystem that controls the hood.
 */
public class Hood extends SubsystemBase
{
	private TalonSRX hoodMotor = new TalonSRX(HoodConstants.motorChannel);

	private DigitalInput upHoodLimit = new DigitalInput(HoodConstants.upLimitChannel);
	private DigitalInput downHoodLimit = new DigitalInput(HoodConstants.downLimitChannel);

	private XboxController controller;

	/**
	 * Creates an instance.
	 */
	public Hood(XboxController controller)
	{
		this.controller = controller;

		hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
		hoodMotor.configAllowableClosedloopError(
			HoodConstants.profileSlot,
			HoodConstants.pidAllowableError,
			HoodConstants.pidTimeoutMs);
		hoodMotor.configClosedLoopPeakOutput(
			HoodConstants.profileSlot,
			HoodConstants.pidPeakOutput,
			HoodConstants.pidTimeoutMs);
		hoodMotor.configClosedLoopPeriod(
			HoodConstants.profileSlot,
			HoodConstants.pidLoopPeriodMs,
			HoodConstants.pidTimeoutMs);
		hoodMotor.config_kP(
			HoodConstants.profileSlot,
			HoodConstants.pidP,
			HoodConstants.pidTimeoutMs);
		hoodMotor.config_kI(
			HoodConstants.profileSlot,
			HoodConstants.pidI,
			HoodConstants.pidTimeoutMs);
		hoodMotor.config_kD(
			HoodConstants.profileSlot,
			HoodConstants.pidD,
			HoodConstants.pidTimeoutMs);
		hoodMotor.config_kF(
			HoodConstants.profileSlot,
			HoodConstants.pidFF,
			HoodConstants.pidTimeoutMs);
		hoodMotor.config_IntegralZone(
			HoodConstants.profileSlot,
			HoodConstants.pidIZ,
			HoodConstants.pidTimeoutMs);
		hoodMotor.selectProfileSlot(
			HoodConstants.profileSlot,
			HoodConstants.primaryClosedLoop);

		hoodMotor.setSensorPhase(HoodConstants.phase);

		hoodMotor.setInverted(HoodConstants.isMotorInverted);
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
			? HoodConstants.downPercentageScaleFactor
			: HoodConstants.upPercentageScaleFactor;
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
