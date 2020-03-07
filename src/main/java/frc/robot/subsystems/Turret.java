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
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.HoodMotorConstants;
import frc.robot.Constants.TurretConstants;

/***
 * A subsystem that controls the turret.
 */
public class Turret extends SubsystemBase
{
	private enum AzimuthPosition
	{
		Unknown,
		LeftOfCenter,
		Center,
		RightOfCenter
	}

	private TalonSRX hoodMotor = new TalonSRX(TurretConstants.hoodMotorChannel);
	private VictorSPX azimuthMotor = new VictorSPX(TurretConstants.azimuthMotorChannel);

	private DigitalInput upHoodLimit = new DigitalInput(TurretConstants.upHoodLimitChannel);
	private DigitalInput downHoodLimit = new DigitalInput(TurretConstants.downHoodLimitChannel);

	private DigitalInput leftAzimuthLimit = new DigitalInput(TurretConstants.leftAzimuthLimitChannel);
	private DigitalInput centerAzimuthLimit = new DigitalInput(TurretConstants.centerAzimuthLimitChannel);
	private DigitalInput rightAzimuthLimit = new DigitalInput(TurretConstants.rightAzimuthLimitChannel);

	private XboxController controller;

	// Where qwe think we are:
	private AzimuthPosition currentAzimuthPosition = AzimuthPosition.Unknown;

	/**
	 * Creates an instance.
	 */
	public Turret(XboxController controller)
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
		azimuthMotor.setInverted(TurretConstants.isAzimuthMotorInverted);

		// If the azimuth is at a limit switch, this will set its current position:
		setCurrentAzimuthPosition();
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

	/**
	 * Run azimuth motor at specified percantage.
	 * 
	 * @param percentage
	 */
	public void setAzimuth(double percentage)
	{
		// See if we are moving through the center position:
		if (!centerAzimuthLimit.get())
		{
			currentAzimuthPosition = AzimuthPosition.Center;
		}
		
		boolean isRumble = false;
		if (percentage > 0)
		{
			// We are moving to the right.
			if (currentAzimuthPosition == AzimuthPosition.Center && centerAzimuthLimit.get())
			{
				// We were at center but not anymore:
				currentAzimuthPosition = AzimuthPosition.RightOfCenter;
			}
			else if (!rightAzimuthLimit.get())
			{
				// We are all the way to the right:
				currentAzimuthPosition = AzimuthPosition.RightOfCenter;
				percentage = 0;
				isRumble = true;
			}
		}
		else if (percentage < 0)
		{
			// We are movine to the left.
			if (currentAzimuthPosition == AzimuthPosition.Center && centerAzimuthLimit.get())
			{
				// We were at center but nut anymore:
				currentAzimuthPosition = AzimuthPosition.LeftOfCenter;
			}
			else if (!leftAzimuthLimit.get())
			{
				// We are all the way to the left:
				currentAzimuthPosition = AzimuthPosition.LeftOfCenter;
				percentage = 0;
				isRumble = true;
			}
		}

		controller.setRumble(RumbleType.kRightRumble, isRumble ? 1 : 0);

		SmartDashboard.putNumber(DashboardConstants.azimuthMotorKey, percentage);
		azimuthMotor.set(ControlMode.PercentOutput, percentage * TurretConstants.azimuthPercentageScaleFactor);
	}

	/**
	 * Moves the turret towards the center (front) position.
	 */
	public void moveToCenterPosition()
	{
		if (!centerAzimuthLimit.get())
		{
			// We are at center, do not move anymore:
			currentAzimuthPosition = AzimuthPosition.Center;
			setAzimuth(0);
		}
		else if (!leftAzimuthLimit.get() || currentAzimuthPosition == AzimuthPosition.LeftOfCenter)
		{
			currentAzimuthPosition = AzimuthPosition.LeftOfCenter;
			setAzimuth(TurretConstants.moveToRightAutoPercentage);
		}
		else if (!rightAzimuthLimit.get() || currentAzimuthPosition == AzimuthPosition.RightOfCenter)
		{
			currentAzimuthPosition = AzimuthPosition.RightOfCenter;
			setAzimuth(TurretConstants.moveToLeftAutoPercentage);
		}
		else
		{
			// We do not know where we are at, move right until we
			// hit center or all the way right.
			setAzimuth(TurretConstants.moveToRightAutoPercentage);
		}
	}

	/***
	 * Convenience method to stop all turret motion.
	 */
	public void stop()
	{
		setHood(0);
		setAzimuth(0);
	}

	/**
	 * If possible, set the current azimuth position.
	 */
	public void setCurrentAzimuthPosition()
	{
		if (!leftAzimuthLimit.get())
		{
			currentAzimuthPosition = AzimuthPosition.LeftOfCenter;
		}
		else if (!centerAzimuthLimit.get())
		{
			currentAzimuthPosition = AzimuthPosition.Center;
		}
		else if (!rightAzimuthLimit.get())
		{
			currentAzimuthPosition = AzimuthPosition.RightOfCenter;
		}
	}

	/**
	 * Indicates if the azimuth is at the center (front) position.
	 */
	public boolean isAtCenterPosition()
	{
		return !centerAzimuthLimit.get();
	}

	public boolean isHoodAtDownPosition()
	{
		return !downHoodLimit.get();
	}

	public boolean isHoodAtUpPosition()
	{
		return !upHoodLimit.get();
	}

	@Override
	public void periodic()
	{
		SmartDashboard.putBoolean(DashboardConstants.downHoodLimitKey, !downHoodLimit.get());
		SmartDashboard.putBoolean(DashboardConstants.upHoodLimitKey, !upHoodLimit.get());

		SmartDashboard.putBoolean(DashboardConstants.leftAzimuthLimitKey, !leftAzimuthLimit.get());
		SmartDashboard.putBoolean(DashboardConstants.frontAzimuthLimitKey, !centerAzimuthLimit.get());
		SmartDashboard.putBoolean(DashboardConstants.rightAzimuthLimitKey, !rightAzimuthLimit.get());

		SmartDashboard.putString(DashboardConstants.turretPositionKey, currentAzimuthPosition.toString());
		SmartDashboard.putNumber(DashboardConstants.hoodCurrentPositionKey, hoodMotor.getSelectedSensorPosition());
	}
}
