/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LimelightConstants;

/**
 * This is a singleton for accessing the Limelight camera.
 */
public class LimelightCamera
{
	private static LimelightCamera instance = null;

	// The following are for accessing the network table entries used by the Limelight:
	private NetworkTable limelightTable = null;
	private NetworkTableEntry pipelineEntry = null;
	private NetworkTableEntry xPositionEntry = null;
	private NetworkTableEntry yPositionEntry = null;
	private NetworkTableEntry areaEntry = null;
	private NetworkTableEntry ledModeEntry = null;
	private NetworkTableEntry targetAcquiredEntry = null;
	

	public synchronized static LimelightCamera getInstance()
	{
		if (instance == null)
		{
			instance = new LimelightCamera();
		}

		return instance;
	}

	/**
	 * Constructor is private as the only way to get an instance
	 * is through the getInstance() static methoid.
	 */
	private LimelightCamera()
	{
		limelightTable = NetworkTableInstance.getDefault().getTable(LimelightConstants.tableName);

		pipelineEntry = limelightTable.getEntry(LimelightConstants.pipelineEntryName);
		xPositionEntry = limelightTable.getEntry(LimelightConstants.xPositionEntryName);
		yPositionEntry = limelightTable.getEntry(LimelightConstants.yPositionEntryName);
		areaEntry = limelightTable.getEntry(LimelightConstants.areaEntryName);
		ledModeEntry = limelightTable.getEntry(LimelightConstants.ledModeEntryName);
		targetAcquiredEntry = limelightTable.getEntry(LimelightConstants.targetAcquiredEntryName);

		setLedMode(LimelightConstants.ledModePipeline);
		setPipeline(LimelightConstants.pipelineDefault);
	}

	/**
	 * Determines if the Limelight hs acquired a target.
	 */
	public boolean isTargetAcquired()
	{
		return targetAcquiredEntry.getDouble(0) != 0;
	}
	
	/**
	 * Returns the x coordinate of the center of the target.
	 */
	public double getTargetXPosition()
	{
		return xPositionEntry.getDouble(0.0);
	}
	
	/**
	 * Returns the y coordinate of the center of the target.
	 */
	public double getTargetYPosition()
	{
		return yPositionEntry.getDouble(0.0);
	}

	/**
	 * Returns the target area as a percentage of the total image area.
	 */
	public double getTargetArea()
	{
		return areaEntry.getDouble(0);
	}

	/***
	 * Acitvates the selected pipeline.
	 */
	public void setPipeline(int pipeline)
	{
		pipelineEntry.setDouble(pipeline);
	}

	/***
	 * Returns the camera's current pipeline.
	 */
	public int getPipeline()
	{
		return (int) pipelineEntry.getDouble(LimelightConstants.pipelineDefault);
	}

	/***
	 * Activates the selected LED mode.
	 */
	public void setLedMode(int ledMode)
	{
		ledModeEntry.setDouble(ledMode);
	}

	/***
	 * Returns the active LED mode.
	 */
	public int getLedMode()
	{
		return (int) ledModeEntry.getDouble(LimelightConstants.ledModePipeline);
	}
}
