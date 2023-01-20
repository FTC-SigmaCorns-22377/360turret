package org.firstinspires.ftc.teamcode.VisionUtils;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.opencv.core.Point;

public class VisionBasedPosition {
    public double distance;
    public double angle;
    public Pose2d cameraPosition;

    public VisionBasedPosition(double distance, double angle, Pose2d cameraPosition) {
        this.distance = distance;
        this.angle = angle;
        this.cameraPosition = cameraPosition;
    }

}
