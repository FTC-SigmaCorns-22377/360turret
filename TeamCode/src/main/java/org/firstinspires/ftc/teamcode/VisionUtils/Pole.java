package org.firstinspires.ftc.teamcode.VisionUtils;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

public class Pole {

    public MatOfPoint contour;
    public RotatedRect rect;
    public VisionBasedPosition position;
    public Pose2d dEstimate;
    public Point top;

    public Pole(MatOfPoint contour,RotatedRect rect, VisionBasedPosition position, Point top, double cameraAngle){
        this.contour = contour;
        this.rect = rect;
        this.position = position;
        this.top = top;
        this.calculate(cameraAngle);
    }
    private void calculate(double cameraAngle) {
        this.dEstimate = new Pose2d(Math.copySign(Math.sin(this.position.angle) * this.position.distance, this.position.angle),Math.cos(this.position.angle) * this.position.distance);
    }
}