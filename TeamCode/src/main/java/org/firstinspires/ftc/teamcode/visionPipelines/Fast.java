package org.firstinspires.ftc.teamcode.visionPipelines;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.teamcode.VisionUtils.Cone;
import org.firstinspires.ftc.teamcode.VisionUtils.VisionBasedPosition;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

public class Fast extends OpenCvPipeline {
    @Config
    public static class ConeConfig {
        public static double distanceCorrection = .8;
        public static int coneMinArea = 900;

        public static int RED_MIN_HUE = 161;
        public static int RED_MIN_SATURATION = 110;
        public static int RED_MIN_VALUE = 50;

        public static int RED_MAX_HUE = 181;
        public static int RED_MAX_SATURATION = 255;
        public static int RED_MAX_VALUE = 255;

        public static int BLUE_MIN_HUE = 146;
        public static int BLUE_MIN_SATURATION = 75;
        public static int BLUE_MIN_VALUE = 50;

        public static int BLUE_MAX_HUE = 196;
        public static int BLUE_MAX_SATURATION = 255;
        public static int BLUE_MAX_VALUE = 255;
        public static double perfectDistance = 13.5;
        public static double perfectTolerance = 2.5;
    }

    private static final double CONE_WIDTH = 4;
    private static final Scalar GREEN = new Scalar(0,255,0);
    private static final Scalar PURPLE = new Scalar(255,0,255);
    private static final Scalar ORANGE = new Scalar(255,165,0);
    private Point camCenter;

    private final Mat structuringSmall = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(3,3));
    private final Mat structuringMedium = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(5,5));
    private final Mat structuringLarge = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(7,7));
    private final Mat hierarchy = new Mat();
    private final ArrayList<Cone> cones = new ArrayList<>();
    private final ArrayList<MatOfPoint> rawContours = new ArrayList<>();
    public List<Cone> perfectList;
    public List<Cone> farList;
    public List<Cone> closeList;
    public volatile Cone perfect = null;
    public volatile Cone far = null;
    public volatile Cone close = null;
    public volatile Cone conestackGuess = null;
    private Team team;
    private Size resolution;
    private double FOV;
    private Pose2d cameraPosition;
    private Rect tempRect;
    private Point tempPoint;
    private Mat mask = new Mat();

    public Fast(Team team, Size resolution, double FOV, Pose2d cameraPosition) {
        this.team = team;
        this.resolution = resolution;
        this.FOV = FOV;
        this.cameraPosition = cameraPosition;
        this.camCenter = getCenter(this.resolution);
    }
    @Override
    public Mat processFrame(Mat input) {
        if (this.team == Team.RED) {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2HSV_FULL);
            Core.inRange(input, new Scalar(ConeConfig.RED_MIN_HUE, ConeConfig.RED_MIN_SATURATION, ConeConfig.RED_MIN_VALUE), new Scalar(ConeConfig.RED_MAX_HUE, ConeConfig.RED_MAX_SATURATION, ConeConfig.RED_MAX_VALUE), this.mask);
        }
        else if (this.team == Team.BLUE) {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV_FULL);
            Core.inRange(input, new Scalar(ConeConfig.BLUE_MIN_HUE, ConeConfig.BLUE_MIN_SATURATION, ConeConfig.BLUE_MIN_VALUE), new Scalar(ConeConfig.BLUE_MAX_HUE, ConeConfig.BLUE_MAX_SATURATION, ConeConfig.BLUE_MAX_VALUE), this.mask);
        }
        else {
            return input;
        }

        Imgproc.morphologyEx(this.mask,this.mask,Imgproc.MORPH_ERODE, this.structuringLarge);
        Imgproc.morphologyEx(this.mask,this.mask,Imgproc.MORPH_DILATE, this.structuringMedium);

        Imgproc.findContours(this.mask, this.rawContours, this.hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        this.cones.clear();
        for (MatOfPoint contour: this.rawContours) {
            this.tempRect = Imgproc.boundingRect(contour);
            this.tempPoint = getTop(contour);
            if (Imgproc.contourArea(contour) >= ConeConfig.coneMinArea && this.tempRect.height > this.tempRect.width) {
                this.cones.add(new Cone(this.tempRect.size(), new VisionBasedPosition(this.getDistance(this.tempRect.width,CONE_WIDTH), this.getAngle(this.tempPoint), this.cameraPosition), this.tempPoint));
            }
        }
        this.rawContours.clear();

        this.perfect = null;
        this.far = null;
        this.close = null;
        this.conestackGuess = null;
        if (this.cones.size() > 0) {
            this.perfectList = this.cones.stream().filter(cone -> cone.classification == Cone.Classification.PERFECT).collect(Collectors.toList());
            if (this.perfectList.size() > 0) this.perfect = Collections.min(perfectList, Comparator.comparing(cone -> Math.abs(cone.position.distance - ConeConfig.perfectDistance)));

            this.farList = this.cones.stream().filter(cone -> cone.classification == Cone.Classification.FAR).collect(Collectors.toList());
            if (this.farList.size() > 0) this.far = Collections.min(this.farList, Comparator.comparing(cone -> Math.abs(cone.position.distance - ConeConfig.perfectDistance)));

            this.closeList = this.cones.stream().filter(cone -> cone.classification == Cone.Classification.CLOSE).collect(Collectors.toList());
            if (this.closeList.size() > 0) this.close = Collections.min(this.closeList, Comparator.comparing(cone -> Math.abs(cone.position.distance - ConeConfig.perfectDistance)));

            this.conestackGuess = Collections.max(this.cones, Comparator.comparing(cone -> cone.size.height));
        }

        this.cones.clear();

//        if (this.perfect != null)
//            Imgproc.drawMarker(input, this.perfect.top,GREEN, Imgproc.MARKER_STAR);
//        if (this.far != null)
//            Imgproc.drawMarker(input, this.far.top,PURPLE, Imgproc.MARKER_SQUARE);
//        if (this.close != null)
//            Imgproc.drawMarker(input, this.close.top,ORANGE, Imgproc.MARKER_TILTED_CROSS);
//        input.setTo(new Scalar(0,0,0), this.mask);
        return this.mask;
    }

    private static Point getTop(MatOfPoint contour) {
        return Collections.min(contour.toList(), Comparator.comparing(h -> h.y));
    }
    private static Point getCenter(Size size) {
        return new Point(size.width/2, size.height/2);
    }
    private double getDistance(double width, double realWidth) {
        double occupiedFOV = this.FOV * (width / this.resolution.width);
        return ConeConfig.distanceCorrection * ( (realWidth/2)/Math.tan(occupiedFOV/2) + (realWidth/2) );
    }
    private double getAngle(Point point) {
        return this.FOV * (point.x / this.resolution.width) - this.FOV/2;
    }
}
