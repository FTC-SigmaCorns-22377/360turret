package org.firstinspires.ftc.teamcode.VisionUtils;

import org.firstinspires.ftc.teamcode.visionPipelines.Fast;
import org.opencv.core.Point;
import org.opencv.core.Size;

public class Cone {
    public enum Classification {
        PERFECT,
        FAR,
        CLOSE
    }
    public Size size;
    public VisionBasedPosition position;
    public Point top;
    public Classification classification;

    public Cone(Size size, VisionBasedPosition position, Point top){
        this.size = size;
        this.position = position;
        this.top = top;
        this.classify();
    }
    private void classify() {
        if (this.position.distance < Fast.ConeConfig.perfectDistance - Fast.ConeConfig.perfectTolerance) {
            this.classification = Classification.CLOSE;
        }
        else if (this.position.distance > Fast.ConeConfig.perfectDistance + Fast.ConeConfig.perfectTolerance) {
            this.classification = Classification.FAR;
        }
        else {
            this.classification = Classification.PERFECT;
        }
    }
}
