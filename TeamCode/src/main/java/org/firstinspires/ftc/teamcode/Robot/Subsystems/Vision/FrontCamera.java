package org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.VisionUtils.Resolution;
import org.firstinspires.ftc.teamcode.visionPipelines.SleeveDetection;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

public class FrontCamera extends Subsystem {
    public Size resolution = Resolution.LOW;
    public double FOV = Math.toRadians(70.428);
    public Pose2d position = new Pose2d(0,0, Math.toRadians(0));
    private OpenCvPipeline pipeline;
    private OpenCvWebcam cam;
    private final OpenCvCameraRotation cameraRotation = OpenCvCameraRotation.UPRIGHT;
    private final long exposureMs = 15;
    private final int gain = 0;
    private boolean open = false;

    public FrontCamera() {
        pipeline = new SleeveDetection();
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        cam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Front Webcam"), hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName()));
        cam.setViewportRenderer(OpenCvWebcam.ViewportRenderer.GPU_ACCELERATED);
        cam.setPipeline(pipeline);
        cam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                open = true;
                cam.startStreaming( (int) resolution.width, (int) resolution.height, cameraRotation);
                cam.getExposureControl().setMode(ExposureControl.Mode.Manual);
                cam.getExposureControl().setExposure(exposureMs, TimeUnit.MILLISECONDS);
                cam.getGainControl().setGain(gain);
                cam.getFocusControl().setMode(FocusControl.Mode.Fixed);
                //cam.getFocusControl().setFocusLength();
            }
            @Override public void onError(int errorCode) { }});
    }

    @Override
    public void periodic() { }

    @Override
    public void shutdown() {
        open = false;
        cam.closeCameraDevice();
    }
    public void close() {
        open = false;
        cam.closeCameraDevice();
    }

    public SleeveDetection.ParkingPosition getParkingPosition() {
        if (cam.getFrameCount() < 1 || !open) {
            return SleeveDetection.ParkingPosition.CENTER;
        }
        assert pipeline instanceof SleeveDetection;
        return ((SleeveDetection) pipeline).getPosition();
    }
}
