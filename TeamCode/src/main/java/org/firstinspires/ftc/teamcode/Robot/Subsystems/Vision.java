package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.visionPipelines.Cam;
import org.firstinspires.ftc.teamcode.visionPipelines.VisionPipe;
import org.firstinspires.ftc.teamcode.visionPipelines.MonocularPole;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Vision extends Subsystem {

    public OpenCvWebcam webcam;

    public FtcDashboard dashboard = FtcDashboard.getInstance();

    static final Size low = new Size(320,240); // bad aspect ratio don't use
    static final Size medium = new Size(960,540);
    static final Size high = new Size(1280,720);
    static final Size hd = new Size(1920,1080);

    static Size resolution = medium;

    public Cam cam = new Cam(0, resolution, new Size(0,4),Math.toRadians(70.428), new Size(1920,1080),1);// for 78 dfov

//    OpenCvPipeline pipeline = new VisionPipe(cam);
    private VisionPipe pipe = new VisionPipe(cam);

    static ArrayList<MonocularPole> rawPoles;
    private Drivetrain drivetrain;

    private boolean hasInitialized = false;
    private boolean isDestroyed = false;

    public void pauseView(){
        webcam.pauseViewport();
    }
    public void resumeView(){
        webcam.resumeViewport();
    }

    public Vision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    void run() {

    }
    @Override
    public void initAuto(HardwareMap hwMap) {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(pipe);
        webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override public void onOpened() { webcam.startStreaming( (int) resolution.width, (int) resolution.height,OpenCvCameraRotation.UPRIGHT); }
            @Override public void onError(int errorCode) { }
        });

        Dashboard.packet.put("CAMCONFIG: Gain Set Success", webcam.getGainControl().setGain(1));
        Dashboard.packet.put("CAMCONFIG: Exposure Mode Success", webcam.getExposureControl().setMode(ExposureControl.Mode.Manual));
        Dashboard.packet.put("CAMCONFIG: Exposure Time Success", webcam.getExposureControl().setExposure(20L, TimeUnit.MILLISECONDS));
        Dashboard.packet.put("CAMCONFIG: Focus Mode Success", webcam.getFocusControl().setMode(FocusControl.Mode.Auto));

        dashboard.startCameraStream(webcam,5);


        rawPoles = new ArrayList<MonocularPole>();
    }

    @Override
    public void periodic() {
        cam.currentFrame= webcam.getFrameCount();
        if (cam.currentFrame> cam.lastFrame) { run(); }
        cam.lastFrame= cam.currentFrame;
        Dashboard.packet.put("Cam FPS", webcam.getFps());
        Dashboard.packet.put("Cam Frame", webcam.getFrameCount());

    }

    @Override
    public void shutdown() {
        webcam.closeCameraDevice();
    }

    public VisionPipe.ParkingPosition getPosition() {
        if (!hasInitialized || isDestroyed) {
            return VisionPipe.ParkingPosition.CENTER;
        }
        return pipe.getPosition();
    }

    public void destroy() {
        isDestroyed = true;
        webcam.closeCameraDevice();
    }
}
