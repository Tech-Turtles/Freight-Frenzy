package org.firstinspires.ftc.teamcode.Utility.Vision;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class UGCoffeeDetector {
    private OpenCvCamera camera;
    private boolean isUsingWebcam;
    private String webcamName;
    private HardwareMap hardwareMap;
    private UGContourRingPipeline pipeline;


    //The constructor is overloaded to allow the use of webcam instead of the phone camera
    public UGCoffeeDetector(HardwareMap hMap) {
        hardwareMap = hMap;
    }

    public UGCoffeeDetector(HardwareMap hMap, String webcamName) {
        hardwareMap = hMap;
        isUsingWebcam = true;
        this.webcamName = webcamName;
    }

    public void init() {
        //This will instantiate an OpenCvCamera object for the camera we'll be using
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        if (isUsingWebcam) {
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        //Set the pipeline the camera should use and start streaming
        pipeline = new UGContourRingPipeline();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(() -> camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));
    }

    public RingDetectionAmount getHeight() {
        try {
            return pipeline.getHeight();
        } catch (NullPointerException e) {
            Log.w("Vision Error", "NullPointerException on the vision pipeline");
            return  RingDetectionAmount.ZERO;
        }
    }
}