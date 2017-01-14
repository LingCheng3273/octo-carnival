package com.stuypulse.frc2017.robot.cv;

import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.Videoio;

import stuyvision.VisionModule;
import stuyvision.capture.DeviceCaptureSource;

public class Camera {
	
	private static final String pathToV4L2 = "/usr/bin/v4l2-ctl";
	
    public static DeviceCaptureSource initializeCamera(int cameraPort) {
        Runtime rt = Runtime.getRuntime();
        String cmdStart = pathToV4L2 + " -d " + cameraPort + " ";
        try {
            rt.exec(cmdStart + "-c exposure_auto=1,exposure_absolute=5,brightness=30,contrast=10,saturation=200,white_balance_temperature_auto=0,sharpness=50").waitFor();
            rt.exec(cmdStart + "-c white_balance_temperature=4624").waitFor();
        } catch (Exception e) {
            System.err.println("Setting v4l settings crashed!");
        }
        DeviceCaptureSource camera = new DeviceCaptureSource(cameraPort);
        System.out.println("Made camera at " + cameraPort);
        camera.setBufferSize(2);
        return camera;
    }

}