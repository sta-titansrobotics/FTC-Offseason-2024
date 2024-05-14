package Camera;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.util.ArrayList;

public class TestMockAprilTagDetectionPipeline {
    public static void main(String[] args) {
        // Mock image data
        Mat mockImage = new Mat(640, 480, CvType.CV_8UC3);

        // Set up the mock AprilTag detection pipeline
        double fx = 640.0;
        double fy = 640.0;
        double cx = 320.0;
        double cy = 240.0;
        MockAprilTagDetectionPipeline pipeline = new MockAprilTagDetectionPipeline(fx, fy, cx, cy);

        // Process the mock image
        ArrayList<MockAprilTagDetectionPipeline.AprilTagDetection> detections = pipeline.processFrame(mockImage);

        // Print the mock detections
        if (detections != null && !detections.isEmpty()) {
            for (MockAprilTagDetectionPipeline.AprilTagDetection detection : detections) {
                System.out.println("Detected tag ID=" + detection.id);
                System.out.printf("Translation X: %.2f meters\n", detection.pose.x);
                System.out.printf("Translation Y: %.2f meters\n", detection.pose.y);
                System.out.printf("Translation Z: %.2f meters\n", detection.pose.z);
            }
        } else {
            System.out.println("No tag detected.");
        }
    }
}
