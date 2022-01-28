package frc.robot.subsystems;

import org.junit.Test;

import static org.junit.Assert.*;

public class DriveTest {
    public static final double kTestEpsilon = 1e-4;

    @Test
    public void testEncoderCountsToMeters() {
        // 0 counts -> 0 meters
        assertEquals(0, Drive.encoderCountsToMeters(0), kTestEpsilon);

        // 54538.7942 counts -> 1 meter
        assertEquals(1, Drive.encoderCountsToMeters(54538.7942), kTestEpsilon);

        // -54538.7942 counts -> -1 meter
        assertEquals(-1, Drive.encoderCountsToMeters(-54538.7942), kTestEpsilon);

        // 168034.0249 counts -> 3.081 meter
        assertEquals(3.081, Drive.encoderCountsToMeters(168034.0249), kTestEpsilon);
    }

    @Test
    public void testEncoderRateToMetersPerSecond() {
        // 0 counts/100ms -> 0 meters/sec
        assertEquals(0, Drive.encoderRateToMetersPerSecond(0), kTestEpsilon);

        // 54538.7942 counts/100ms -> 10 meter/sec
        assertEquals(10, Drive.encoderRateToMetersPerSecond(54538.7942), kTestEpsilon);

        // -54538.7942 counts/100ms -> -10 meter/sec
        assertEquals(-10, Drive.encoderRateToMetersPerSecond(-54538.7942), kTestEpsilon);

        // 168034.0249 counts/100ms -> 30.81 meter/sec
        assertEquals(30.81, Drive.encoderRateToMetersPerSecond(168034.0249), kTestEpsilon);
    }
}
