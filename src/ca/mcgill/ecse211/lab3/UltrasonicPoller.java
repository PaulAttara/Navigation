package ca.mcgill.ecse211.lab3;

import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;

/**
 * Control of the wall follower is applied periodically by the UltrasonicPoller thread. The while
 * loop at the bottom executes in a loop. Assuming that the us.fetchSample, and cont.processUSData
 * methods operate in about 20mS, and that the thread sleeps for 50 mS at the end of each loop, then
 * one cycle through the loop is approximately 70 mS. This corresponds to a sampling rate of 1/70mS
 * or about 14 Hz.
 */
public class UltrasonicPoller extends Thread {
  //private SampleProvider us;
  private MedianFilter mf;
  public int distance;
  private float[] usData;

  public UltrasonicPoller(SampleProvider us) {
    this.mf = new MedianFilter(us, 7);
    usData = new float[us.sampleSize()];
    distance = 40;
  }

  /*
   * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
   * [0,255] (non-Javadoc)
   * 
   * @see java.lang.Thread#run()
   */
  public void run() {
    try {
      Thread.sleep(100);
    } catch (Exception e) {}
    while (true) {
      mf.fetchSample(usData, 0); // acquire data
      distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      } // Poor man's timed sampling
    }
  }

}
