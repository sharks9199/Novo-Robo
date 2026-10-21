package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Background thread that samples drivetrain signals (module positions, gyro yaw) at a high
 * frequency and publishes them, together with a shared timestamp, into queues that are drained
 * by the main robot loop.
 *
 * <p>On CAN FD buses the thread blocks on the signals' arrival; on classic CAN it polls at the
 * configured odometry frequency. Access to the queues is synchronized with the main thread through
 * {@link Drive#odometryLock}.
 */
public final class SwerveOdometry extends Thread {
  /** Maximum number of samples buffered per queue. */
  private static final int QUEUE_CAPACITY = 20;

  private static final double LOOP_PERIOD_SECONDS = 1.0 / Drive.ODOMETRY_FREQUENCY;

  private static final boolean IS_CAN_FD =
      new CANBus(Constants.Swerve.DRIVETRAIN.CANBusName).isNetworkFD();

  /** Guards the signal arrays/lists while the thread is waiting on them. */
  private final Lock signalsLock = new ReentrantLock();

  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];

  private final List<DoubleSupplier> genericSignals = new ArrayList<>();

  private final List<Queue<Double>> phoenixQueues = new ArrayList<>();

  private final List<Queue<Double>> genericQueues = new ArrayList<>();

  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private volatile boolean running = true;

  private SwerveOdometry() {
    super("PhoenixOdometryThread");
    setDaemon(true);
  }

  // ---------------------------------------------------------------------------
  // Singleton
  // ---------------------------------------------------------------------------

  /** Lazy, thread-safe holder for the singleton instance. */
  private static class InstanceHolder {
    private static final SwerveOdometry INSTANCE = new SwerveOdometry();
  }

  /** Returns the singleton odometry thread. */
  public static SwerveOdometry getInstance() {
    return InstanceHolder.INSTANCE;
  }

  // ---------------------------------------------------------------------------
  // Lifecycle
  // ---------------------------------------------------------------------------

  /**
   * Starts the thread if it has not been started yet and at least one timestamp queue has been
   * registered.
   */
  @Override
  public synchronized void start() {
    if (getState() != State.NEW) {
      return;
    }

    Drive.odometryLock.lock();
    try {
      if (!timestampQueues.isEmpty()) {
        super.start();
      }
    } finally {
      Drive.odometryLock.unlock();
    }
  }

  /** Requests the thread to stop. */
  public void stopOdometry() {
    running = false;
    interrupt();
  }

  // ---------------------------------------------------------------------------
  // Signal registration
  // ---------------------------------------------------------------------------

  /**
   * Registers a Phoenix signal to be sampled by the thread.
   *
   * @param signal the signal to sample
   * @return a queue that receives the sampled values
   */
  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(QUEUE_CAPACITY);

    signalsLock.lock();
    Drive.odometryLock.lock();

    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];

      System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);

      newSignals[phoenixSignals.length] = signal;
      phoenixSignals = newSignals;

      phoenixQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
      signalsLock.unlock();
    }

    return queue;
  }

  /**
   * Registers a generic (non-Phoenix) signal to be sampled by the thread.
   *
   * @param signal supplier of the value to sample
   * @return a queue that receives the sampled values
   */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(QUEUE_CAPACITY);

    signalsLock.lock();
    Drive.odometryLock.lock();

    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
      signalsLock.unlock();
    }

    return queue;
  }

  /**
   * Creates a queue that receives the timestamp of every sampling cycle.
   *
   * @return the timestamp queue
   */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(QUEUE_CAPACITY);

    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }

    return queue;
  }

  // ---------------------------------------------------------------------------
  // Thread body
  // ---------------------------------------------------------------------------

  @Override
  public void run() {
    while (running && !Thread.currentThread().isInterrupted()) {
      signalsLock.lock();

      try {
        if (IS_CAN_FD && phoenixSignals.length > 0) {
          // CAN FD: block until all signals have fresh data.
          BaseStatusSignal.waitForAll(LOOP_PERIOD_SECONDS, phoenixSignals);
        } else {
          // Classic CAN: sleep for one period, then poll the latest values.
          Thread.sleep((long) (LOOP_PERIOD_SECONDS * 1000.0));

          if (phoenixSignals.length > 0) {
            BaseStatusSignal.refreshAll(phoenixSignals);
          }
        }

        Drive.odometryLock.lock();
        try {
          publishSamples();
        } finally {
          Drive.odometryLock.unlock();
        }

      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        break;
      } finally {
        signalsLock.unlock();
      }
    }
  }

  /**
   * Pushes the latest values of every registered signal, plus a latency-compensated timestamp, into
   * their queues. Must be called while holding {@link Drive#odometryLock}.
   */
  private void publishSamples() {
    double timestamp = RobotController.getFPGATime() / 1_000_000.0;

    // Compensate for the average CAN latency of the Phoenix signals.
    if (phoenixSignals.length > 0) {
      double totalLatency = 0.0;

      for (BaseStatusSignal signal : phoenixSignals) {
        totalLatency += signal.getTimestamp().getLatency();
      }

      timestamp -= totalLatency / phoenixSignals.length;
    }

    for (int i = 0; i < phoenixSignals.length; i++) {
      phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
    }

    for (int i = 0; i < genericSignals.size(); i++) {
      genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
    }

    for (Queue<Double> queue : timestampQueues) {
      queue.offer(timestamp);
    }
  }
}
