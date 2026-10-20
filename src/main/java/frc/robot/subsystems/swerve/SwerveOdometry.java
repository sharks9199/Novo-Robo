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

public final class SwerveOdometry extends Thread {
  private static final int QUEUE_CAPACITY = 20;

  private static final double LOOP_PERIOD_SECONDS =
      1.0 / Drive.ODOMETRY_FREQUENCY;

  private static final boolean IS_CAN_FD =
      new CANBus(Constants.Swerve.DRIVETRAIN.CANBusName)
          .isNetworkFD();

  private final Lock signalsLock = new ReentrantLock();

  private BaseStatusSignal[] phoenixSignals =
      new BaseStatusSignal[0];

  private final List<DoubleSupplier> genericSignals =
      new ArrayList<>();

  private final List<Queue<Double>> phoenixQueues =
      new ArrayList<>();

  private final List<Queue<Double>> genericQueues =
      new ArrayList<>();

  private final List<Queue<Double>> timestampQueues =
      new ArrayList<>();

  private volatile boolean running = true;

  private SwerveOdometry() {
    super("PhoenixOdometryThread");
    setDaemon(true);
  }

  private static class InstanceHolder {
    private static final SwerveOdometry INSTANCE =
        new SwerveOdometry();
  }

  public static SwerveOdometry getInstance() {
    return InstanceHolder.INSTANCE;
  }

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

  public Queue<Double> registerSignal(
      StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(
        QUEUE_CAPACITY);

    signalsLock.lock();
    Drive.odometryLock.lock();

    try {
      BaseStatusSignal[] newSignals =
          new BaseStatusSignal[phoenixSignals.length + 1];

      System.arraycopy(
          phoenixSignals,
          0,
          newSignals,
          0,
          phoenixSignals.length);

      newSignals[phoenixSignals.length] = signal;
      phoenixSignals = newSignals;

      phoenixQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
      signalsLock.unlock();
    }

    return queue;
  }

  public Queue<Double> registerSignal(
      DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(
        QUEUE_CAPACITY);

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

  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(
        QUEUE_CAPACITY);

    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }

    return queue;
  }

  public void stopOdometry() {
    running = false;
    interrupt();
  }

  @Override
  public void run() {
    while (running && !Thread.currentThread().isInterrupted()) {
      signalsLock.lock();

      try {
        if (IS_CAN_FD && phoenixSignals.length > 0) {
          BaseStatusSignal.waitForAll(
              LOOP_PERIOD_SECONDS,
              phoenixSignals);
        } else {
          Thread.sleep(
              (long) (LOOP_PERIOD_SECONDS * 1000.0));

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

  private void publishSamples() {
    double timestamp =
        RobotController.getFPGATime() / 1_000_000.0;

    if (phoenixSignals.length > 0) {
      double totalLatency = 0.0;

      for (BaseStatusSignal signal : phoenixSignals) {
        totalLatency += signal.getTimestamp().getLatency();
      }

      timestamp -= totalLatency / phoenixSignals.length;
    }

    for (int i = 0; i < phoenixSignals.length; i++) {
      phoenixQueues.get(i).offer(
          phoenixSignals[i].getValueAsDouble());
    }

    for (int i = 0; i < genericSignals.size(); i++) {
      genericQueues.get(i).offer(
          genericSignals.get(i).getAsDouble());
    }

    for (Queue<Double> queue : timestampQueues) {
      queue.offer(timestamp);
    }
  }
}