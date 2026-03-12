#pragma once
/// @file frame_queue.hpp
/// Thread-safe, bounded, single-producer / single-consumer frame queue.
///
/// Designed for streaming camera pipelines where:
///   - A producer thread copies raw sensor data into the queue as fast as
///     possible and returns the camera buffer.
///   - A consumer thread dequeues frames for compression and publishing.
///
/// Allocation strategy
/// -------------------
/// All internal data buffers are pre-allocated once via `reserve_buffers()`.
/// The consumer swaps a spare buffer into the slot it just consumed so the
/// producer always writes into an already-reserved vector — zero per-frame
/// heap allocations on the producer side.  The consumer pays one
/// `reserve()` per frame to restore its spare after moving data out.

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

/// Metadata + pixel data for a single captured frame.
struct CapturedFrame {
  std::vector<uint8_t> data;
  int64_t  timestamp_ns  = 0;
  uint64_t frame_id      = 0;
  bool     is_big_endian = false;
  size_t   bits_per_pixel = 0;
  size_t   width         = 0;
};

/// Bounded, blocking SPSC queue for `CapturedFrame` objects.
///
/// @tparam Capacity  Maximum number of frames that can be buffered.
template <size_t Capacity = 4>
class FrameQueue {
 public:
  FrameQueue() = default;

  // Non-copyable, non-movable.
  FrameQueue(const FrameQueue&) = delete;
  FrameQueue& operator=(const FrameQueue&) = delete;

  /// Pre-allocate every internal buffer to `bytes` so the producer never
  /// triggers a heap allocation during normal operation.
  void reserve_buffers(size_t bytes) {
    for (auto& slot : slots_) {
      slot.data.reserve(bytes);
    }
    spare_.reserve(bytes);
    reserved_bytes_ = bytes;
  }

  /// Signal all blocked threads to wake up and exit.
  void shutdown() {
    shutdown_.store(true, std::memory_order_release);
    not_empty_.notify_all();
    not_full_.notify_all();
  }

  /// Current number of frames buffered (approximate, lock-free read).
  size_t depth() const { return depth_.load(std::memory_order_acquire); }

  /// Number of times the producer blocked on a full queue.
  /// A non-zero value means the consumer (GPU) is the bottleneck and camera
  /// buffers are being starved of RequeueBuffer() calls.
  uint64_t enqueue_stalls() const {
    return enqueue_stalls_.load(std::memory_order_relaxed);
  }

  // ---------------------------------------------------------------------------
  // Producer API
  // ---------------------------------------------------------------------------

  /// Copy raw pixel data + metadata into the next free slot.
  /// Blocks if the queue is full.  Returns false if shutdown was requested.
  bool enqueue(const uint8_t* pixel_data, size_t byte_count,
               int64_t timestamp_ns, uint64_t frame_id,
               bool is_big_endian, size_t bits_per_pixel, size_t width) {
    std::unique_lock<std::mutex> lock(mutex_);
    // Count a stall if we are about to block on a full queue — this is the
    // telemetry signal that camera buffers are being held too long.
    if (count_ >= Capacity && !shutdown_.load(std::memory_order_acquire)) {
      enqueue_stalls_.fetch_add(1, std::memory_order_relaxed);
    }
    not_full_.wait(lock, [this] {
      return count_ < Capacity || shutdown_.load(std::memory_order_acquire);
    });
    if (shutdown_.load(std::memory_order_acquire)) return false;

    auto& slot = slots_[head_];
    std::memcpy(slot.data.data(), pixel_data, byte_count);  // write into pre-reserved capacity
    slot.data.resize(byte_count);              // set logical size (no capacity change)
    slot.timestamp_ns  = timestamp_ns;
    slot.frame_id      = frame_id;
    slot.is_big_endian = is_big_endian;
    slot.bits_per_pixel = bits_per_pixel;
    slot.width         = width;

    head_ = (head_ + 1) % Capacity;
    ++count_;
    depth_.store(count_, std::memory_order_release);

    lock.unlock();
    not_empty_.notify_one();
    return true;
  }

  // ---------------------------------------------------------------------------
  // Consumer API
  // ---------------------------------------------------------------------------

  /// Dequeue the next frame.  Metadata fields are written via out-params;
  /// the heavy pixel buffer is swapped into `data_out` (zero-copy).
  /// Returns false if shutdown was requested and the queue is empty.
  bool dequeue(std::vector<uint8_t>& data_out,
               int64_t& timestamp_ns, uint64_t& frame_id,
               bool& is_big_endian, size_t& bits_per_pixel, size_t& width) {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait(lock, [this] {
      return count_ > 0 || shutdown_.load(std::memory_order_acquire);
    });
    if (shutdown_.load(std::memory_order_acquire) && count_ == 0) return false;

    auto& slot     = slots_[tail_];
    timestamp_ns   = slot.timestamp_ns;
    frame_id       = slot.frame_id;
    is_big_endian  = slot.is_big_endian;
    bits_per_pixel = slot.bits_per_pixel;
    width          = slot.width;

    // O(1) swap: slot gets the pre-reserved spare (ready for the producer),
    // data_out gets the actual frame data.
    std::swap(slot.data, spare_);
    std::swap(spare_, data_out);

    tail_ = (tail_ + 1) % Capacity;
    --count_;
    depth_.store(count_, std::memory_order_release);

    lock.unlock();
    not_full_.notify_one();

    // Restore spare capacity so the next swap gives the producer a usable
    // buffer.  The caller (consumer) moved data_out into a ROS message, so
    // spare_ was left empty after the swap chain above — reserve it again.
    if (spare_.capacity() < reserved_bytes_) {
      spare_.reserve(reserved_bytes_);
    }
    return true;
  }

 private:
  std::array<CapturedFrame, Capacity> slots_;
  size_t head_  = 0;
  size_t tail_  = 0;
  size_t count_ = 0;  // guarded by mutex_

  std::mutex              mutex_;
  std::condition_variable not_empty_;
  std::condition_variable not_full_;

  std::atomic<bool>     shutdown_{false};
  std::atomic<size_t>   depth_{0};           // lock-free mirror of count_
  std::atomic<uint64_t> enqueue_stalls_{0};  // incremented each time producer blocks on full queue

  // Spare buffer used by the consumer swap chain.
  std::vector<uint8_t> spare_;
  size_t reserved_bytes_ = 0;
};
