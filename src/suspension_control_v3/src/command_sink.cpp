#include "suspension_control_v3/command_sink.hpp"

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <utility>

namespace suspension_control_v3 {
namespace {

using Clock = std::chrono::steady_clock;
using Driver = suspension_preview_v2::ISuspensionDriver;
using Cmd = std::array<int, kWheelCount>;
using DriverCmd = std::array<int, suspension_preview_v2::kSuspensionWheelCount>;

DriverCmd to_driver_cmd(const Cmd& c) { return {c[0], c[1], c[2], c[3]}; }

double since_ms(Clock::time_point t, Clock::time_point now) {
    return std::chrono::duration<double, std::milli>(now - t).count();
}

// ---------------------------------------------------------------------------
// 同步通道：行为与旧版完全一致（写失败不重试，成功才记账）
// ---------------------------------------------------------------------------
class DirectCommandSink : public ICommandSink {
public:
    explicit DirectCommandSink(std::shared_ptr<Driver> driver) : driver_(std::move(driver)) {}

    void submit(const Cmd& cmd) override {
        std::lock_guard<std::mutex> lk(mu_);
        ++stats_.submitted;
        if (!driver_) return;
        if (driver_->set_target_height(to_driver_cmd(cmd))) {
            ++stats_.sent;
            last_sent_ = Clock::now();
        } else {
            ++stats_.failed;
        }
    }

    void request_emergency_stop() override {
        std::lock_guard<std::mutex> lk(mu_);
        if (!driver_) return;
        if (driver_->emergency_stop()) {
            ++stats_.estop_sent;
            estop_delivered_ = true;
        } else {
            ++stats_.failed;
        }
    }

    bool emergency_stop_delivered() const override {
        std::lock_guard<std::mutex> lk(mu_);
        return estop_delivered_;
    }

    bool is_async() const override { return false; }

    bool flush(double) override { return true; }  // 提交即完成，没有队列

    CommandSinkStats stats() const override {
        std::lock_guard<std::mutex> lk(mu_);
        CommandSinkStats s = stats_;
        s.last_sent_age_ms =
            last_sent_ == Clock::time_point{} ? -1.0 : since_ms(last_sent_, Clock::now());
        return s;
    }

private:
    mutable std::mutex mu_;
    std::shared_ptr<Driver> driver_;
    CommandSinkStats stats_;
    bool estop_delivered_{false};
    Clock::time_point last_sent_{};
};

// ---------------------------------------------------------------------------
// 异步通道：控制线程只投递，后台线程负责真的写出去
// ---------------------------------------------------------------------------
class AsyncCommandSink : public ICommandSink {
public:
    AsyncCommandSink(std::shared_ptr<Driver> driver, double retry_interval_s)
        : driver_(std::move(driver)),
          retry_interval_(std::chrono::duration_cast<Clock::duration>(
              std::chrono::duration<double>(retry_interval_s > 0.0 ? retry_interval_s : 0.02))),
          worker_(&AsyncCommandSink::run, this) {}

    ~AsyncCommandSink() override {
        // 退出前把最后一帧尽量发出去：进程退出不应该把作动器停在半路的指令上
        flush(1.0);
        {
            std::lock_guard<std::mutex> lk(mu_);
            stop_ = true;
        }
        cv_.notify_all();
        if (worker_.joinable()) worker_.join();
    }

    void submit(const Cmd& cmd) override {
        {
            std::lock_guard<std::mutex> lk(mu_);
            ++stats_.submitted;
            // 覆盖未发送的旧值：下行只有几十 Hz 时，中间值本来就没有执行意义
            if (has_cmd_) ++stats_.coalesced;
            cmd_ = cmd;
            has_cmd_ = true;
        }
        cv_.notify_one();
    }

    void request_emergency_stop() override {
        {
            std::lock_guard<std::mutex> lk(mu_);
            if (estop_delivered_) return;
            estop_requested_ = true;
        }
        cv_.notify_one();
    }

    bool emergency_stop_delivered() const override {
        std::lock_guard<std::mutex> lk(mu_);
        return estop_delivered_;
    }

    bool is_async() const override { return true; }

    bool flush(double timeout_s) override {
        std::unique_lock<std::mutex> lk(mu_);
        const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                                std::chrono::duration<double>(timeout_s));
        return cv_.wait_until(lk, deadline, [this] {
            return !has_cmd_ && (!estop_requested_ || estop_delivered_);
        });
    }

    CommandSinkStats stats() const override {
        std::lock_guard<std::mutex> lk(mu_);
        CommandSinkStats s = stats_;
        s.last_sent_age_ms =
            last_sent_ == Clock::time_point{} ? -1.0 : since_ms(last_sent_, Clock::now());
        return s;
    }

private:
    void run() {
        for (;;) {
            Cmd cmd{};
            bool do_cmd = false;
            bool do_estop = false;
            {
                std::unique_lock<std::mutex> lk(mu_);
                cv_.wait(lk, [this] { return stop_ || estop_requested_ || has_cmd_; });
                // 注意：急停送达后 estop_requested_ 必须清掉，否则 stop 之后
                // 这个条件永远不成立，worker 会空转、析构 join 挂死（已被测试抓到）
                if (stop_ && !has_cmd_ && (!estop_requested_ || estop_delivered_)) break;
                // 优先路径：急停插队。只要还没送达就先处理它，不排队
                if (estop_requested_ && !estop_delivered_) {
                    do_estop = true;
                } else if (has_cmd_) {
                    cmd = cmd_;
                    has_cmd_ = false;
                    do_cmd = true;
                }
            }

            if (do_estop) {
                const bool ok = driver_ && driver_->emergency_stop();
                {
                    std::lock_guard<std::mutex> lk(mu_);
                    if (ok) {
                        estop_delivered_ = true;
                        estop_requested_ = false;  // 已送达：清请求，允许 worker 退出
                        ++stats_.estop_sent;
                    } else {
                        ++stats_.failed;
                    }
                }
                if (!ok) std::this_thread::sleep_for(retry_interval_);
                cv_.notify_all();
                continue;
            }

            if (do_cmd) {
                const bool ok = driver_ && driver_->set_target_height(to_driver_cmd(cmd));
                {
                    std::lock_guard<std::mutex> lk(mu_);
                    if (ok) {
                        ++stats_.sent;
                        last_sent_ = Clock::now();
                    } else {
                        ++stats_.failed;
                        // 失败不丢：放回队列重试；期间控制回路的新值会直接覆盖它
                        if (!has_cmd_) {
                            cmd_ = cmd;
                            has_cmd_ = true;
                        }
                    }
                }
                cv_.notify_all();
                if (!ok) std::this_thread::sleep_for(retry_interval_);
            }
        }
    }

    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::shared_ptr<Driver> driver_;
    const Clock::duration retry_interval_;
    std::thread worker_;

    Cmd cmd_{};
    bool has_cmd_{false};
    bool estop_requested_{false};
    bool estop_delivered_{false};
    bool stop_{false};
    CommandSinkStats stats_;
    Clock::time_point last_sent_{};
};

}  // namespace

std::shared_ptr<ICommandSink> make_direct_command_sink(
    std::shared_ptr<suspension_preview_v2::ISuspensionDriver> driver) {
    return std::make_shared<DirectCommandSink>(std::move(driver));
}

std::shared_ptr<ICommandSink> make_async_command_sink(
    std::shared_ptr<suspension_preview_v2::ISuspensionDriver> driver, double retry_interval_s) {
    return std::make_shared<AsyncCommandSink>(std::move(driver), retry_interval_s);
}

}  // namespace suspension_control_v3
