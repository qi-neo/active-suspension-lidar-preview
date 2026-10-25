#include "suspension_preview_v2/height_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace suspension_preview_v2 {

namespace {

bool parseCsvLine(const std::string& line, double& mm, double& pwm, std::string& err) {
    std::istringstream ss(line);
    std::string field;
    std::vector<double> vals;
    while (std::getline(ss, field, ',')) {
        // 去掉首尾空白
        const std::size_t a = field.find_first_not_of(" \t\r\n");
        const std::size_t b = field.find_last_not_of(" \t\r\n");
        if (a == std::string::npos) continue;  // 空字段（如尾随逗号）
        const std::string tok = field.substr(a, b - a + 1);
        try {
            vals.push_back(std::stod(tok));
        } catch (...) {
            err = "非数值字段: " + tok;
            return false;
        }
    }
    if (vals.size() < 2) {
        err = "每行至少需要 2 列（target_mm, measured_pwm_us），实际 " +
              std::to_string(vals.size());
        return false;
    }
    mm = vals[0];
    pwm = vals[1];
    return true;
}

}  // namespace

bool HeightCalibration::loadCsv(const std::string& path, std::string& err) {
    err.clear();
    std::ifstream f(path);
    if (!f) {
        err = "无法打开标定文件: " + path;
        return false;
    }
    std::vector<CalibPoint> pts;
    std::string line;
    int lineno = 0;
    bool seen_data = false;   // 是否已经读过至少一个数据行
    while (std::getline(f, line)) {
        ++lineno;
        // 去掉 UTF-8 BOM（Excel 导出的 CSV 常带），否则首行 '#' 注释会被误判为数据
        if (line.size() >= 3 && line[0] == '\xEF' && line[1] == '\xBB' &&
            line[2] == '\xBF')
            line.erase(0, 3);
        // 跳过空行与注释
        const std::size_t a = line.find_first_not_of(" \t\r\n");
        if (a == std::string::npos) continue;
        if (line[a] == '#') continue;
        // 跳过头行：首字段非数值（如 "target_mm,..."）按 CSV 表头处理，不参与换算；
        // 真正的坏数据（如 "0.0, abc"）首字段仍是数值，会进入解析并照常报错。
        //
        // 两个必要的限定（原先缺失，会静默丢数据）：
        //   1) 表头只能出现在第一个数据行之前。原先"首字段非数值就跳过"对每一行
        //      都成立，于是任何坏行都被当成表头悄悄跳掉——标定表少了一个点，
        //      却没有任何报错，插值结果偏了也看不出来。
        //   2) std::stod 对越界数值（如 "1e400"）抛的是 out_of_range，不是
        //      invalid_argument。原先被同一个 catch(...) 吞掉，同样按表头跳过，
        //      于是溢出的数据行会凭空消失。越界是坏数据，必须报错。
        const std::size_t comma = line.find(',');
        const std::string first =
            line.substr(a, comma == std::string::npos ? std::string::npos
                                                      : comma - a);
        try {
            (void)std::stod(first);
        } catch (const std::out_of_range&) {
            err = "第 " + std::to_string(lineno) + " 行: 首字段数值超出 double "
                  "可表示范围: '" + first + "'";
            return false;
        } catch (...) {
            if (seen_data) {
                err = "第 " + std::to_string(lineno) + " 行: 首字段 '" + first +
                      "' 不是数值。表头只允许出现在第一个数据行之前；"
                      "之后的非数值行按坏数据处理，不再静默跳过。";
                return false;
            }
            continue;   // 首个数据行之前的非数值行 = 表头
        }
        seen_data = true;
        double mm = 0.0, pwm = 0.0;
        if (!parseCsvLine(line, mm, pwm, err)) {
            err = "第 " + std::to_string(lineno) + " 行: " + err;
            return false;
        }
        pts.push_back({mm, pwm});
    }
    return setPoints(pts, err);
}

bool HeightCalibration::setPoints(const std::vector<CalibPoint>& pts, std::string& err) {
    err.clear();
    if (pts.size() < 2) {
        err = "标定表至少需要 2 个点，实际 " + std::to_string(pts.size());
        return false;
    }
    // 拷贝后按 mm 升序排序（允许重复输入的乱序）
    std::vector<CalibPoint> sorted = pts;
    std::sort(sorted.begin(), sorted.end(),
              [](const CalibPoint& x, const CalibPoint& y) { return x.mm < y.mm; });
    for (const auto& p : sorted) {
        if (!std::isfinite(p.mm) || !std::isfinite(p.pwm_us)) {
            err = "存在非有限值（mm=" + std::to_string(p.mm) +
                  ", pwm_us=" + std::to_string(p.pwm_us) + "）";
            return false;
        }
        // 量程校验：有限值不等于合法值。舵机只认 500~2500 us，超出的点几乎
        // 一定是单位写错，而它会一路走到硬件（原先只查了 finite，999999 us
        // 能被正常载入并下发）。
        if (p.pwm_us < kMinServoPwmUs || p.pwm_us > kMaxServoPwmUs) {
            err = "pwm_us=" + std::to_string(p.pwm_us) + " 超出舵机量程 [" +
                  std::to_string(kMinServoPwmUs) + ", " +
                  std::to_string(kMaxServoPwmUs) + "] us（对应 mm=" +
                  std::to_string(p.mm) + "）。通常是单位写错（占空比/毫秒/定时器"
                  "计数值被当成了微秒），按原值下发会把舵机顶到机械极限。";
            return false;
        }
    }
    // 严格递增检查（允许极小浮点误差内相等判为重复）
    for (std::size_t i = 1; i < sorted.size(); ++i) {
        if (sorted[i].mm <= sorted[i - 1].mm + 1e-9) {
            err = "target_mm 必须严格递增（第 " + std::to_string(i) + " 点 mm=" +
                  std::to_string(sorted[i].mm) + " 未大于前一点）";
            return false;
        }
    }
    pts_ = std::move(sorted);
    return true;
}

double HeightCalibration::pwmForHeight(double mm) const {
    if (pts_.empty()) return std::numeric_limits<double>::quiet_NaN();
    if (mm <= pts_.front().mm) return pts_.front().pwm_us;  // 下界钳位
    if (mm >= pts_.back().mm) return pts_.back().pwm_us;    // 上界钳位
    // 二分找 mm 所在的区间 [i-1, i]
    std::size_t lo = 0, hi = pts_.size() - 1;
    while (hi - lo > 1) {
        const std::size_t mid = lo + (hi - lo) / 2;
        if (pts_[mid].mm <= mm)
            lo = mid;
        else
            hi = mid;
    }
    const double x0 = pts_[lo].mm, x1 = pts_[hi].mm;
    const double y0 = pts_[lo].pwm_us, y1 = pts_[hi].pwm_us;
    const double t = (mm - x0) / (x1 - x0);  // x1 > x0 由严格递增保证
    return y0 + t * (y1 - y0);
}

double HeightCalibration::heightForPwm(double pwm_us) const {
    if (pts_.empty()) return std::numeric_limits<double>::quiet_NaN();

    // 端点钳位必须先判断方向：反装舵机的 pwm 随高度递减，此时 front 是最大值、
    // back 是最小值。原先无条件写 "<= front 就返回 front"，在递减表上会把区间内
    // 的查询全部钳到最低点（实测：1750 us -> 0 mm，正确值 5 mm）。
    const bool descending = pts_.back().pwm_us < pts_.front().pwm_us;
    if (descending) {
        if (pwm_us >= pts_.front().pwm_us) return pts_.front().mm;
        if (pwm_us <= pts_.back().pwm_us) return pts_.back().mm;
    } else {
        if (pwm_us <= pts_.front().pwm_us) return pts_.front().mm;
        if (pwm_us >= pts_.back().pwm_us) return pts_.back().mm;
    }

    // 线性扫描找包含 pwm_us 的那一段。
    //
    // 原来是二分——二分要求被查序列单调，而标定表只保证 mm 单调，pwm 可能
    // 递减也可能非单调（回程差、机构死区）。标定表通常只有几到几十个点，
    // 线性扫描的代价可以忽略，换来的是对任意形状的表都给出确定的答案，
    // 而不是"碰巧单调就对、反装就错"。
    for (std::size_t i = 1; i < pts_.size(); ++i) {
        const double y0 = pts_[i - 1].pwm_us;
        const double y1 = pts_[i].pwm_us;
        const double lo = std::min(y0, y1);
        const double hi = std::max(y0, y1);
        if (pwm_us < lo || pwm_us > hi) continue;
        if (!(std::fabs(y1 - y0) > 0.0)) return pts_[i - 1].mm;  // 退化段
        const double t = (pwm_us - y0) / (y1 - y0);
        return pts_[i - 1].mm + t * (pts_[i].mm - pts_[i - 1].mm);
    }

    // 非单调表：查询值可能落在任何一段之外。退化为"pwm 最接近的那个标定点"，
    // 比返回固定端点或 0 合理，且 pwmMonotonic() 会如实报告这张表不可靠
    std::size_t best = 0;
    double best_d = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < pts_.size(); ++i) {
        const double d = std::fabs(pts_[i].pwm_us - pwm_us);
        if (d < best_d) {
            best_d = d;
            best = i;
        }
    }
    return pts_[best].mm;
}

bool HeightCalibration::pwmMonotonic() const {
    if (pts_.size() < 3) return true;
    const bool desc = pts_.back().pwm_us < pts_.front().pwm_us;
    for (std::size_t i = 1; i < pts_.size(); ++i) {
        if (desc) {
            if (pts_[i].pwm_us > pts_[i - 1].pwm_us) return false;
        } else {
            if (pts_[i].pwm_us < pts_[i - 1].pwm_us) return false;
        }
    }
    return true;
}

std::string HeightCalibration::toJson() const {
    std::string s = "[";
    for (std::size_t i = 0; i < pts_.size(); ++i) {
        if (i) s += ",";
        s += "{\"mm\":" + std::to_string(pts_[i].mm) +
             ",\"pwm_us\":" + std::to_string(pts_[i].pwm_us) + "}";
    }
    s += "]";
    return s;
}

std::string HeightCalibration::toHeader(const std::string& varName) const {
    std::string s = "const CalibPoint " + varName + "[] = {\n";
    for (const auto& p : pts_) {
        s += "  {mm:" + std::to_string(p.mm) +
             ", pwm_us:" + std::to_string(p.pwm_us) + "},\n";
    }
    s += "};\n";
    return s;
}

bool HeightCalibration::strictlyIncreasing() const {
    for (std::size_t i = 1; i < pts_.size(); ++i) {
        if (pts_[i].mm <= pts_[i - 1].mm) return false;
    }
    return true;
}

}  // namespace suspension_preview_v2
