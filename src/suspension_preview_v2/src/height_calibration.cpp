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
        // 真正的坏数据（如 "0.0, abc"）首字段仍是数值，会进入解析并照常报错
        const std::size_t comma = line.find(',');
        const std::string first =
            line.substr(a, comma == std::string::npos ? std::string::npos
                                                      : comma - a);
        try {
            (void)std::stod(first);
        } catch (...) {
            continue;
        }
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
    if (pwm_us <= pts_.front().pwm_us) return pts_.front().mm;
    if (pwm_us >= pts_.back().pwm_us) return pts_.back().mm;
    std::size_t lo = 0, hi = pts_.size() - 1;
    while (hi - lo > 1) {
        const std::size_t mid = lo + (hi - lo) / 2;
        if (pts_[mid].pwm_us <= pwm_us)
            lo = mid;
        else
            hi = mid;
    }
    const double x0 = pts_[lo].mm, x1 = pts_[hi].mm;
    const double y0 = pts_[lo].pwm_us, y1 = pts_[hi].pwm_us;
    if (!(y1 > y0)) return x0;  // pwm 非单调时退化为下界
    const double t = (pwm_us - y0) / (y1 - y0);
    return x0 + t * (x1 - x0);
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
