# 隐私与脱敏策略

本文件定义 `qi-neo` 账号 5 个公开仓库的脱敏边界。此前「论文 PDF 已脱敏」只是一句口头承诺，
项目仓与展示仓的边界没有统一定义；这里把它写成可执行、可在 CI 里校验的规则。

## 一、两条边界

| 仓库类型 | 仓库 | 允许出现 | 禁止出现 |
|---|---|---|---|
| **展示仓**（求职门面） | `qi-neo`（profile README）、`qi-neo.github.io`（主页） | 真实姓名、公开联系邮箱、学校/届别、获奖 | 学号、真实 Wi-Fi SSID、任何口令/密钥 |
| **项目仓**（代码、论文、硬件、固件、日志） | `active-suspension-lidar-preview`、`unilidar-driver`、`maixcam-esp32-tcp-stepper-motor` | 占位符、脱敏后的实验数据 | 学号、个人邮箱、真实 SSID、真实密码/token、密钥、`secrets.h` |

一句话：**展示仓保留姓名 + 邮箱用于联系；项目仓不出现任何可定位到个人或真实网络/凭据的字面量。**

## 二、禁止字面量清单

由 `tools/check_privacy.py`（本仓）与 maixcam 的 `tools/check_no_secrets.py` 在 CI 中门住：

| 类别 | 字面量 | 说明 |
|---|---|---|
| 学号 | `182311123` | 任何形式都不得进入项目仓 |
| 真实 SSID | `205-03` | 早期 CSDN 迁移文档/`.ino` 残留；已改为 `<YOUR_WIFI_SSID>` / `secrets.h` |
| 个人邮箱 | `1794194885@qq.com` | 仅展示仓白名单允许 |
| 真实口令 | Wi-Fi 密码、AUTH_TOKEN 明文 | 一律 `secrets.h` / 环境变量，仓库只留 `secrets.example.h` |
| 本地凭据文件 | `secrets.h`、`.env` | 必须被 `.gitignore` 忽略，不得提交 |

> **例外（白名单）**：上述字面量允许出现在**策略与校验文件自身**
> （`docs/PRIVACY.md`、`tools/check_privacy.py`、`tools/check_no_secrets.py`），
> 因为它们正是用来检测这些串的；也允许出现在展示仓的联系方式段落。

## 三、校验与防护

1. `python3 tools/check_privacy.py`（本仓，零第三方依赖，接入 `ros-ci.yml` 零耗时阶段）：
   扫描项目仓全部文本文件，命中禁止串即失败并列出文件与行号。
2. `python3 tools/check_no_secrets.py`（maixcam）：额外校验固件 `#include "secrets.h"`、
   服务端 token 来自环境变量、`secrets.h` 被忽略。
3. **GitHub Secret Scanning**：5 个公开仓库全部开启（public repo 免费），
   推送时自动拦截常见 token 形态；仓库 Settings → Security 可查看。
4. **gitleaks**：各仓 CI 可选步骤扫描历史提交中的凭据；本轮以 1/2/3 为主，
   gitleaks 作为后续增强，不阻塞。

## 四、验收

```bash
# 在项目仓（排除展示仓邮箱）中不应有命中
grep -RIn "182311123\|205-03\|1794194885" \
  --exclude-dir=.git --exclude=PRIVACY.md --exclude=check_privacy.py \
  --exclude=check_no_secrets.py .
# 展示仓邮箱属白名单，允许出现在 profile README 与主页的联系段落
```

## 五、历史处置记录

- 论文 PDF 早期已脱敏；文字版 `docs/paper/*.md` 保留 OCR 原貌，不含个人信息。
- `esp32_tcp_stepper.ino` 的 `WIFI_SSID` / 密码 / token 已外置到 `secrets.h`（P0-6）。
- maixcam `README.md` 里 CSDN 迁移示例残留的真实 SSID 已在 P2 改为占位符。
