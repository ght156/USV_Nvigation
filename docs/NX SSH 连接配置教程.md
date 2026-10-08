# NX 船端 SSH 连接配置教程（`ssh nx` 一键登录）

> 目标：在自己的开发机（Ubuntu）上配置一次，之后任意终端输入 `ssh nx`（或加别名后直接敲 `nx`）即可登录船上 NX。
> 本文实测环境：NX 主机名 `tegra-ubuntu`、局域网 IP `192.168.2.101`、登录用户 `jetson`、
> 系统 Ubuntu 22.04.5 LTS（aarch64 / Jetson）。
>
> ROS2 组网、Domain/RMW、节点联调见另一篇 [`USV 地面站NX 联网部署手册.md`](./USV%20地面站NX%20联网部署手册.md)，
> 本文只讲「怎么把 `nx` 这个 SSH 别名配好、能稳定登上去」。

---

## 1. 前置条件

- 开发机与 NX 在**同一局域网**：路由器 / 交换机 / 网线直连 / 手机热点都可以。
- 知道 NX 的 IP 或主机名。
- 知道 NX 的登录账号（本文为 `jetson`）和密码（向船端维护者索取）。

先确认网络能通：

```bash
ping -c 3 192.168.2.101
```

如果换了网络、IP 变了，用 mDNS 主机名：

```bash
ping -c 3 tegra-ubuntu.local

# 若提示无法解析，安装 mDNS 支持：
sudo apt install -y avahi-daemon libnss-mdns
```

---

## 2. 配置 `~/.ssh/config`（核心步骤）

创建目录并打开配置：

```bash
mkdir -p ~/.ssh && chmod 700 ~/.ssh
nano ~/.ssh/config
```

写入下面内容（如果文件里已有其它 `Host`，直接追加即可）：

```sshconfig
Host nx
    HostName 192.168.2.101
    User jetson
    ForwardAgent yes
    StrictHostKeyChecking accept-new
```

保存后设置权限（`ssh` 对 config 权限敏感）：

```bash
chmod 600 ~/.ssh/config
```

字段说明：

| 字段 | 作用 |
|------|------|
| `Host nx` | 别名。之后 `ssh nx` 等价于 `ssh jetson@192.168.2.101` |
| `HostName` | NX 的真实地址，可填 IP 或 `tegra-ubuntu.local` |
| `User` | 登录 NX 的用户名，当前为 `jetson` |
| `ForwardAgent yes` | 把本机 ssh-agent 转发到 NX，方便在 NX 上直接拉 GitLab 代码（可选） |
| `StrictHostKeyChecking accept-new` | 首次连接自动接受主机指纹，避免反复询问 |

> **两种用法，别混淆：**
> - `ssh nx`：配完本节 `~/.ssh/config` 就能用，**不需要**动 `.bashrc`。
> - 直接敲 `nx`（不带 `ssh`）：`.ssh/config` 管不到，**必须**再加一个 shell 别名，
>   见 [§4.4](#44-让裸敲-nx-生效bashrc-别名)。

---

## 3. 首次连接（密码登录）

```bash
ssh nx
```

- 第一次会提示 `Are you sure you want to continue connecting (yes/no)?`，输入 `yes`。
- 接着输入 `jetson` 用户的密码（输入时不显示）。
- 看到 `jetson@tegra-ubuntu:~$` 即登录成功。

非交互式验证一条命令：

```bash
ssh nx 'hostname; hostname -I; whoami'

# 期望输出类似：
# tegra-ubuntu
# 192.168.2.101 ......
# jetson
```

---

## 4. 配置免密登录（推荐）

每次输密码很麻烦，建议配置公钥登录。**只需做一次。**

### 4.1 生成密钥（已有可跳过）

```bash
ssh-keygen -t ed25519 -C "你的名字@dev"
# 一路回车，默认生成 ~/.ssh/id_ed25519 与 ~/.ssh/id_ed25519.pub
```

如果本机已有 `~/.ssh/id_ed25519.pub`，这一步跳过。

### 4.2 把公钥拷到 NX

```bash
ssh-copy-id nx
# 输入一次 jetson 密码
```

### 4.3 验证

```bash
ssh nx 'echo 免密成功'
```

不再要求密码即配置完成。

### 4.4 让裸敲 `nx` 生效（bashrc 别名）

`ssh nx` 依赖的是 `~/.ssh/config` 的 `Host nx`；而**直接输入 `nx`** 是 shell 行为，
SSH 配置管不到，必须在 shell 启动文件里加别名：

```bash
echo "alias nx='ssh nx'" >> ~/.bashrc
source ~/.bashrc
```

验证：

```bash
type nx          # 期望输出：nx is aliased to `ssh nx'
nx               # 直接登录 NX
```

注意：

- 别名只在**交互式 bash** 里生效。脚本（`bash script.sh`）、非交互 `ssh nx 'cmd'` 里
  不会展开别名，脚本里请写完整的 `ssh nx`。
- 新开终端会自动加载；当前终端要执行一次 `source ~/.bashrc`。
- 如果希望所有 shell（zsh 等）都生效，zsh 用户写到 `~/.zshrc`，或改用
  `~/.local/bin/nx` 可执行脚本 / `~/.ssh/config` 的 `Match` 规则。

---

## 5. 多网络场景（可选，推荐保留两个别名）

NX 在固定局域网和手机热点下 IP 会不同，建议同时配置有线/无线两个别名：

```sshconfig
Host nx              # 固定局域网 / 网线直连（当前使用）
    HostName 192.168.2.101
    User jetson
    ForwardAgent yes
    StrictHostKeyChecking accept-new

Host nx-wired        # 同上，语义更明确的别名
    HostName 192.168.2.101
    User jetson
    ForwardAgent yes
    StrictHostKeyChecking accept-new

Host nx-wifi         # 手机热点 / 动态 IP，靠 mDNS 主机名
    HostName tegra-ubuntu.local
    User jetson
    ForwardAgent yes
    StrictHostKeyChecking accept-new
```

用法：

- 有线或同一路由器：`ssh nx`
- 手机热点、IP 不定：`ssh nx-wifi`

---

## 6. 日常使用命令

```bash
nx                              # 登录 NX（已配 §4.4 别名）
ssh nx                          # 等价写法，任何 shell/脚本里都能用
ssh nx 'ls -la ~/mavros_ws'     # 远程执行一条命令，不进交互
scp file.txt nx:~/              # 上传文件到 NX 家目录
scp nx:~/mavros_ws/log/x.log .  # 从 NX 下载文件
rsync -av ./dir/ nx:~/dir/      # 增量同步目录（推荐大目录）
```

---

## 7. 故障排查

| 现象 | 常见原因 | 处理办法 |
|------|----------|----------|
| `connect to host ... port 22: Connection timed out` | 网络不通 / IP 写错 / 不在同一网段 | `ping` 确认；改用 `ssh nx-wifi`；在路由器后台或用 `ip neigh` 查 NX 新 IP |
| `Connection refused` | NX 未开机，或 sshd 未运行 | 上船检查；`sudo systemctl status ssh` 应为 running |
| `Permission denied (publickey)` | 免密没配好 / 公钥没拷上 | 先用密码登录：`ssh -o PreferredAuthentications=password nx`，再重跑 `ssh-copy-id nx` |
| `WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED` | NX 重装或换了机器，主机指纹变了 | 确认无误后执行 `ssh-keygen -R 192.168.2.101`，再重新连接 |
| `Could not resolve hostname tegra-ubuntu.local` | mDNS 不可用 | `sudo apt install -y avahi-daemon libnss-mdns`，或直接用 IP |
| 能登录但 ROS2 看不到船端节点 | `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` 不一致 | 见部署手册；当前实船常用 `ROS_DOMAIN_ID=5` + `rmw_cyclonedds_cpp` |

---

## 8. 一键配置脚本（可选）

把下面内容保存为 `setup_nx_ssh.sh`，执行一次即可完成「写 config + 生成密钥 + 免密 + 验证」：

```bash
#!/usr/bin/env bash
set -euo pipefail

NX_HOST="${NX_HOST:-192.168.2.101}"
NX_USER="${NX_USER:-jetson}"

mkdir -p ~/.ssh && chmod 700 ~/.ssh
touch ~/.ssh/config && chmod 600 ~/.ssh/config

if ! grep -q "^Host nx$" ~/.ssh/config; then
  cat >> ~/.ssh/config <<EOF

Host nx
    HostName ${NX_HOST}
    User ${NX_USER}
    ForwardAgent yes
    StrictHostKeyChecking accept-new
EOF
fi

[ -f ~/.ssh/id_ed25519 ] || ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519
ssh-copy-id -i ~/.ssh/id_ed25519.pub nx

if ! grep -q "alias nx='ssh nx'" ~/.bashrc; then
  echo "alias nx='ssh nx'" >> ~/.bashrc
fi

echo "=== 验证 ==="
ssh nx 'hostname; hostname -I; whoami'
echo "已写入 ~/.bashrc 别名；当前终端执行 source ~/.bashrc 后可直接敲 nx"
```

执行：

```bash
chmod +x setup_nx_ssh.sh
./setup_nx_ssh.sh
```

> 换网络时可用环境变量覆盖：`NX_HOST=tegra-ubuntu.local ./setup_nx_ssh.sh`

---

## 附：当前实测信息

| 项目 | 值 |
|------|----|
| NX 主机名 | `tegra-ubuntu` |
| 局域网 IP | `192.168.2.101` |
| 登录用户 | `jetson` |
| 系统 | Ubuntu 22.04.5 LTS / aarch64（Jetson） |
| 常用工作区 | `~/mavros_ws`（含 `src/`、`install/`、`build/`、`log/`） |

> 若船端更换了 NX 或网络环境，请同步更新 `HostName` 与本节表格。
