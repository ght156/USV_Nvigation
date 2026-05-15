#!/bin/bash
# setup_nx_ssh.sh —— 配置免密 SSH，并自动在 /etc/hosts 写入 NX 主机名
# 只需运行一次，之后 ssh/rsync 无需密码、不依赖 IP

set -e

NX_USER="${NX_USER:-nvidia}"
NX_HOST="${NX_HOST:-nvidia-desktop}"
NX_IP="${NX_IP:-192.168.1.2}"

echo "=== 配置免密 SSH: ${NX_USER}@${NX_IP} (主机名 ${NX_HOST}) ==="

# 生成密钥（如无）
if [ ! -f ~/.ssh/id_rsa.pub ]; then
  echo "生成 SSH 密钥对..."
  ssh-keygen -t rsa -b 4096 -f ~/.ssh/id_rsa -N "" -q
fi

# 复制公钥
echo "将公钥复制到 NX（输入密码: nvidia）..."
ssh-copy-id -o StrictHostKeyChecking=no ${NX_USER}@${NX_IP}

# 写入 /etc/hosts，让 ${NX_HOST}.local 可解析
HOSTS_LINE="${NX_IP}  ${NX_HOST}  ${NX_HOST}.local"
if grep -q "${NX_HOST}" /etc/hosts 2>/dev/null; then
  echo "/etc/hosts 已有 ${NX_HOST} 条目，跳过"
else
  echo "写入 /etc/hosts（需 sudo）: ${HOSTS_LINE}"
  echo "${HOSTS_LINE}" | sudo tee -a /etc/hosts > /dev/null
fi

# 验证
echo "=== 验证免密连接 ==="
ssh ${NX_USER}@${NX_IP} "echo '免密 OK; 主机名='\$(hostname); uname -a"
