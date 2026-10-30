#!/usr/bin/env bash
set -u

printf '=== 系统与当前用户 ===\n'
grep -E '^(MajorVersion|MinorVersion|EditionName)=' /etc/os-version 2>/dev/null || true
id
printf '\n=== sudo 状态 ===\n'
stat -c '%n 属主UID=%u 属组GID=%g 权限=%a' /usr/bin/sudo 2>/dev/null || true
sudo -n true 2>&1
sudo_status=$?
if [ "$sudo_status" -eq 0 ]; then
    printf 'sudo -n true: 成功\n'
else
    printf 'sudo -n true: 失败（退出码 %s；-n 不会请求密码）\n' "$sudo_status"
fi

printf '\n=== Docker 连接 ===\n'
if ! command -v docker >/dev/null 2>&1; then
    printf '未找到 docker 命令。\n'
    exit 0
fi
printf '当前上下文：'
docker context show 2>&1 || true
printf 'DOCKER_HOST=%s\n' "${DOCKER_HOST:-<未设置>}"
docker context inspect --format '上下文端点：{{.Endpoints.docker.Host}}' 2>&1 || true
if ! server_version=$(docker version --format '{{.Server.Version}}' 2>&1); then
    printf '无法连接 Docker daemon：%s\n' "$server_version"
    exit 0
fi
printf '服务端版本：%s\n' "$server_version"
docker_info=$(docker info --format '安全选项={{json .SecurityOptions}}' 2>&1)
printf '%s\n' "$docker_info"

printf '\n=== 已缓存镜像的容器 root 检查 ===\n'
found=0
for img in ubuntu:16.04 busybox:latest alpine:latest debian:latest ubuntu:latest; do
    if docker image inspect "$img" >/dev/null 2>&1; then
        printf '使用镜像 %s 执行 id：\n' "$img"
        docker run --rm --network none --pull=never --user 0 --entrypoint id "$img" 2>&1 || true
        found=1
        break
    fi
done
if [ "$found" -eq 0 ]; then
    printf '没有找到上述已缓存镜像；可在有网络时手动执行 docker pull ubuntu:16.04 后重试。\n'
fi

printf '\n容器内 uid=0 只表示容器 root；是否能管理宿主机取决于 daemon、用户命名空间和挂载权限。\n'
