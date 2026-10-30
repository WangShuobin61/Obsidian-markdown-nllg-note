#!/usr/bin/env bash
set -euo pipefail

if ! command -v docker >/dev/null 2>&1; then
    printf '未找到 docker 命令。\n' >&2
    exit 1
fi

# 仅允许当前机器的标准 Unix socket，避免误操作远程 Docker 主机。
context_endpoint=$(docker context inspect --format '{{.Endpoints.docker.Host}}' "$(docker context show)")
for endpoint in "$context_endpoint" "${DOCKER_HOST:-$context_endpoint}"; do
    case "$endpoint" in
        unix:///var/run/docker.sock|unix:///run/docker.sock) ;;
        *) printf '检测到 Docker 端点 %s；脚本只支持本机标准 socket。\n' "$endpoint" >&2; exit 1 ;;
    esac
done

if ! server_version=$(docker version --format '{{.Server.Version}}' 2>&1); then
    printf '无法连接本机 Docker daemon：%s\n' "$server_version" >&2
    exit 1
fi
security=$(docker info --format '{{json .SecurityOptions}}' 2>&1)
case "$security" in
    *name=rootless*|*name=userns*)
        printf '检测到 rootless 或 userns：容器 root 未必对应宿主机 root，停止。\n' >&2
        exit 1 ;;
esac

if ! docker image inspect ubuntu:16.04 >/dev/null 2>&1; then
    printf '缺少 ubuntu:16.04 镜像。请先手动执行 docker pull ubuntu:16.04。\n' >&2
    exit 1
fi

printf '即将把宿主机 / 以可写方式挂载到临时容器，并尝试进入宿主机 /bin/bash。\n'
printf '此操作将获得对宿主机文件系统的完整修改权限。输入 ROOT 后继续：'
read -r answer
if [ "$answer" != ROOT ]; then
    printf '已取消。\n'
    exit 0
fi

docker run --rm -it --network none --pull=never --user 0 \
    --mount type=bind,src=/,dst=/host \
    --entrypoint /usr/sbin/chroot ubuntu:16.04 /host /bin/bash
