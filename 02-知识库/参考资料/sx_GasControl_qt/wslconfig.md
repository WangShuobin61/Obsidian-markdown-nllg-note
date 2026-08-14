[wsl2]

# 限制 WSL2 虚拟机可使用的最大内存
# 这里设置为 24GB，超过该值 WSL 不会再向 Windows 申请内存
memory=24GB

# 限制 WSL2 可使用的 CPU 核心数
# 这里最多使用 10 个逻辑处理器
processors=10

# 设置 WSL2 使用的交换分区（Swap）大小
# 当内存不足时用于临时换出内存数据
swap=16GB

# WSL2 的网络模式
# mirrored 表示与 Windows 主机共享同一网络（类似桥接），不再经过 NAT
# 必须开启：默认 NAT 模式下，IP 摄像头(192.168.1.x)用 UDP 发回的 RTP 视频流
#          会因为没有入站映射被丢弃，导致画面拉不出来一直 Error；
#          mirrored 让 WSL 直接复用 Windows 网络栈，UDP 回程才能进来
# networkingMode=mirrored

# 是否将 WSL2 的端口自动转发到 Windows 的 localhost 默认为true
# localhostForwarding=true

# 是否允许 WSL2 内部再运行虚拟化（如 Docker-in-Docker、KVM）
# 开启后可在 WSL 中使用嵌套虚拟化能力
# 无需开启：已默认支持 nestedVirtualization=true 

# 启用 DNS 隧道模式
# 可解决某些 VPN / 企业网络下 DNS 解析异常的问题
# dnsTunneling=true

# 是否自动继承 Windows 的代理设置（版本不稳定）
# 对需要走系统代理的网络环境（公司、校园网）非常有用
# autoProxy=true