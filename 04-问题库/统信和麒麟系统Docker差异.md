UOS@UOS-PC:/data/home/UOS/Downloads$ docker version 
Client: Docker Engine - Community Version: 26.1.4 API version: 1.45 Go version: go1.21.11 Git commit: 5650f9b Built: Wed Jun 5 11:29:15 2024 OS/Arch: linux/amd64 Context: default Server: Docker Engine - Community Engine: Version: 26.1.4 API version: 1.45 (minimum version 1.24) Go version: go1.21.11 Git commit: de5c9cf Built: Wed Jun 5 11:29:15 2024 OS/Arch: linux/amd64 Experimental: false containerd: Version: 1.6.33 GitCommit: d2d58213f83a351ca8f528a95fbd145f5654e957 runc: Version: 1.1.12 GitCommit: v1.1.12-0-g51d5e94 docker-init: Version: 0.19.0 GitCommit: de40ad0 
UOS@UOS-PC:/data/home/UOS/Downloads$ docker info 
Client: Docker Engine - Community Version: 26.1.4 Context: default Debug Mode: false Plugins: buildx: Docker Buildx (Docker Inc.) Version: v0.14.1 Path: /usr/libexec/docker/cli-plugins/docker-buildx compose: Docker Compose (Docker Inc.) Version: v2.27.1 Path: /usr/libexec/docker/cli-plugins/docker-compose Server: Containers: 2 Running: 1 Paused: 0 Stopped: 1 Images: 3 Server Version: 26.1.4 Storage Driver: overlay2 Backing Filesystem: extfs Supports d_type: true Using metacopy: false Native Overlay Diff: true userxattr: false Logging Driver: json-file Cgroup Driver: cgroupfs Cgroup Version: 1 Plugins: Volume: local Network: bridge host ipvlan macvlan null overlay Log: awslogs fluentd gcplogs gelf journald json-file local splunk syslog Swarm: inactive Runtimes: io.containerd.runc.v2 runc Default Runtime: runc Init Binary: docker-init containerd version: d2d58213f83a351ca8f528a95fbd145f5654e957 runc version: v1.1.12-0-g51d5e94 init version: de40ad0 Security Options: seccomp Profile: builtin Kernel Version: 5.10.0-amd64-desktop Operating System: UOS Desktop 20 Professional OSType: linux Architecture: x86_64 CPUs: 16 Total Memory: 7.748GiB Name: UOS-PC ID: bef82fbb-a05d-4044-bab5-f9a678898561 Docker Root Dir: /var/lib/docker Debug Mode: false HTTP Proxy: http://192.168.1.36:7890 HTTPS Proxy: http://192.168.1.36:7890 Experimental: false Insecure Registries: 127.0.0.0/8 Registry Mirrors: https://mirrors.aliyun.com/docker-ce/ Live Restore Enabled: false



UOS@UOS-PC:/data/home/UOS/Downloads$ docker run -it --rm     -e DISPLAY=$DISPLAY     -v /tmp/.X11-unix:/tmp/.X11-unix     -v /opt/gas_control_data:/root/.local/share/GasControl     gas_control

qt.svg: Cannot open file ':/img/icon/icon.svg', because: No such file or directory

QML 加载模式: 模块 (Release)

libGL error: pci id for fd 8: 1ec8:9810, driver (null)

libGL error: MESA-LOADER: failed to open innogpu: /usr/lib/dri/innogpu_dri.so: cannot open shared object file: No such file or directory (search paths /usr/lib/x86_64-linux-gnu/dri:\$${ORIGIN}/dri:/usr/lib/dri, suffix _dri)

libGL error: failed to load driver: innogpu

libGL error: failed to open /dev/dri/card0: No such file or directory

libGL error: failed to load driver: innogpu

qt.qpa.xcb: atomName: bad atom 1026734272

qt.qpa.xcb: atomName: bad atom 1026734288

qt.qpa.xcb: atomName: bad atom 1026734528

qt.qpa.xcb: atomName: bad atom 4920186


it0@it0-pc:~/桌面$ docker version 
Client:
 Version:           26.1.3
 API version:       1.45
 Go version:        go1.22.2
 Git commit:        26.1.3-0kylin1~20.04.1+esm1k0.4
 Built:             Thu Jul 10 18:25:44 2025
 OS/Arch:           linux/amd64
 Context:           default

Server:
 Engine:
  Version:          26.1.3
  API version:      1.45 (minimum version 1.24)
  Go version:       go1.22.2
  Git commit:       26.1.3-0kylin1~20.04.1+esm1k0.4
  Built:            Thu Jul 10 18:25:44 2025
  OS/Arch:          linux/amd64
  Experimental:     false
 containerd:
  Version:          1.7.24
  GitCommit:        
 runc:
  Version:          1.1.7-0kylin1~20.04.2
  GitCommit:        
 docker-init:
  Version:          0.19.0
  GitCommit:        
it0@it0-pc:~/桌面$ ^C
it0@it0-pc:~/桌面$ docker info
Client:
 Version:    26.1.3
 Context:    default
 Debug Mode: false

Server:
 Containers: 1
  Running: 0
  Paused: 0
  Stopped: 1
 Images: 2
 Server Version: 26.1.3
 Storage Driver: overlay2
  Backing Filesystem: extfs
  Supports d_type: true
  Using metacopy: false
  Native Overlay Diff: false
  userxattr: false
 Logging Driver: json-file
 Cgroup Driver: cgroupfs
 Cgroup Version: 1
 Plugins:
  Volume: local
  Network: bridge host ipvlan macvlan null overlay
  Log: awslogs fluentd gcplogs gelf journald json-file local splunk syslog
 Swarm: inactive
 Runtimes: io.containerd.runc.v2 runc
 Default Runtime: runc
 Init Binary: docker-init
 containerd version: 
 runc version: 
 init version: 
 Security Options:
  seccomp
   Profile: builtin
 Kernel Version: 5.4.18-167-generic
 Operating System: Kylin V10 SP1
 OSType: linux
 Architecture: x86_64
 CPUs: 8
 Total Memory: 15.13GiB
 Name: it0-pc
 ID: d8cd31a0-ed11-4d34-8291-e6f1a2606bdf
 Docker Root Dir: /var/lib/docker
 Debug Mode: false
 Experimental: false
 Insecure Registries:
  127.0.0.0/8
 Registry Mirrors:
  https://docker.1panel.live/
 Live Restore Enabled: false

WARNING: No swap limit support


it0@it0-pc:~/gas_control_app$ docker run -it --rm \

>     -e DISPLAY=$DISPLAY \

>     -v /tmp/.X11-unix:/tmp/.X11-unix \

>     -v /opt/gas_control_data:/root/.local/share/GasControl \

>     gas_control

qt.svg: Cannot open file ':/img/icon/icon.svg', because: No such file or directory

QML 加载模式: 模块 (Release)

qt.qpa.xcb: atomName: bad atom 2332018880

qt.qpa.xcb: atomName: bad atom 2332018896

qt.qpa.xcb: atomName: bad atom 2332019136

qt.qpa.xcb: atomName: bad atom 2147497232


