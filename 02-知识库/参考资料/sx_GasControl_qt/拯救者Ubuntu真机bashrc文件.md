```Shell
# ~/.bashrc：由 bash(1) 在非登录 shell 启动时执行
# 参考 /usr/share/doc/bash/examples/startup-files（位于 bash-doc 软件包中）
# 可查看官方示例配置

# 如果不是交互式 shell，则不执行后续内容
case $- in
    *i*) ;;
      *) return;;
esac

# 不在历史记录中保存重复命令或以空格开头的命令
# 更多选项见 bash(1) 手册
HISTCONTROL=ignoreboth

# 历史记录追加到文件，而不是覆盖原文件
shopt -s histappend

# 历史记录长度设置（详见 bash 手册中的 HISTSIZE 与 HISTFILESIZE）
HISTSIZE=1000
HISTFILESIZE=2000

# 每次命令执行后检查窗口大小，并在需要时更新 LINES 和 COLUMNS
shopt -s checkwinsize

# 如果启用，路径扩展中的 "**" 可以匹配所有文件及多级子目录
#shopt -s globstar

# 让 less 在处理非文本文件时更友好，参考 lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# 设置当前 chroot 环境变量（用于下面的提示符显示）
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# 设置一个较美观的提示符（默认无颜色，除非终端支持颜色）
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# 如果终端支持颜色，可以取消注释启用彩色提示符
# 默认关闭，以免分散注意力
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
        # 终端支持颜色，假定符合 Ecma-48（ISO/IEC-6429）标准
        # 不支持的情况极为罕见
        color_prompt=yes
    else
        color_prompt=
    fi
fi

# ===== 自定义终端前缀为 nllg@PC:当前目录$ =====
# \u = 用户名
# \w = 当前目录
if [ "$color_prompt" = yes ]; then
    PS1='\[\033[01;32m\]\u@LegionR7000\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='\u@LegionR7000:\w\$ '
fi
unset color_prompt force_color_prompt

# 如果是 xterm 终端，则设置窗口标题为 user@PC:目录
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;\u@PC: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# 启用 ls 的彩色显示，并添加常用别名
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# GCC 编译警告与错误的彩色显示（默认注释）
export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# 一些常用的 ls 别名
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'
alias LL='ls -alF'
alias LS='ls -CF'
alias S='source ~/.bashrc'
alias s='source ~/.bashrc'
alias qt='/opt/Qt/Tools/QtCreator/bin/qtcreator'
alias QT='/opt/Qt/Tools/QtCreator/bin/qtcreator'
alias qt6='/opt/Qt/Tools/QtCreator/bin/qtcreator'
alias QT6='/opt/Qt/Tools/QtCreator/bin/qtcreator'
alias CD='cd'
alias '才'='cd'
alias '龙'='ls'

# 为长时间运行的命令添加完成提醒
# 使用方法： sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# 别名定义区域
# 你也可以把自定义别名放到 ~/.bash_aliases 文件中，而不是直接写在这里
# 参考 /usr/share/doc/bash-doc/examples

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# 启用 bash 自动补全功能
# 如果 /etc/bash.bashrc 已经启用，这里其实可以不用再启用
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

# >>> fishros 初始化（ROS2 环境） >>>
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
# <<< fishros 初始化结束 <<<

# >>> Node.js 环境初始化 >>>
export PATH=$PATH:/opt/nodejs/node-v18.12.1-linux-x64/bin/
# <<< Node.js 初始化结束 <<<

# >>> Qt 初始化 >>> ($PATH在后新设置优先，$PATH在前已有设置优先) 
export PATH=$PATH:/opt/Qt/Tools/QtCreator/bin
export PATH=/opt/Qt/Tools/CMake/bin:$PATH
export PATH=/opt/Qt/Tools/Ninja:$PATH
# <<< Qt 初始化结束 <<<
```