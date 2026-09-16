#!/bin/bash

# Git仓库管理脚本
# 功能：自动检测仓库状态，如果未初始化则进行初始化设置，如果已初始化则执行同步操作
# 特点：支持为不同仓库（公司Gitea和个人Gitee）设置和使用不同的用户名和邮箱

# 颜色定义
GREEN="\033[32m"
YELLOW="\033[33m"
BLUE="\033[34m"
RED="\033[31m"
NC="\033[0m"

# 提示：请将以下URL替换为您实际的Gitea服务URL
TEAM_REPO_URL="http://192.168.1.33:3000/SX_Project/sx_GasControl.git"

# 检查Git是否已安装
echo -e "${BLUE}检查Git是否已安装...${NC}"
git --version > /dev/null 2>&1
if [ $? -ne 0 ]; then
    echo -e "${RED}错误：未安装Git！请先安装Git后再运行此脚本。${NC}"
    exit 1
fi

# 检查是否已有Git仓库，根据结果执行不同操作
if [ ! -d .git ]; then
    # 仓库未初始化，执行初始化设置
    echo -e "\n${BLUE}仓库初始化设置开始...${NC}"
    
    # 初始化Git仓库
    echo -e "${YELLOW}当前目录不是Git仓库，正在初始化...${NC}"
    git init
    if [ $? -ne 0 ]; then
        echo -e "${RED}错误：Git仓库初始化失败！${NC}"
        exit 1
    fi
    echo -e "${GREEN}Git仓库初始化成功！${NC}"
    
    # 创建.gitignore文件
    echo -e "\n${BLUE}检查是否存在.gitignore文件...${NC}"
    if [ ! -f .gitignore ]; then
        echo -e "${YELLOW}.gitignore文件不存在，创建基本的.gitignore文件...${NC}"
        cat > .gitignore << 'EOF'
# IDE和编辑器文件
.vscode/
.idea/
*.swp
*.swo
*~

# 编译产物
/build/
/dist/
/bin/
/obj/

# 临时文件和缓存
*.tmp
*.temp
.cache/
*.log

# 操作系统文件
.DS_Store
Thumbs.db

# 环境变量文件
.env
.env.local
.env.development.local
.env.test.local
.env.production.local

# 依赖目录
/node_modules/
/vendor/
EOF
        echo -e "${GREEN}.gitignore文件创建成功！${NC}"
    else
        echo -e "${GREEN}.gitignore文件已存在，跳过创建步骤。${NC}"
    fi
    
    # 创建README.md文件
    echo -e "\n${BLUE}检查是否存在README.md文件...${NC}"
    if [ ! -f README.md ]; then
        echo -e "${YELLOW}README.md文件不存在，创建基本的README.md文件...${NC}"
        echo "请填写内容" > README.md
        echo -e "${GREEN}README.md文件创建成功！${NC}"
    else
        echo -e "${GREEN}README.md文件已存在，跳过创建步骤。${NC}"
    fi
    
    # 配置用户名和邮箱
    echo -e "\n${BLUE}配置Git用户名和邮箱...${NC}"
    
    # 设置本地全局用户名和邮箱（作为全局默认值，两个仓库输入任意一个即可）
    echo -e "${YELLOW}设置全局默认用户名和邮箱...${NC}"
    git config --global user.name "WangShuobin"
    git config --global user.email "wsb@vip.qq.com"
    echo -e "${GREEN}Git全局配置已更新！${NC}"
    
    # 为当前仓库设置特定的用户名和邮箱（覆盖全局配置）
    echo -e "\n${YELLOW}为当前仓库设置特定的用户名和邮箱...${NC}"
    echo -n -e "${YELLOW}请输入用于公司Gitea仓库的用户名（默认为全局设置）: ${NC}"
    read LOCAL_USERNAME
    if [ -n "$LOCAL_USERNAME" ]; then
        git config user.name "$LOCAL_USERNAME"
        echo -e "${GREEN}仓库用户名已设置为：$LOCAL_USERNAME${NC}"
    else
        echo -e "${GREEN}使用全局用户名：$(git config --global user.name)${NC}"
    fi
    
    echo -n -e "${YELLOW}请输入用于公司Gitea仓库的邮箱（默认为全局设置）: ${NC}"
    read LOCAL_EMAIL
    if [ -n "$LOCAL_EMAIL" ]; then
        git config user.email "$LOCAL_EMAIL"
        echo -e "${GREEN}仓库邮箱已设置为：$LOCAL_EMAIL${NC}"
    else
        echo -e "${GREEN}使用全局邮箱：$(git config --global user.email)${NC}"
    fi
    
    # 配置Git凭证管理，避免每次操作都输入用户名和密码
    echo -e "\n${BLUE}配置Git凭证管理...${NC}"
    echo -e "${GREEN}配置永久保存凭证...${NC}"
    git config --global credential.helper store
    echo -e "${GREEN}Git凭证已配置为永久保存模式！${NC}"
    echo -e "${YELLOW}提示：凭证将明文保存在 ~/.git-credentials 文件中。${NC}"
    echo -e "${YELLOW}      Git会根据仓库URL自动匹配不同的凭证，支持不同仓库使用不同的用户名和密码。${NC}"
    echo -e "${YELLOW}      首次操作两个不同仓库时需要分别输入一次用户名和密码，之后无需重复输入。${NC}"
    
    # 配置远程仓库
    echo -e "\n${BLUE}配置远程团队仓库...${NC}"
    echo -e "${YELLOW}添加团队远程仓库：$TEAM_REPO_URL${NC}"
    git remote add gitea $TEAM_REPO_URL
    if [ $? -ne 0 ]; then
        echo -e "${RED}错误：添加远程仓库失败！${NC}"
        exit 1
    fi
    echo -e "${GREEN}远程仓库添加成功！${NC}"
    
    # 验证远程仓库配置
    echo -e "\n${BLUE}当前远程仓库配置：${NC}"
    git remote -v
    
    echo -e "\n${GREEN}团队Gitea仓库配置完成！${NC}"
    echo -e "\n${BLUE}以下是使用指南：${NC}"
    echo -e "\n${YELLOW}1. 首次推送代码到团队仓库：${NC}"
    echo "   git push -u gitea master"
    echo -e "\n${YELLOW}2. 日常开发后推送到团队仓库：${NC}"
    echo "   git push gitea master"
    echo -e "\n${YELLOW}3. 再次运行此脚本可进行仓库同步：${NC}"
    echo "   bash git_manage.sh"
    echo -e "\n${YELLOW}4. 从远程仓库拉取更新：${NC}"
    echo "   git pull gitea master"
    
    echo -e "\n${YELLOW}提示：如果需要修改仓库URL，请编辑此脚本中的TEAM_REPO_URL变量！${NC}"
    echo -e "${YELLOW}      当前配置的仓库URL：${TEAM_REPO_URL}${NC}"
    
    # 凭证管理说明
    echo -e "\n${BLUE}凭证管理说明：${NC}"
    echo -e "${YELLOW}1. 当前配置：${NC}"
    echo "   已配置为永久保存凭证模式，凭证将明文保存在 ~/.git-credentials 文件中"
    echo -e "${YELLOW}2. 双仓库支持：${NC}"
    echo "   Git会根据仓库URL自动匹配不同的凭证，支持不同仓库使用不同的用户名和密码"
    echo -e "${YELLOW}3. 首次使用流程：${NC}"
    echo "   - 首次推送到Gitea仓库时，输入Gitea的用户名和密码"
    echo "   - 首次推送到Gitee仓库时，输入Gitee的用户名和密码"
    echo "   - Git会自动保存这些凭证，以后操作无需重复输入"
    echo -e "${YELLOW}4. 如需修改凭证配置：${NC}"
    echo "   运行 'git config --global credential.helper' 查看当前配置"
    echo "   运行 'git config --global --unset credential.helper' 清除当前配置"
    echo "   然后重新运行此脚本设置新的凭证方式"
    
    echo -e "\n${GREEN}初始化完成！祝您使用愉快！${NC}"
else
    # 仓库已初始化，执行同步操作
    echo -e "\n${BLUE}仓库同步操作开始...${NC}"
    
    # 添加所有修改的文件
    echo "添加所有修改的文件..."
    git add .
    if [ $? -ne 0 ]; then
        echo "警告：添加文件失败！"
    fi
    
    # 提交更改
    echo "提交更改..."
    git commit -m "一键提交.sh"
    if [ $? -ne 0 ]; then
        echo "警告：提交失败！可能没有需要提交的更改。"
    fi
    
    # 推送到公司内部gitea仓库
    echo "推送到公司内部gitea仓库..."
    # 公司仓库使用本地配置的身份（已在初始化时设置）
    git push gitea master
    if [ $? -ne 0 ]; then
        echo "警告：推送到公司内部gitea仓库失败！"
    fi
    
    # 推送到gitee私人仓库
    echo "推送到gitee私人仓库..."
    git push gitee master
    if [ $? -ne 0 ]; then
        echo "警告：推送到gitee私人仓库失败！"
        echo "注意：可能需要先在gitee创建同名仓库，或者检查网络连接和权限设置。"
    fi
    
    echo -e "\n${GREEN}双仓同步完成！${NC}"
fi
