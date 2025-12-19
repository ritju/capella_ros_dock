#!/bin/bash

#使用Shell脚本处理YAML
generate_env_from_yaml() {
    local yaml_file=$1
    local prefix=${2:-DOCK_}
    
    # 使用yq工具（需要安装yq）
    if command -v yq &> /dev/null; then
        # 获取所有参数键值对
        yq eval '.. | select(tag != "!!map") | (path | join("_") | ascii_upcase) + "=" + (tostring)' "$yaml_file" | \
        while read -r line; do
            if [[ $line == *"="* ]]; then
                # 提取键和值
                key="${line%%=*}"
                value="${line#*=}"
                echo "export ${prefix}${key}=\"$value\""
            fi
        done
    else
        echo "错误: 需要安装yq工具 (pip install yq)"
        return 1
    fi
}

# 使用方法
# generate_env_from_yaml config.yaml > env_vars.sh
# source env_vars.sh