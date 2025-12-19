#!/usr/bin/env python3
import yaml
import sys
import os

"""
用于生成{'param_name': '', 'evn_name': '', 'default_value': '', 'current_value': ''}的列表
参数
yaml_file  : 参数文件的地址
output_file: 输入文件的名字
prefix     : 环境变量的前缀，适配docker-compose.yaml文件
返回值
返回{'param_name': '', 'evn_name': '', 'default_value': ''}的列表
"""
def generate_env_vars_from_yaml(yaml_file, output_file=None, prefix="- DOCK_"):
    # 读取YAML文件
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    
    # 查找节点参数（通常位于ros__parameters下）
    params = {}

    if isinstance(data, dict):
        # 如果没有ros__parameters，尝试直接读取参数
        params = data
    else:
        print("无法找到参数部分")
        return
    
    env_lines = []
    param_lines = []
   
    # 用于提取参数文件中所有完整路径的parameters
    def process_params2(param_dict, param_name = None, surffix="."):
        for key, value in param_dict.items():
            if isinstance(value, dict):
                if param_name == None:
                    param_name = f"{key}"
                else:
                    param_name = f"{param_name}{surffix}{key}"
                process_params2(value, param_name, surffix)
            else:
                if param_name != None:
                    param_lines.append(f"{param_name}{surffix}{key}")
                else:
                    param_lines.append(f"{key}")
  
    # 用于生成docker-compose.yaml中使用的完整的环境变量
    def process_params(prefix_path, param_dict):        
        for key, value in param_dict.items():
            if key == "ros__parameters":
                omit=True
                full_key = f"{prefix_path}"
                process_params2(value)
            else:
                omit=False
                full_key = f"{prefix_path}{key.upper()}"
            if isinstance(value, dict):
                # 处理嵌套参数
                if omit:
                    process_params(f"{full_key}", value)
                else:
                    process_params(f"{full_key}_", value)
            else:
                # 生成环境变量行
                env_lines.append(f"{full_key}=\"{value}\"")
    
    process_params(prefix, params)

    keys, values = zip(*[(k.strip(), v.strip().strip('"\'')) 
                        for k, v in (item.split('=', 1) for item in env_lines)])
    keys, values = list(keys), list(values)    

    result = [
        {'param_name': param, 'env_name': env.split(' ', 1)[1], 'default_value': value, 'current_value': value}
         for param, env, value in zip(param_lines, keys, values)
         ]
    
    # for i in result:
    #     print(i)
    
    # 输出到文件或控制台
    if output_file:
        if output_file.startswith('/'):
            output_file = output_file
        else:
            if yaml_file.startswith('/'):
                directory = os.path.dirname(yaml_file)
                output_file = os.path.join(directory, output_file)
            else:
                output_file = output_file
        with open(output_file, 'w') as f:
            f.write("\n".join(env_lines))
        print(f"已生成环境变量文件: {output_file}")
    else:
        print("\n".join(env_lines))   
    
    return result

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python script.py <config.yaml> [output_file]")
        sys.exit(1)
    
    yaml_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None
    result = generate_env_vars_from_yaml(yaml_file, output_file)
    
    
            
