import os

"""
用于获取环境变量值
参数
env: 环境变量名称
default: 环境变量为赋值时使用的默认值
返回值
返回最终采用的值
"""
def get_environment_value(env, default):
    try:
        if env in os.environ:
            value = os.environ.get(env, default)
            print(f'get {env} value: {value} from environment')
            return value
        else:
            print(f"Using default {env} value: {default}.")
            return default
    except Exception as e:
        print(f'exception: {str(e)}')
        print(f"Please input {env} in environment")
        return default
    


