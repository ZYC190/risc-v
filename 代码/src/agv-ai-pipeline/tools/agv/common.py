import threading
import time
import math
import difflib
    
def execute_command(user_input, function_dict_zh):
    valid_commands = list(function_dict_zh.keys())
    
    # 使用 difflib 进行模糊匹配
    match = difflib.get_close_matches(user_input, valid_commands, n=1, cutoff=0.6)
    
    if match:
        try:
            print(f"识别到指令: {match[0]}")
            func_name = function_dict_zh[match[0]]
            
            return True, func_name
        except:
            return False, None
    else:
        return False, None
        print("未知指令，请重试！")