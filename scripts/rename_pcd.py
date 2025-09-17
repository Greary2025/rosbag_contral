#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import glob

def rename_pcd_files(input_dir):
    """
    批量重命名PCD文件，将5位数字改为6位数字格式
    
    Args:
        input_dir (str): 包含PCD文件的目录路径
    """
    # 确保输入路径存在
    if not os.path.exists(input_dir):
        print(f"错误：目录不存在: {input_dir}")
        return False
    
    try:
        # 获取目录中所有的.pcd文件
        pcd_files = glob.glob(os.path.join(input_dir, "*.pcd"))
        
        if not pcd_files:
            print(f"错误：在目录 {input_dir} 中未找到PCD文件")
            return False
        
        print(f"找到 {len(pcd_files)} 个PCD文件")
        print("开始重命名...")
        
        # 重命名计数器
        renamed_count = 0
        
        # 遍历所有PCD文件
        for pcd_file in sorted(pcd_files):
            # 获取文件名和扩展名
            dir_path = os.path.dirname(pcd_file)
            filename = os.path.basename(pcd_file)
            name, ext = os.path.splitext(filename)
            
            # 检查是否是5位数字格式
            if name.isdigit() and len(name) == 5:
                # 创建新的6位数字格式文件名
                new_name = f"{int(name):06d}{ext}"
                new_path = os.path.join(dir_path, new_name)
                
                # 重命名文件
                os.rename(pcd_file, new_path)
                renamed_count += 1
        
        print(f"\n重命名完成！")
        print(f"共处理了 {renamed_count} 个文件")
        return True
        
    except Exception as e:
        print(f"重命名过程中发生错误: {e}")
        return False

def main():
    """
    主函数：批量重命名PCD文件
    """
    # 配置参数
    input_dirs = [
        "/mnt/d/rosbag/dynamic/07/pcds",  # PCD文件目录
        # 如果需要处理更多目录，可以在这里添加
    ]
    
    print("=== PCD文件批量重命名工具 ===")
    print("将把5位数字格式(00000.pcd)改为6位数字格式(000000.pcd)")
    print("="*50)
    
    # 处理每个目录
    for input_dir in input_dirs:
        print(f"\n处理目录: {input_dir}")
        success = rename_pcd_files(input_dir)
        
        if success:
            print(f"目录 {input_dir} 处理完成")
        else:
            print(f"目录 {input_dir} 处理失败")

if __name__ == "__main__":
    main()